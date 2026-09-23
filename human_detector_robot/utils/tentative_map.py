"""
Tentative Map & MPU/Ultrasonic Movement Trajectory Logger.

Features:
  1. 2D ASCII Grid Map (maps/tentative_map.txt):
     Legend: S=start  .=robot path  *=human  #=wall  @=current robot pos
  2. Movement & Direction Action Log (maps/robot_movement_log.txt):
     Records:
       - Distance moved in centimeters (dead-reckoning + ultrasonic verification)
       - Degrees turned via MPU6050 gyroscope yaw
       - Wall encounters via Front/Left ultrasonic sensors
       - Human detection coordinates and posture
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

HumanSource = Literal["cv", "wifi", "cv_standing", "cv_fallen"]


@dataclass
class TentativeMap:
    cell_size_m: float = 0.05
    width: int = 80
    height: int = 40
    output_path: Path = field(default_factory=lambda: Path("maps/tentative_map.txt"))
    trajectory_path: Path = field(default_factory=lambda: Path("maps/robot_movement_log.txt"))
    robot_id: int = 1

    # Robot pose (meters, world frame)
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0
    _last_t: float = field(default_factory=time.time)

    # Movement metrics
    total_distance_cm: float = 0.0
    total_turns_deg: float = 0.0
    turn_count: int = 0
    human_count: int = 0

    # Last logged anchors for significant movement thresholding
    _last_logged_x: float = 0.0
    _last_logged_y: float = 0.0
    _last_logged_yaw: float = 0.0
    _last_state: str = ""

    # Grid cells: (gx, gy) -> char
    _cells: dict[tuple[int, int], str] = field(default_factory=dict)
    _events: list[str] = field(default_factory=list)
    _movement_log: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.trajectory_path.parent.mkdir(parents=True, exist_ok=True)
        gx, gy = self._world_to_grid(0.0, 0.0)
        self._cells[(gx, gy)] = "S"
        self._log_action("START", "Robot started at origin (0, 0)", delta_str="0 cm", sonar_str="Initial")

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        cx = self.width // 2
        cy = self.height - 3
        gx = int(round(x / self.cell_size_m)) + cx
        gy = cy - int(round(y / self.cell_size_m))
        return gx, gy

    def _set(self, gx: int, gy: int, ch: str, overwrite: bool = False) -> None:
        if 0 <= gx < self.width and 0 <= gy < self.height:
            if overwrite or (gx, gy) not in self._cells or self._cells[(gx, gy)] == ".":
                self._cells[(gx, gy)] = ch

    def _angle_diff(self, a: float, b: float) -> float:
        return (a - b + 180.0) % 360.0 - 180.0

    def _log_action(self, action: str, details: str, delta_str: str = "", sonar_str: str = "") -> None:
        ts = time.strftime("%H:%M:%S")
        x_cm = self.x * 100.0
        y_cm = self.y * 100.0
        pose_str = f"({x_cm:6.1f}cm, {y_cm:6.1f}cm, {self.yaw_deg:+6.1f}°)"
        entry = f"[{ts}] {action:14} | {delta_str:12} | {details:32} | {pose_str} | {sonar_str}"
        self._movement_log.append(entry)

    def update_pose(
        self,
        yaw_deg: float,
        v: float,
        omega: float,
        dist_front_cm: int = -1,
        dist_left_cm: int = -1,
        state_name: str = "CRUISE",
    ) -> None:
        """Integrate movement from commanded velocity + continuous MPU yaw + sonars."""
        now = time.time()
        dt = min(now - self._last_t, 0.25)
        self._last_t = now

        prev_yaw = self.yaw_deg
        self.yaw_deg = yaw_deg
        yaw_rad = math.radians(yaw_deg)

        # Dead reckoning displacement
        ds_m = v * dt
        ds_cm = abs(ds_m) * 100.0
        self.total_distance_cm += ds_cm

        self.x += ds_m * math.cos(yaw_rad)
        self.y += ds_m * math.sin(yaw_rad)

        gx, gy = self._world_to_grid(self.x, self.y)
        self._set(gx, gy, ".")

        # Sonar status string
        sonar_str = f"F:{dist_front_cm:3d}cm, L:{dist_left_cm:3d}cm"

        # Check turn threshold (turned >= 15 degrees from last logged yaw)
        d_yaw = self._angle_diff(self.yaw_deg, self._last_logged_yaw)
        if abs(d_yaw) >= 15.0:
            turn_dir = "LEFT" if d_yaw > 0 else "RIGHT"
            self.total_turns_deg += abs(d_yaw)
            self.turn_count += 1
            self._log_action(
                f"TURN_{turn_dir}",
                f"Turned {abs(d_yaw):.1f}° {turn_dir} via MPU (State: {state_name})",
                delta_str=f"{d_yaw:+.1f}°",
                sonar_str=sonar_str,
            )
            self._last_logged_yaw = self.yaw_deg

        # Check distance threshold (moved >= 20 cm from last logged position)
        dx = (self.x - self._last_logged_x) * 100.0
        dy = (self.y - self._last_logged_y) * 100.0
        dist_since_log = math.hypot(dx, dy)
        if dist_since_log >= 20.0:
            direction = "FORWARD" if v >= 0 else "REVERSE"
            self._log_action(
                f"MOVE_{direction}",
                f"Advanced {dist_since_log:.1f} cm (State: {state_name})",
                delta_str=f"+{dist_since_log:.1f} cm",
                sonar_str=sonar_str,
            )
            self._last_logged_x = self.x
            self._last_logged_y = self.y

        # Check state transition
        if state_name != self._last_state and state_name in ("STOP_AT_WALL", "CREEP_TO_SCAN", "SCAN_ROTATE"):
            self._log_action(
                f"STATE_{state_name[:8]}",
                f"Obstacle Encounter -> {state_name}",
                delta_str=f"F={dist_front_cm}cm",
                sonar_str=sonar_str,
            )
            self._last_state = state_name

    def mark_wall_from_ultrasonic(self, dist_cm: int, yaw_deg: float | None = None) -> None:
        """Place wall cells along sonar ray."""
        if dist_cm <= 0 or dist_cm > 400:
            return
        yaw = math.radians(yaw_deg if yaw_deg is not None else self.yaw_deg)
        dist_m = dist_cm / 100.0
        wx = self.x + dist_m * math.cos(yaw)
        wy = self.y + dist_m * math.sin(yaw)
        gx, gy = self._world_to_grid(wx, wy)

        # Wall segment perpendicular to ray
        perp = yaw + math.pi / 2
        for i in range(-2, 3):
            ox = int(round(i * math.cos(perp)))
            oy = int(round(i * math.sin(perp)))
            self._set(gx + ox, gy + oy, "#", overwrite=True)

    def mark_human(
        self,
        source: HumanSource,
        note: str = "",
        offset_m: float = 0.0,
    ) -> None:
        """Mark human at current heading (offset forward for wifi-behind-wall)."""
        yaw_rad = math.radians(self.yaw_deg)
        hx = self.x + offset_m * math.cos(yaw_rad)
        hy = self.y + offset_m * math.sin(yaw_rad)
        gx, gy = self._world_to_grid(hx, hy)
        self._set(gx, gy, "*", overwrite=True)

        self.human_count += 1
        ts = time.strftime("%H:%M:%S")
        event_str = f"[{ts}] HUMAN ({source}) at (X={self.x*100:.1f}cm, Y={self.y*100:.1f}cm) {note}"
        self._events.append(event_str)

        self._log_action(
            "HUMAN_DETECTED",
            f"Target ({source}) {note[:20]}",
            delta_str="*** ALERT ***",
            sonar_str=f"Map: ({hx*100:.0f}cm, {hy*100:.0f}cm)",
        )

    def render_map(self) -> str:
        gx_r, gy_r = self._world_to_grid(self.x, self.y)
        lines: list[str] = [
            "================================================================================",
            f"  ARK-5 2D TENTATIVE MAP — ROBOT #{self.robot_id}",
            "  Legend: S=start  .=robot path  @=current robot  #=wall  *=human detected",
            f"  Current Pose: X={self.x*100:.1f} cm, Y={self.y*100:.1f} cm, Heading={self.yaw_deg:+.1f}° (MPU)",
            f"  Total Traveled: {self.total_distance_cm:.1f} cm | Humans Found: {self.human_count}",
            f"  Updated: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "================================================================================",
            "",
        ]
        if self._events:
            lines.append("# Human Detections & Critical Events:")
            for ev in self._events[-8:]:
                lines.append(f"# {ev}")
            lines.append("")

        grid: list[list[str]] = [[" " for _ in range(self.width)] for _ in range(self.height)]
        for (gx, gy), ch in self._cells.items():
            if 0 <= gx < self.width and 0 <= gy < self.height:
                grid[gy][gx] = ch
        if 0 <= gx_r < self.width and 0 <= gy_r < self.height:
            grid[gy_r][gx_r] = "@"

        lines.extend("".join(row).rstrip() for row in grid)
        return "\n".join(lines) + "\n"

    def render_trajectory_log(self) -> str:
        lines: list[str] = [
            "===========================================================================================",
            f"  ARK-5 ROBOT MOVEMENT & DIRECTION LOG (MPU6050 + ULTRASONIC DEAD-RECKONING)",
            f"  Robot ID              : #{self.robot_id}",
            f"  Total Distance Moved  : {self.total_distance_cm:7.1f} cm  ({self.total_distance_cm/100.0:.2f} meters)",
            f"  Total Turns Executed  : {self.turn_count} turns  (Cumulative rotation: {self.total_turns_deg:.1f}°)",
            f"  Current Heading (MPU) : {self.yaw_deg:+7.1f}°",
            f"  Current Position      : X = {self.x*100:6.1f} cm,  Y = {self.y*100:6.1f} cm",
            f"  Humans Located        : {self.human_count}",
            f"  Last Log Time         : {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "===========================================================================================",
            "[TIME]   | ACTION         | DELTA        | DETAILS                          | POSE (X, Y, YAW)          | SONAR",
            "---------|----------------|--------------|----------------------------------|---------------------------|-------------------",
        ]
        lines.extend(self._movement_log[-100:])  # keep last 100 entries
        return "\n".join(lines) + "\n"

    def save(self) -> tuple[Path, Path]:
        """Saves both the ASCII grid map and the detailed direction trajectory log."""
        map_text = self.render_map()
        self.output_path.write_text(map_text, encoding="utf-8")

        traj_text = self.render_trajectory_log()
        self.trajectory_path.write_text(traj_text, encoding="utf-8")

        return self.output_path, self.trajectory_path
