"""
Tentative Map & MPU/Ultrasonic Movement Trajectory Logger.

This module provides a 2D ASCII grid map and a detailed movement/action log
for tracking the robot's exploration path, wall encounters, and human detections
in real time. The map is written to `maps/tentative_map.txt` and the trajectory
log to `maps/robot_movement_log.txt`.

Features:
  1. 2D ASCII Grid Map (maps/tentative_map.txt):
     Legend: S=start  .=robot path  *=human  #=wall  @=current robot pos
  2. Movement & Direction Action Log (maps/robot_movement_log.txt):
     Records:
       - Distance moved in centimeters (dead-reckoning + ultrasonic verification)
       - Degrees turned via MPU6050 gyroscope yaw
       - Wall encounters via Front/Left ultrasonic sensors
       - Human detection coordinates and posture

Algorithm Overview:
  - The robot's pose (x, y, yaw) is tracked in world-frame meters.
  - World coordinates are converted to grid cells using a configurable cell size.
  - Dead-reckoning integrates commanded velocity over elapsed time to update position.
  - Significant movements (>=20cm) and turns (>=15deg) are logged as discrete events.
  - Ultrasonic readings place wall segments (#) on the grid perpendicular to the ray.
  - Human detections are marked with '*' on the grid at the reported offset.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

# Type alias for the source of a human detection (camera, wifi, or posture-specific).
HumanSource = Literal["cv", "wifi", "cv_standing", "cv_fallen"]


@dataclass
class TentativeMap:
    """2D ASCII grid map + movement trajectory logger for robot exploration.

    Attributes:
        cell_size_m: Size of each grid cell in meters (default 0.05m = 5cm).
        width: Width of the ASCII grid in characters.
        height: Height of the ASCII grid in characters.
        output_path: File path for the rendered ASCII grid map.
        trajectory_path: File path for the movement/action trajectory log.
        robot_id: Identifier for this robot (used in map headers).

        x: Robot's current X position in meters (world frame, right-positive).
        y: Robot's current Y position in meters (world frame, forward-positive).
        yaw_deg: Robot's current heading in degrees (0=East, CCW positive).
        _last_t: Timestamp of the last pose update (for dt calculation).

        total_distance_cm: Cumulative distance traveled in centimeters.
        total_turns_deg: Cumulative rotation in degrees across all turns.
        turn_count: Number of discrete turn events logged.
        human_count: Number of human detections logged.

        _last_logged_x: X position at last significant movement log (for thresholding).
        _last_logged_y: Y position at last significant movement log (for thresholding).
        _last_logged_yaw: Yaw at last significant turn log (for thresholding).
        _last_state: Last FSM state name (for state-transition logging).

        _cells: Dictionary mapping (gx, gy) grid coordinates to character symbols.
        _events: List of human detection event strings (for map header display).
        _movement_log: List of formatted movement/action log entries.
    """

    # Grid configuration
    cell_size_m: float = 0.05       # meters per cell
    width: int = 80                  # grid width in characters
    height: int = 40                 # grid height in characters
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
        """Initialize map directories and mark the start position.

        Algorithm:
          1. Create output directories if they don't exist.
          2. Convert world origin (0,0) to grid coordinates.
          3. Mark the start cell with 'S'.
          4. Log the initial START action to the trajectory log.
        """
        # Ensure maps/ directory exists
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.trajectory_path.parent.mkdir(parents=True, exist_ok=True)
        # Convert world origin to grid and mark start
        gx, gy = self._world_to_grid(0.0, 0.0)
        self._cells[(gx, gy)] = "S"
        self._log_action("START", "Robot started at origin (0, 0)", delta_str="0 cm", sonar_str="Initial")

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world-frame coordinates (meters) to grid cell indices.

        Algorithm:
          1. The grid center-x is at column width//2 (robot starts centered horizontally).
          2. The grid bottom-y is at row height-3 (leaves room for header text).
          3. X is scaled by cell_size_m and offset from center column.
          4. Y is scaled by cell_size_m and inverted (world +Y = grid up = lower row index).

        Args:
            x: World X coordinate in meters.
            y: World Y coordinate in meters.

        Returns:
            Tuple of (grid_column, grid_row) integer indices.
        """
        cx = self.width // 2
        cy = self.height - 3
        gx = int(round(x / self.cell_size_m)) + cx
        gy = cy - int(round(y / self.cell_size_m))
        return gx, gy

    def _set(self, gx: int, gy: int, ch: str, overwrite: bool = False) -> None:
        """Place a character on the grid at the given cell coordinates.

        Args:
            gx: Grid column index.
            gy: Grid row index.
            ch: Character to place ('.', '#', '*', 'S', etc.).
            overwrite: If True, overwrites any existing character.
                       If False, only overwrites '.' (path) or empty cells.

        Algorithm:
          1. Bounds-check gx and gy against grid dimensions.
          2. If overwrite=True, unconditionally set the cell.
          3. If overwrite=False, only set if cell is empty or currently a path marker '.'.
        """
        if 0 <= gx < self.width and 0 <= gy < self.height:
            if overwrite or (gx, gy) not in self._cells or self._cells[(gx, gy)] == ".":
                self._cells[(gx, gy)] = ch

    def _angle_diff(self, a: float, b: float) -> float:
        """Compute the shortest signed angular difference between two angles.

        Args:
            a: First angle in degrees.
            b: Second angle in degrees.

        Returns:
            Signed difference in degrees in the range [-180, +180].
            Positive means 'a' is counterclockwise from 'b'.

        Algorithm:
          Uses the formula (a - b + 180) % 360 - 180 to wrap the difference
          into the [-180, 180] range, giving the shortest turning direction.
        """
        return (a - b + 180.0) % 360.0 - 180.0

    def _log_action(self, action: str, details: str, delta_str: str = "", sonar_str: str = "") -> None:
        """Append a formatted entry to the movement action log.

        Args:
            action: Short action label (e.g. 'TURN_LEFT', 'MOVE_FORWARD', 'HUMAN_DETECTED').
            details: Human-readable description of what happened.
            delta_str: Numeric delta string (e.g. '+25.3 cm', '+45.0°').
            sonar_str: Ultrasonic sensor readings string (e.g. 'F:120cm, L:80cm').

        Algorithm:
          1. Get current timestamp as HH:MM:SS.
          2. Convert robot pose to centimeters for readability.
          3. Format a fixed-width pipe-delimited log line.
          4. Append to the internal _movement_log list.
        """
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
        """Integrate movement from commanded velocity + continuous MPU yaw + sonars.

        This is the main update method called every control tick. It performs
        dead-reckoning pose integration and logs significant movements/turns.

        Args:
            yaw_deg: Current heading from MPU6050 gyroscope in degrees.
            v: Commanded linear velocity in m/s (positive=forward, negative=reverse).
            omega: Commanded angular velocity in rad/s (not directly used for pose here;
                   yaw_deg from MPU is authoritative).
            dist_front_cm: Front ultrasonic distance in cm (-1 if unavailable).
            dist_left_cm: Left ultrasonic distance in cm (-1 if unavailable).
            state_name: Current FSM state name for context in logs.

        Algorithm:
          1. Compute dt from last update, capped at 0.25s to prevent jumps.
          2. Update yaw from MPU (authoritative heading source).
          3. Dead-reckon: ds = v * dt, then dx = ds*cos(yaw), dy = ds*sin(yaw).
          4. Accumulate total_distance_cm.
          5. Mark the new grid cell as path '.'.
          6. If yaw changed >= 15° from last logged yaw, log a TURN event.
          7. If position moved >= 20cm from last logged position, log a MOVE event.
          8. If FSM state changed to an obstacle-related state, log a STATE event.
        """
        # Compute time delta, capped to prevent large jumps on lag
        now = time.time()
        dt = min(now - self._last_t, 0.25)
        self._last_t = now

        # Store previous yaw and update to current MPU reading
        prev_yaw = self.yaw_deg
        self.yaw_deg = yaw_deg
        yaw_rad = math.radians(yaw_deg)

        # Dead reckoning: integrate velocity over time to get displacement
        ds_m = v * dt
        ds_cm = abs(ds_m) * 100.0
        self.total_distance_cm += ds_cm

        # Update position in world frame
        self.x += ds_m * math.cos(yaw_rad)
        self.y += ds_m * math.sin(yaw_rad)

        # Mark current position on the grid as a path cell
        gx, gy = self._world_to_grid(self.x, self.y)
        self._set(gx, gy, ".")

        # Format sonar readings for log entries
        sonar_str = f"F:{dist_front_cm:3d}cm, L:{dist_left_cm:3d}cm"

        # Log turn event if yaw changed by >= 15 degrees since last logged turn
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

        # Log move event if position changed by >= 20cm since last logged move
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

        # Log state transition when entering obstacle-handling states
        if state_name != self._last_state and state_name in ("STOP_AT_WALL", "CREEP_TO_SCAN", "SCAN_ROTATE"):
            self._log_action(
                f"STATE_{state_name[:8]}",
                f"Obstacle Encounter -> {state_name}",
                delta_str=f"F={dist_front_cm}cm",
                sonar_str=sonar_str,
            )
            self._last_state = state_name

    def mark_wall_from_ultrasonic(self, dist_cm: int, yaw_deg: float | None = None) -> None:
        """Place wall cells along the sonar ray on the grid map.

        Args:
            dist_cm: Ultrasonic distance reading in centimeters.
            yaw_deg: Heading in degrees at which the reading was taken.
                     If None, uses the current robot yaw.

        Algorithm:
          1. Ignore invalid readings (<=0 or >400cm max range).
          2. Compute wall position: robot_pos + dist * direction_vector.
          3. Convert wall position to grid coordinates.
          4. Place '#' characters in a 5-cell line perpendicular to the sonar ray
             (2 cells each side + center) to represent a wall segment.
          5. Uses overwrite=True so walls always appear on the map.
        """
        if dist_cm <= 0 or dist_cm > 400:
            return
        yaw = math.radians(yaw_deg if yaw_deg is not None else self.yaw_deg)
        dist_m = dist_cm / 100.0
        wx = self.x + dist_m * math.cos(yaw)
        wy = self.y + dist_m * math.sin(yaw)
        gx, gy = self._world_to_grid(wx, wy)

        # Draw a 5-cell wall segment perpendicular to the sonar ray direction
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
        """Mark a human detection on the grid map at the current heading.

        Args:
            source: Detection source — 'cv' (computer vision), 'wifi' (RF sensing),
                    'cv_standing' (standing posture), or 'cv_fallen' (fallen posture).
            note: Optional annotation string (e.g. posture details, confidence).
            offset_m: Forward offset in meters from current position along heading.
                      Used for wifi-behind-wall detections where the human is
                      estimated to be further ahead.

        Algorithm:
          1. Compute human position = robot_pos + offset * heading_direction.
          2. Convert to grid coordinates and place '*' marker (overwrite=True).
          3. Increment human_count.
          4. Append a timestamped event string to _events list.
          5. Log a HUMAN_DETECTED action to the trajectory log.
        """
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
        """Render the complete ASCII grid map as a string.

        Returns:
            Multi-line string containing the formatted map with header info,
            recent human detection events, and the grid with all markers.

        Algorithm:
          1. Build header lines with robot ID, legend, current pose, and stats.
          2. Append the last 8 human detection events as comments.
          3. Create an empty (space-filled) grid of width x height.
          4. Place all recorded cells (path, wall, human, start) onto the grid.
          5. Overlay the current robot position with '@' symbol.
          6. Join all rows into a single string with newlines.
        """
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
        # Show last 8 human detection events in the map header
        if self._events:
            lines.append("# Human Detections & Critical Events:")
            for ev in self._events[-8:]:
                lines.append(f"# {ev}")
            lines.append("")

        # Build the visual grid from recorded cells
        grid: list[list[str]] = [[" " for _ in range(self.width)] for _ in range(self.height)]
        for (gx, gy), ch in self._cells.items():
            if 0 <= gx < self.width and 0 <= gy < self.height:
                grid[gy][gx] = ch
        # Mark current robot position with '@'
        if 0 <= gx_r < self.width and 0 <= gy_r < self.height:
            grid[gy_r][gx_r] = "@"

        lines.extend("".join(row).rstrip() for row in grid)
        return "\n".join(lines) + "\n"

    def render_trajectory_log(self) -> str:
        """Render the movement/action trajectory log as a formatted string.

        Returns:
            Multi-line string with a summary header and the last 100 log entries
            in a pipe-delimited table format.

        Algorithm:
          1. Build header with robot ID, total distance, turns, heading, position.
          2. Add column headers for the log table.
          3. Append the last 100 entries from _movement_log.
          4. Join into a single newline-terminated string.
        """
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
        # Show last 100 entries to keep log manageable
        lines.extend(self._movement_log[-100:])
        return "\n".join(lines) + "\n"

    def save(self) -> tuple[Path, Path]:
        """Save both the ASCII grid map and the trajectory log to their files.

        Returns:
            Tuple of (map_file_path, trajectory_file_path) that were written.

        Algorithm:
          1. Render the map string and write to output_path.
          2. Render the trajectory log string and write to trajectory_path.
          3. Return both paths for caller reference.
        """
        map_text = self.render_map()
        self.output_path.write_text(map_text, encoding="utf-8")

        traj_text = self.render_trajectory_log()
        self.trajectory_path.write_text(traj_text, encoding="utf-8")

        return self.output_path, self.trajectory_path
