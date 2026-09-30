"""
Tentative Map & MPU/Ultrasonic Movement Trajectory Logger.

Features:
  1. 2D ASCII Grid Map (maps/tentative_map.txt):
     Legend: S=start  .=robot path  *=human confirmed  #=wall
             ?=wifi hint (unconfirmed)  @=current robot pos
  2. Movement & Direction Action Log (maps/robot_movement_log.txt):
     Records:
       - Distance moved in centimeters (dead-reckoning + ultrasonic verification)
       - Degrees turned via MPU6050 gyroscope yaw
       - Wall encounters via Front/Left ultrasonic sensors
       - Human detection coordinates and posture

Algorithm Overview:
  - The robot's pose (x, y, yaw) is tracked in world-frame meters.
  - World coordinates are converted to grid cells using a configurable cell size.
  - Dead-reckoning integrates commanded velocity over elapsed time to update
    position — but ONLY when the motion is plausible (see TASK-03 below).
  - Significant movements (>=20cm) and turns (>=15deg) are logged as discrete events.
  - Ultrasonic readings place wall segments (#) on the grid perpendicular to the ray.
  - Human detections are marked '*' (confirmed) or '?' (wifi hint) on the grid.

TASK-03 (Honest Dead Reckoning):
  Position only advances when failsafe is off, velocity is non-zero, AND the
  front-sonar trend agrees with the commanded motion direction. This prevents
  map pins from "walking" while the robot is paused, failsafe'd, or stuck
  with spinning wheels.
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
        yaw_deg: Robot's current heading in degrees (0=+X axis, CCW positive).
        _last_t: Timestamp of the last pose update (for dt calculation).

        total_distance_cm: Cumulative distance traveled in centimeters.
        total_turns_deg: Cumulative rotation in degrees across all turns.
        turn_count: Number of discrete turn events logged.
        human_count: Number of CONFIRMED human detections logged (hints excluded).

        _last_logged_x: X position at last significant movement log (for thresholding).
        _last_logged_y: Y position at last significant movement log (for thresholding).
        _last_logged_yaw: Yaw at last significant turn log (for thresholding).
        _last_state: Last FSM state name (for state-transition logging).

        _last_dist_front_cm: TASK-03 — last valid front-sonar reading, used to
            verify that commanded motion matches what the ultrasonic observes.
        frozen_last_tick: TASK-03 — True when position was frozen last tick
            (failsafe, zero velocity, or implausible motion).

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

    # TASK-03: last front-sonar reading, used to sanity-check that
    # commanded motion actually matches what the ultrasonic sees
    # TASK-03: dedicated anchors for the 10cm plausibility check
    _anchor_x: float = 0.0
    _anchor_y: float = 0.0
    _anchor_dist_cm: int = -1
    frozen_last_tick: bool = False

    # Grid cells: (gx, gy) -> char
    _cells: dict[tuple[int, int], str] = field(default_factory=dict)
    _events: list[str] = field(default_factory=list)
    _movement_log: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Initialize map directories, mark start position, log START event.

        Algorithm:
          1. Create parent directories for the map and trajectory files.
          2. Convert world origin (0, 0) to grid coordinates.
          3. Mark that cell with 'S'.
          4. Append an initial START entry to the trajectory log.
        """
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.trajectory_path.parent.mkdir(parents=True, exist_ok=True)
        gx, gy = self._world_to_grid(0.0, 0.0)
        self._cells[(gx, gy)] = "S"
        self._log_action("START", "Robot started at origin (0, 0)", delta_str="0 cm", sonar_str="Initial")

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world-frame coordinates (meters) to grid cell indices.

        Args:
            x: World X coordinate in meters.
            y: World Y coordinate in meters.

        Returns:
            Tuple of (grid_column, grid_row) integer indices.

        Algorithm:
          1. Grid horizontal center is at column width//2 (robot starts centered).
          2. Grid vertical reference is at row height-3 (leaves header room).
          3. X is scaled by cell_size_m and offset from the center column.
          4. Y is scaled by cell_size_m and inverted (world +Y = grid up = smaller row).
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
            ch: Character to place ('.', '#', '*', '?', 'S', etc.).
            overwrite: If True, overwrites any existing character.
                       If False, only overwrites empty/path ('.') cells.

        Algorithm:
          1. Bounds-check gx and gy against grid dimensions.
          2. If overwrite=True, unconditionally set the cell.
          3. If overwrite=False, only set if cell is empty or currently '.'.
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
            Signed difference in degrees in [-180, +180). Positive means 'a'
            is counterclockwise of 'b'.

        Algorithm:
          (a - b + 180) % 360 - 180 wraps the difference into the shortest
          turning direction.
        """
        return (a - b + 180.0) % 360.0 - 180.0

    def _log_action(self, action: str, details: str, delta_str: str = "", sonar_str: str = "") -> None:
        """Append a formatted entry to the movement action log.

        Args:
            action: Short action label (e.g. 'TURN_LEFT', 'MOVE_FORWARD').
            details: Human-readable description of what happened.
            delta_str: Numeric delta string (e.g. '+25.3 cm', '+45.0°').
            sonar_str: Ultrasonic readings string (e.g. 'F:120cm, L:80cm').

        Algorithm:
          1. Format a timestamp (HH:MM:SS).
          2. Convert pose to centimeters for readability.
          3. Build a fixed-width pipe-delimited log line.
          4. Append it to _movement_log.
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
        failsafe_active: bool = False,
    ) -> None:
        """Integrate movement from commanded velocity + continuous MPU yaw + sonars.

        TASK-03 (Honest Dead Reckoning): the OLD version moved (x, y)
        purely from `v * dt`, even if the robot was paused, failsafe'd,
        or physically stuck (wheels spinning, not actually advancing) --
        so a map pin could "walk" across the map during ALERT_PAUSE.

        Now (x, y) only advances when ALL of these hold:
          (a) failsafe is not active,
          (b) the robot is actually being commanded to move (|v| > 0),
          (c) the front sonar trend agrees with that motion. We track an
              anchor over 10cm of expected travel. If the sonar doesn't
              shrink by at least 2cm over that 10cm of forward travel, we
              assume the robot is physically stuck and freeze position.
        Yaw and logging still update every tick regardless.

        Args:
            yaw_deg: Current heading from MPU6050 gyroscope in degrees.
            v: Commanded linear velocity in m/s (positive=forward).
            omega: Commanded angular velocity in rad/s (not used for pose;
                   MPU yaw is authoritative).
            dist_front_cm: FILTERED front sonar distance in cm (-1 = invalid).
            dist_left_cm: FILTERED left sonar distance in cm (-1 = invalid).
            state_name: Current FSM state name for context in logs.
            failsafe_active: True when telemetry is stale / motors force-stopped.

        Algorithm:
          1. Compute dt since last call, capped at 0.25s to prevent jumps.
          2. Store yaw as the new heading; convert to radians.
          3. TASK-03 plausibility check: track an anchor position. When the
             expected motion reaches 10cm, verify that the front sonar
             decreased (if moving forward) by at least 2cm. Contradictions
             (e.g. didn't shrink) mark the motion implausible and freeze
             the map, allowing the stuck watchdog to rescue the robot.
          4. can_move = not failsafe AND |v| > 0 AND motion plausible;
             record frozen_last_tick = not can_move.
          5. If can_move: ds = v*dt; accumulate total_distance_cm; advance
             (x, y) by ds projected on yaw; mark the grid cell as path '.'.
          6. Log a TURN event when |yaw change| since last logged >= 15°.
          7. Log a MOVE event when position moved >= 20cm since last log.
          8. Log a STATE event when entering STOP_AT_WALL/CREEP_TO_SCAN/SCAN_ROTATE.
        """
        now = time.time()
        dt = min(now - self._last_t, 0.25)
        self._last_t = now

        self.yaw_deg = yaw_deg
        yaw_rad = math.radians(yaw_deg)

        # TASK-03: verify commanded motion against sonar trend
        # Instead of a tick-to-tick check (which is always ~0cm and fails to catch
        # a stuck robot), we anchor the pose and wait for 10cm of expected travel.
        # If the sonar doesn't shrink by at least 2cm over that 10cm, we are stuck.
        motion_plausible = True
        
        # Reset anchor if not moving linearly, BUT only if we actually turned!
        # If we are stuck, the watchdog will command a recovery turn (v=0, omega>0).
        # If the physical robot doesn't turn, yaw won't change. We shouldn't reset
        # the anchor unless the yaw actually changed, otherwise we'll accumulate
        # another 10cm of fake distance after every watchdog trigger!
        yaw_changed = True
        if hasattr(self, '_anchor_yaw'):
            diff = (yaw_deg - self._anchor_yaw + 180) % 360 - 180
            yaw_changed = abs(diff) > 5.0
            
        if (abs(v) < 1e-6 and yaw_changed) or failsafe_active:
            self._anchor_dist_cm = dist_front_cm
            self._anchor_x = self.x
            self._anchor_y = self.y
            self._anchor_yaw = yaw_deg
            
        if dist_front_cm > 0 and self._anchor_dist_cm > 0:
            expected_dist_cm = math.hypot(self.x - self._anchor_x, self.y - self._anchor_y) * 100.0
            if expected_dist_cm >= 10.0:
                delta = dist_front_cm - self._anchor_dist_cm
                if v > 0 and delta > -2.0:
                    motion_plausible = False  # told to go forward 10cm, but sonar didn't shrink by even 2cm
                elif v < 0 and delta < 2.0:
                    motion_plausible = False  # told to reverse 10cm, but sonar didn't grow by even 2cm
                    
                if motion_plausible:
                    # Real motion! Re-anchor for the next 10cm segment.
                    self._anchor_dist_cm = dist_front_cm
                    self._anchor_x = self.x
                    self._anchor_y = self.y
                    self._anchor_yaw = yaw_deg
        elif dist_front_cm > 0:
            self._anchor_dist_cm = dist_front_cm
            self._anchor_x = self.x
            self._anchor_y = self.y
            self._anchor_yaw = yaw_deg

        can_move = (not failsafe_active) and abs(v) > 1e-6 and motion_plausible
        self.frozen_last_tick = not can_move

        if can_move:
            ds_m = v * dt
            ds_cm = abs(ds_m) * 100.0
            self.total_distance_cm += ds_cm
            self.x += ds_m * math.cos(yaw_rad)
            self.y += ds_m * math.sin(yaw_rad)
            gx, gy = self._world_to_grid(self.x, self.y)
            self._set(gx, gy, ".")

        # Sonar status string used in log entries below
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
          2. Compute wall position = robot_pos + dist * heading_vector.
          3. Convert wall position to grid coordinates.
          4. Draw '#' in a 5-cell line perpendicular to the ray (2 cells each
             side plus center) to represent a wall segment, overwriting freely.
        """
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
        override_xy: tuple[float, float] | None = None,
        is_hint: bool = False,
    ) -> None:
        """Mark human at current heading (offset forward for wifi-behind-wall).

        TASK-09/TASK-10 additions:
          - `override_xy`: if given, use this exact (x, y) world position
            instead of computing it from current pose+yaw+offset. This is
            how HumanTracker (TASK-09) places a de-duplicated pin.
          - `is_hint`: TASK-10 -- a WiFi-only hint is drawn as '?' and does
            NOT increment human_count (it isn't a confirmed detection),
            so the rescue log doesn't get spammed with unconfirmed pings.

        Args:
            source: Detection source — 'cv', 'wifi', 'cv_standing', 'cv_fallen'.
            note: Optional annotation string (posture details, confidence).
            offset_m: Forward offset in meters from current pose along heading.
            override_xy: Exact world (x, y) to mark, bypassing pose+offset math.
            is_hint: If True, draw '?' and skip counting/logging (unconfirmed).

        Algorithm:
          1. Position = override_xy if given, else pose + offset * heading.
          2. Convert to grid and place '*' (confirmed) or '?' (hint).
          3. If is_hint: return early — visible on map but not counted.
          4. Otherwise: increment human_count, append a timestamped event to
             _events, and log a HUMAN_DETECTED action to the trajectory log.
        """
        if override_xy is not None:
            hx, hy = override_xy
        else:
            yaw_rad = math.radians(self.yaw_deg)
            hx = self.x + offset_m * math.cos(yaw_rad)
            hy = self.y + offset_m * math.sin(yaw_rad)
        gx, gy = self._world_to_grid(hx, hy)
        glyph = "?" if is_hint else "*"
        self._set(gx, gy, glyph, overwrite=True)

        if is_hint:
            # Hints are visible on the map but not counted as confirmed
            # rescues -- avoids the "18 pings for one person" spam.
            return

        self.human_count += 1
        ts = time.strftime("%H:%M:%S")
        event_str = f"[{ts}] HUMAN ({source}) at (X={hx*100:.1f}cm, Y={hy*100:.1f}cm) {note}"
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
            Multi-line string with header (pose, stats, legend), the last 8
            human events, and the grid with all markers plus '@' at the robot.

        Algorithm:
          1. Build header lines: title, legend (including '?' hint glyph),
             current pose, distance/human totals, timestamp.
          2. Append the last 8 human detection events as '#' comment lines.
          3. Create a space-filled width x height grid.
          4. Stamp all recorded cells (path, wall, human, hint, start).
          5. Overlay the current robot position with '@'.
          6. Join rows into one newline-terminated string.
        """
        gx_r, gy_r = self._world_to_grid(self.x, self.y)
        lines: list[str] = [
            "================================================================================",
            f"  ARK-5 2D TENTATIVE MAP — ROBOT #{self.robot_id}",
            "  Legend: S=start  .=robot path  @=current robot  #=wall  *=human confirmed  ?=wifi hint (unconfirmed)",
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
        """Render the movement/action trajectory log as a formatted string.

        Returns:
            Multi-line string with a summary header (totals, heading,
            position, human count) plus the last 100 log entries in a
            pipe-delimited table.

        Algorithm:
          1. Build the summary header block.
          2. Add column headers for the table.
          3. Append the last 100 entries of _movement_log.
          4. Join into one newline-terminated string.
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
        lines.extend(self._movement_log[-100:])  # keep last 100 entries
        return "\n".join(lines) + "\n"

    def save(self) -> tuple[Path, Path]:
        """Save both the ASCII grid map and the trajectory log to their files.

        Returns:
            Tuple of (map_file_path, trajectory_file_path) that were written.

        Algorithm:
          1. Render the map string and write it to output_path (UTF-8).
          2. Render the trajectory log string and write it to trajectory_path.
          3. Return both paths so the caller can log them.
        """
        map_text = self.render_map()
        self.output_path.write_text(map_text, encoding="utf-8")

        traj_text = self.render_trajectory_log()
        self.trajectory_path.write_text(traj_text, encoding="utf-8")

        return self.output_path, self.trajectory_path
