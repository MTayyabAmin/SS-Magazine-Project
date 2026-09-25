"""
Autonomous Finite State Machine (FSM) for L-turn and T-junction path finding.

This module implements the robot's autonomous navigation decision-making using
a state machine. The robot has NO servo — it rotates its entire body in place
(v=0, omega!=0) to scan surroundings and choose the best path.

Corner / T-junction Flow:
  1. CRUISE forward until wall detected at 100cm
  2. STOP_AT_WALL — halt immediately
  3. CREEP_TO_SCAN — creep forward to 30-40cm standoff from wall
  4. WIFI_SCAN — perform WiFi human sensing at standoff position
  5. SCAN_ROTATE — spin in place: scan LEFT 90° then RIGHT 90°
     (sonar samples taken every 15°)
  6. TURN_TO_PATH — rotate to the best open heading
  7. CRUISE — resume forward movement on the new heading

Multi-Robot Support:
  - Each robot has an 'exploration_bias' (left or right) to diverge paths
    and maximize area coverage in swarm deployments.
  - SWARM_YIELD state prevents collisions when robots get too close.
"""

from __future__ import annotations

import math
from enum import Enum, auto


class RobotState(Enum):
    """Enumeration of all possible FSM states.

    Each state defines a distinct behavior mode for the robot:
      - CRUISE: Move forward at cruising speed.
      - STOP_AT_WALL: Halt when a wall is detected within safety zone.
      - CREEP_TO_SCAN: Slowly approach the wall to a standoff distance.
      - WIFI_SCAN: Pause to perform WiFi RSSI human sensing.
      - SCAN_ROTATE: Spin in place to scan left and right with sonar.
      - TURN_TO_PATH: Rotate to align with the chosen open path.
      - ALERT_PAUSE: Brief stop when a human is detected (CV or WiFi).
      - SWARM_YIELD: Yield/turn to avoid colliding with a peer robot.
    """
    CRUISE = auto()
    STOP_AT_WALL = auto()
    CREEP_TO_SCAN = auto()
    WIFI_SCAN = auto()
    SCAN_ROTATE = auto()    # robot spins in place — no servo
    TURN_TO_PATH = auto()
    ALERT_PAUSE = auto()
    SWARM_YIELD = auto()    # pause / yield to avoid crashing into peer robot


class AutonomousFSM:
    """Finite State Machine controlling autonomous robot navigation.

    The FSM processes sensor inputs each tick and returns velocity commands
    (v, omega) that the motor controller executes. All scanning is done by
    rotating the entire robot — there is no servo-mounted sensor.

    Attributes:
        state: Current RobotState.
        v_cruise: Forward velocity during normal cruising (m/s).
        v_creep: Forward velocity during close-approach creeping (m/s).
        omega_scan: Angular velocity during scan rotation (rad/s).
        wall_stop_cm: Front sonar distance to trigger wall stop (cm).
        scan_standoff_min_cm: Minimum standoff distance for scanning (cm).
        scan_standoff_max_cm: Maximum standoff distance for scanning (cm).
        open_path_cm: Sonar distance threshold to consider a path "open" (cm).
        scan_step_deg: Degrees between sonar samples during scan (deg).
        scan_side_deg: Total degrees to scan on each side (left/right) (deg).
        alert_pause_s: Duration to pause when human is detected (seconds).
        exploration_bias: Direction bias for multi-robot divergence ('left'/'right').
    """

    def __init__(
        self,
        v_cruise: float = 0.15,
        v_creep: float = 0.06,
        omega_scan: float = 0.35,
        wall_stop_cm: int = 100,
        scan_standoff_min_cm: int = 30,
        scan_standoff_max_cm: int = 40,
        open_path_cm: int = 120,
        scan_step_deg: float = 15.0,
        scan_side_deg: float = 90.0,
        alert_pause_s: float = 2.0,
        exploration_bias: str = "left",  # "left" for Robot 1, "right" for Robot 2
    ) -> None:
        """Initialize the FSM with navigation parameters.

        Args:
            v_cruise: Forward speed in m/s during normal cruising.
            v_creep: Forward speed in m/s during close-approach creeping.
            omega_scan: Rotational speed in rad/s during scan spin.
            wall_stop_cm: Front sonar distance (cm) that triggers a wall stop.
            scan_standoff_min_cm: Minimum distance from wall to begin scanning.
            scan_standoff_max_cm: Maximum distance; creep forward if beyond this.
            open_path_cm: Sonar distance (cm) to consider a direction "open".
            scan_step_deg: Degrees between consecutive sonar samples during scan.
            scan_side_deg: Degrees to scan on each side (left and right) from center.
            alert_pause_s: Seconds to pause when a human is detected.
            exploration_bias: 'left' or 'right' — which direction to prefer at
                             junctions for multi-robot area coverage.
        """
        self.state = RobotState.CRUISE
        self.v_cruise = v_cruise
        self.v_creep = v_creep
        self.omega_scan = omega_scan
        self.wall_stop_cm = wall_stop_cm
        self.scan_standoff_min = scan_standoff_min_cm
        self.scan_standoff_max = scan_standoff_max_cm
        self.open_path_cm = open_path_cm
        self.scan_step_deg = scan_step_deg
        self.scan_side_deg = scan_side_deg
        self.alert_pause_s = alert_pause_s
        self.exploration_bias = exploration_bias  # left or right priority

        # Internal state tracking
        self._state_start: float = 0.0          # timestamp when current state began
        self._scan_center_yaw: float = 0.0      # yaw at center before scan started
        self._scan_last_sample_yaw: float = 0.0 # yaw of last sonar sample taken
        self._scan_phase: str = "left"          # current scan phase: "left" or "right"
        self._scan_samples: list[tuple[float, int]] = []  # (relative_deg, distance_cm)
        self._target_heading: float | None = None  # heading to align to before cruising
        self.alert_message: str | None = None    # message to display during alert
        self.human_detected_this_stop: bool = False  # flag for human detection at this stop
        self.last_sonar_sample: int = -1         # most recent sonar reading
        self._pre_alert_state: RobotState = RobotState.CRUISE  # state before alert pause
        self.chosen_direction: str = ""          # human-readable chosen direction label

    def _elapsed(self, now: float) -> float:
        """Compute seconds elapsed since the current state was entered.

        Args:
            now: Current timestamp (time.time()).

        Returns:
            Elapsed time in seconds since state entry.
        """
        return now - self._state_start

    def _enter(self, state: RobotState, now: float) -> None:
        """Transition to a new state and record the entry timestamp.

        Args:
            state: The RobotState to transition to.
            now: Current timestamp (time.time()).
        """
        self.state = state
        self._state_start = now

    def _rel_yaw(self, yaw_deg: float) -> float:
        """Compute yaw in degrees relative to the scan center.

        Args:
            yaw_deg: Current absolute yaw in degrees.

        Returns:
            Relative angle in degrees: positive = left of center, negative = right.

        Algorithm:
          Uses _angle_diff to compute the signed difference between
          the current yaw and the scan center yaw.
        """
        return self._angle_diff(yaw_deg, self._scan_center_yaw)

    def _sample_sonar(self, yaw_deg: float, dist_cm: int) -> None:
        """Record a sonar sample during the scan phase if spacing is sufficient.

        Args:
            yaw_deg: Current yaw when the sonar reading was taken.
            dist_cm: Sonar distance reading in centimeters.

        Algorithm:
          1. Ignore invalid readings (dist_cm <= 0).
          2. Check if robot has rotated at least scan_step_deg since last sample.
          3. If so, record (relative_angle, distance) and update last sample yaw.
        """
        if dist_cm <= 0:
            return
        if abs(self._angle_diff(yaw_deg, self._scan_last_sample_yaw)) >= self.scan_step_deg:
            rel = self._rel_yaw(yaw_deg)
            self._scan_samples.append((rel, dist_cm))
            self._scan_last_sample_yaw = yaw_deg

    def _pick_best_path(self) -> tuple[float, str]:
        """Choose the best heading from collected sonar samples with exploration bias.

        Returns:
            Tuple of (heading_degrees, direction_label).

        Algorithm:
          1. Filter samples for "open" paths (distance >= open_path_cm).
          2. If open paths exist:
             a. Apply exploration bias: prefer left if bias='left', right if bias='right'.
             b. Among biased candidates, pick the one with maximum distance.
             c. If no biased candidates, fall back to any open path with max distance.
          3. If no open paths: pick the sample with maximum distance (least blocked).
          4. Compute absolute heading = scan_center_yaw + best_relative_angle.
          5. Label the direction as LEFT, RIGHT, FORWARD, or BEST AVAILABLE.
        """
        if not self._scan_samples:
            return self._scan_center_yaw, "none"

        # Filter for open paths (distance >= open_path_cm threshold)
        open_candidates = [s for s in self._scan_samples if s[1] >= self.open_path_cm]
        if open_candidates:
            # Apply multi-robot directional divergence via exploration bias
            if self.exploration_bias == "left":
                left_ops = [s for s in open_candidates if s[0] > 5]
                chosen = max(left_ops, key=lambda s: s[1]) if left_ops else max(open_candidates, key=lambda s: s[1])
            elif self.exploration_bias == "right":
                right_ops = [s for s in open_candidates if s[0] < -5]
                chosen = max(right_ops, key=lambda s: s[1]) if right_ops else max(open_candidates, key=lambda s: s[1])
            else:
                chosen = max(open_candidates, key=lambda s: s[1])
            best_rel, best_dist = chosen
        else:
            # No fully open paths — pick the least blocked direction
            best_rel, best_dist = max(self._scan_samples, key=lambda s: s[1])

        # Convert relative angle to absolute heading
        heading = (self._scan_center_yaw + best_rel) % 360
        if best_dist >= self.open_path_cm:
            if best_rel > 5:
                direction = f"LEFT ({self.exploration_bias} bias)"
            elif best_rel < -5:
                direction = f"RIGHT ({self.exploration_bias} bias)"
            else:
                direction = "FORWARD"
        else:
            direction = "BEST AVAILABLE (partial opening)"

        return heading, direction

    def note_sonar(self, dist_cm: int) -> None:
        """Store the most recent sonar reading for external access.

        Args:
            dist_cm: Front sonar distance in centimeters.
        """
        self.last_sonar_sample = dist_cm

    def step(
        self,
        now: float,
        dist_cm: int,
        dist_left: int,
        dist_right: int,
        wall_near: bool,
        yaw_deg: float,
        cv_alert: str | None,
        wifi_alert: str | None,
        peer_too_close: bool = False,
    ) -> tuple[float, float, str | None]:
        """Execute one FSM tick and return motor commands + status message.

        This is the main entry point called every control loop iteration.

        Args:
            now: Current timestamp (time.time()).
            dist_cm: Front ultrasonic distance in centimeters.
            dist_left: Left ultrasonic distance in centimeters.
            dist_right: Right ultrasonic distance in centimeters.
            wall_near: Boolean flag indicating a wall is in the safety zone.
            yaw_deg: Current heading from MPU6050 in degrees.
            cv_alert: Computer vision alert string (e.g. 'HUMAN_STANDING') or None.
            wifi_alert: WiFi sensing alert string or None.
            peer_too_close: True if swarm peer robot is within collision distance.

        Returns:
            Tuple of (v, omega, message):
              - v: Linear velocity command in m/s (positive=forward, negative=reverse).
              - omega: Angular velocity command in rad/s (positive=left/CCW).
              - message: Status string for HUD display, or None if no message.

        Algorithm (state-by-state):
          1. SWARM_YIELD: If peer is too close, turn in place to yield.
          2. ALERT_PAUSE: If human detected, pause for alert_pause_s seconds.
          3. CRUISE: Move forward until wall detected, then transition to STOP_AT_WALL.
          4. STOP_AT_WALL: Immediately transition to CREEP_TO_SCAN.
          5. CREEP_TO_SCAN: Creep toward wall until within standoff range [30-40cm].
          6. WIFI_SCAN: Pause for WiFi sensing, then check side sensors.
             If side sensor shows open path, go directly to TURN_TO_PATH.
             Otherwise, begin SCAN_ROTATE.
          7. SCAN_ROTATE: Spin left 90° sampling sonar, then spin right 90°.
             Take sonar samples every scan_step_deg. If open path found early,
             exit immediately to TURN_TO_PATH.
          8. TURN_TO_PATH: Rotate to align with the chosen heading using proportional
             control. When aligned within 8°, transition to CRUISE.
        """
        msg: str | None = None

        # --- SWARM YIELD: prevent collision with peer robot ---
        if peer_too_close:
            self._enter(RobotState.SWARM_YIELD, now)
            yield_turn = 0.35 if self.exploration_bias == "left" else -0.35
            return 0.0, yield_turn, "[SWARM] Peer robot nearby — yielding to prevent clash!"

        if self.state == RobotState.SWARM_YIELD:
            if not peer_too_close:
                self._enter(RobotState.CRUISE, now)
                return self.v_cruise, 0.0, "[SWARM] Path clear — resuming search"
            yield_turn = 0.35 if self.exploration_bias == "left" else -0.35
            return 0.0, yield_turn, "[SWARM] Still yielding to peer robot..."

        # --- ALERT PAUSE: brief stop when human detected by CV or WiFi ---
        if cv_alert or wifi_alert:
            if self.state != RobotState.ALERT_PAUSE:
                self.alert_message = cv_alert or wifi_alert
                self.human_detected_this_stop = True
                self._pre_alert_state = self.state
                self._enter(RobotState.ALERT_PAUSE, now)
                return 0.0, 0.0, self.alert_message

        if self.state == RobotState.ALERT_PAUSE:
            if self._elapsed(now) >= self.alert_pause_s:
                # Resume previous activity after pause
                if self._pre_alert_state in (RobotState.CRUISE, RobotState.STOP_AT_WALL):
                    self._enter(RobotState.CRUISE, now)
                    return self.v_cruise, 0.0, "[MAP] Human logged — resuming..."
                self._enter(RobotState.CREEP_TO_SCAN, now)
                return 0.0, 0.0, "[MAP] Human logged — continuing scan..."
            return 0.0, 0.0, None

        # --- CRUISE: move forward until wall detected ---
        if self.state == RobotState.CRUISE:
            if wall_near or (0 < dist_cm <= self.wall_stop_cm):
                self._enter(RobotState.STOP_AT_WALL, now)
                self.human_detected_this_stop = False
                return 0.0, 0.0, f"[SONAR] Wall at {dist_cm}cm — stopping"
            return self.v_cruise, 0.0, None

        # --- STOP_AT_WALL: immediate halt, then transition to creep ---
        if self.state == RobotState.STOP_AT_WALL:
            self._enter(RobotState.CREEP_TO_SCAN, now)
            return 0.0, 0.0, "[SCAN] Creep to 30-40cm (robot will rotate in place next)..."

        # --- CREEP_TO_SCAN: approach wall to standoff distance ---
        if self.state == RobotState.CREEP_TO_SCAN:
            if dist_cm < 0:
                return 0.0, 0.0, None  # no valid reading yet
            if self.scan_standoff_min <= dist_cm <= self.scan_standoff_max:
                # In position — begin WiFi scan
                self._enter(RobotState.WIFI_SCAN, now)
                return 0.0, 0.0, f"[SCAN] Standoff {dist_cm}cm OK — WiFi scan..."
            if dist_cm > self.scan_standoff_max:
                return self.v_creep, 0.0, None  # keep creeping forward
            return -self.v_creep * 0.5, 0.0, f"[SCAN] Too close ({dist_cm}cm) — reverse"

        # --- WIFI_SCAN: perform WiFi RSSI sensing, then decide scan strategy ---
        if self.state == RobotState.WIFI_SCAN:
            if self._elapsed(now) >= 0.8:
                # Record center yaw and initial sonar sample
                self._scan_center_yaw = yaw_deg
                self._scan_last_sample_yaw = yaw_deg
                self._scan_samples = [(0.0, dist_cm if dist_cm > 0 else 0)]

                # Quick check: if side sensors show open path, skip full scan
                if dist_left >= self.open_path_cm:
                    self._target_heading = yaw_deg + 90.0
                    self.chosen_direction = "LEFT (sonar)"
                    self._enter(RobotState.TURN_TO_PATH, now)
                    return 0.0, 0.0, f"[SCAN] LEFT path open {dist_left}cm — turning"
                if dist_right >= self.open_path_cm:
                    self._target_heading = yaw_deg - 90.0
                    self.chosen_direction = "RIGHT (sonar)"
                    self._enter(RobotState.TURN_TO_PATH, now)
                    return 0.0, 0.0, f"[SCAN] RIGHT path open {dist_right}cm — turning"

                # No quick path found — begin full rotational scan
                self._scan_phase = "left"
                self._enter(RobotState.SCAN_ROTATE, now)
                return 0.0, 0.0, "[SCAN] Side sensors blocked — robot rotating..."
            return 0.0, 0.0, None

        # --- SCAN_ROTATE: spin in place, scan left 90° then right 90° ---
        if self.state == RobotState.SCAN_ROTATE:
            self._sample_sonar(yaw_deg, dist_cm)
            rel = self._rel_yaw(yaw_deg)

            # Phase 1: Scan LEFT (positive relative angles)
            if self._scan_phase == "left":
                # Early exit if open path found on the left side
                if dist_cm >= self.open_path_cm and rel > 10:
                    self._target_heading = yaw_deg
                    self.chosen_direction = "LEFT"
                    self._enter(RobotState.TURN_TO_PATH, now)
                    return 0.0, 0.0, f"[SCAN] LEFT open {dist_cm}cm @ {rel:.0f}° — turning"

                # Reached end of left scan range — switch to right phase
                if rel >= self.scan_side_deg:
                    self._scan_phase = "right"
                    self._scan_last_sample_yaw = yaw_deg
                    return 0.0, 0.0, "[SCAN] Robot rotating RIGHT..."

                # Continue rotating left (v=0, omega>0 = CCW/left spin)
                return 0.0, self.omega_scan, None

            # Phase 2: Scan RIGHT (negative relative angles)
            if self._scan_phase == "right":
                # Early exit if open path found on the right side
                if dist_cm >= self.open_path_cm and rel < -10:
                    self._target_heading = yaw_deg
                    self.chosen_direction = "RIGHT"
                    self._enter(RobotState.TURN_TO_PATH, now)
                    return 0.0, 0.0, f"[SCAN] RIGHT open {dist_cm}cm @ {rel:.0f}° — turning"

                # Reached end of right scan range — pick best path from all samples
                if rel <= -self.scan_side_deg:
                    heading, direction = self._pick_best_path()
                    self._target_heading = heading
                    self.chosen_direction = direction
                    self._enter(RobotState.TURN_TO_PATH, now)
                    return 0.0, 0.0, f"[SCAN] T/L junction → go {direction} ({self._scan_samples[-1][1] if self._scan_samples else 0}cm)"

                # Continue rotating right (v=0, omega<0 = CW/right spin)
                return 0.0, -self.omega_scan, None

        # --- TURN_TO_PATH: align robot to the chosen heading ---
        if self.state == RobotState.TURN_TO_PATH:
            if self._target_heading is None:
                # No target — resume cruising
                self._enter(RobotState.CRUISE, now)
                return self.v_cruise, 0.0, "[ROBOT] Cruising forward"

            # Proportional control: compute heading error
            err = self._angle_diff(self._target_heading, yaw_deg)
            if abs(err) < 8.0:
                # Aligned within tolerance — resume cruising
                self._target_heading = None
                self._enter(RobotState.CRUISE, now)
                d = self.chosen_direction or "forward"
                return self.v_cruise, 0.0, f"[ROBOT] Aligned — going {d}"
            # Proportional angular velocity with saturation
            return 0.0, math.copysign(min(abs(err) * 0.04, self.omega_scan), err), None

        # Default: no action
        return 0.0, 0.0, None

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        """Compute the shortest signed angular difference between two angles.

        Args:
            target: Target angle in degrees.
            current: Current angle in degrees.

        Returns:
            Signed difference in degrees in [-180, +180].
            Positive means target is counterclockwise from current.

        Algorithm:
          Uses (target - current + 180) % 360 - 180 to wrap the difference
          into the shortest turning direction.
        """
        return (target - current + 180) % 360 - 180
