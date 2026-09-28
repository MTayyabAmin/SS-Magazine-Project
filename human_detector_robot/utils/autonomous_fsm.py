"""
Autonomous FSM — L-turn + T-junction path finder.

NO SERVO — poora robot apni jagah rotate hota hai (v=0, omega≠0).
Left/right dono directions scan hoti hain; jo rasta zyada khula ho wahan jata hai.

Corner / T-junction flow:
  1. Stop @ 100cm → creep to 30-40cm standoff
  2. WiFi human scan
  3. Robot rotate LEFT 90°  (sonar samples har 15°)
  4. Robot rotate RIGHT 90° (wapas center se dusri taraf)
  5. Best open direction choose → align → CRUISE

TASK-07 (D4 -- FSM: Path Follow + Junctions) changes vs the original:
  - CRUISE now does corridor centering (left/right sonar difference) and,
    if the planner (TASK-06 A*) supplies a target heading, blends in a
    small steering term toward it.
  - WIFI_SCAN checks BOTH side sensors before deciding a direction,
    instead of always preferring LEFT just because it was checked first.
  - SCAN_ROTATE completes the FULL +/-90 deg sweep -- no early-exit on
    the first opening found (which could miss a wider opening a few
    degrees further round).
  - CREEP_TO_SCAN aborts after ~3s if the 30-40cm standoff isn't reached
    (avoids infinite creep / stuck-at-50cm spinning).
  - TURN_TO_PATH alignment tolerance tightened from 8deg to 4deg.
"""

from __future__ import annotations

import math
from enum import Enum, auto


class RobotState(Enum):
    CRUISE = auto()
    STOP_AT_WALL = auto()
    CREEP_TO_SCAN = auto()
    WIFI_SCAN = auto()
    SCAN_ROTATE = auto()    # robot spins in place — no servo
    TURN_TO_PATH = auto()
    ALERT_PAUSE = auto()
    SWARM_YIELD = auto()    # pause / yield to avoid crashing into peer robot


class AutonomousFSM:
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
        exploration_bias: str = "left",  # "left" for Robot 1, "right" for Robot 2 (max area coverage)
        align_tolerance_deg: float = 4.0,   # TASK-07: was 8, now 4 (tighter align)
        creep_timeout_s: float = 3.0,       # TASK-07: abort creep if standoff not reached
        center_gain: float = 0.0035,        # TASK-07: corridor centering P-gain
        center_omega_max: float = 0.15,     # TASK-07: cap on centering omega
        path_gain: float = 0.02,            # TASK-07: A* heading-follow P-gain
        path_omega_max: float = 0.25,       # TASK-07: cap on path-follow omega
        yield_timeout_s: float = 1.5,       # TASK-11: max time to yield before replanning
    ) -> None:
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
        self.align_tolerance_deg = align_tolerance_deg
        self.creep_timeout_s = creep_timeout_s
        self.center_gain = center_gain
        self.center_omega_max = center_omega_max
        self.path_gain = path_gain
        self.path_omega_max = path_omega_max
        self.yield_timeout_s = yield_timeout_s

        self._state_start: float = 0.0
        self._scan_center_yaw: float = 0.0
        self._scan_last_sample_yaw: float = 0.0
        self._scan_phase: str = "left"  # left → right → pick
        self._scan_samples: list[tuple[float, int]] = []  # (rel_deg, dist_cm)
        self._target_heading: float | None = None
        self.alert_message: str | None = None
        self.human_detected_this_stop: bool = False
        self.last_sonar_sample: int = -1
        self._pre_alert_state: RobotState = RobotState.CRUISE
        self.chosen_direction: str = ""

        # TASK-08 (watchdog) hooks -- main_controller/watchdog.py reads
        # these to detect "stuck in the same state too long".
        self.state_entered_at: float = 0.0

    def _elapsed(self, now: float) -> float:
        return now - self._state_start

    def _enter(self, state: RobotState, now: float) -> None:
        self.state = state
        self._state_start = now
        self.state_entered_at = now

    def _rel_yaw(self, yaw_deg: float) -> float:
        """Degrees relative to scan center (+ = left, - = right)."""
        return self._angle_diff(yaw_deg, self._scan_center_yaw)

    def _sample_sonar(self, yaw_deg: float, dist_cm: int) -> None:
        if dist_cm <= 0:
            return
        if abs(self._angle_diff(yaw_deg, self._scan_last_sample_yaw)) >= self.scan_step_deg:
            rel = self._rel_yaw(yaw_deg)
            self._scan_samples.append((rel, dist_cm))
            self._scan_last_sample_yaw = yaw_deg

    def _pick_best_path(self) -> tuple[float, str]:
        """Choose best heading from left/right samples with exploration bias."""
        if not self._scan_samples:
            return self._scan_center_yaw, "none"

        open_candidates = [s for s in self._scan_samples if s[1] >= self.open_path_cm]
        if open_candidates:
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
            best_rel, best_dist = max(self._scan_samples, key=lambda s: s[1])

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
        self.last_sonar_sample = dist_cm

    def force_state(self, state: RobotState, now: float) -> None:
        """TASK-08: watchdog uses this to force a reset (e.g. back to
        CRUISE) when the FSM has been stuck."""
        self._enter(state, now)
        self._target_heading = None

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
        target_heading_deg: float | None = None,
    ) -> tuple[float, float, str | None]:
        """Returns (v, omega). Robot rotates in place — no servo movement.

        `target_heading_deg` (TASK-07, optional): if the A* planner (D3)
        has a next waypoint, main_controller passes the bearing toward it
        here and CRUISE blends a small steering term toward it on top of
        corridor centering. If None, CRUISE just goes straight + centers.
        """
        # Swarm anti-collision: avoid clashing into peer robot
        if peer_too_close:
            if self.state != RobotState.SWARM_YIELD:
                self._enter(RobotState.SWARM_YIELD, now)
            # TASK-11: don't yield forever -- after yield_timeout_s, stop
            # waiting passively and let the caller know it's time to
            # replan (e.g. pick a different frontier / direction).
            if self._elapsed(now) >= self.yield_timeout_s:
                return 0.0, 0.0, "[SWARM] Yield timeout — requesting replan"
            yield_turn = 0.35 if self.exploration_bias == "left" else -0.35
            return 0.0, yield_turn, "[SWARM] Peer robot nearby — yielding to prevent clash!"

        if self.state == RobotState.SWARM_YIELD:
            if not peer_too_close:
                self._enter(RobotState.CRUISE, now)
                return self.v_cruise, 0.0, "[SWARM] Path clear — resuming search"
            if self._elapsed(now) >= self.yield_timeout_s:
                return 0.0, 0.0, "[SWARM] Yield timeout — requesting replan"
            yield_turn = 0.35 if self.exploration_bias == "left" else -0.35
            return 0.0, yield_turn, "[SWARM] Still yielding to peer robot..."

        if cv_alert or wifi_alert:
            if self.state != RobotState.ALERT_PAUSE:
                self.alert_message = cv_alert or wifi_alert
                self.human_detected_this_stop = True
                self._pre_alert_state = self.state
                self._enter(RobotState.ALERT_PAUSE, now)
                return 0.0, 0.0, self.alert_message

        if self.state == RobotState.ALERT_PAUSE:
            if self._elapsed(now) >= self.alert_pause_s:
                if self._pre_alert_state in (RobotState.CRUISE, RobotState.STOP_AT_WALL):
                    self._enter(RobotState.CRUISE, now)
                    return self.v_cruise, 0.0, "[MAP] Human logged — resuming..."
                self._enter(RobotState.CREEP_TO_SCAN, now)
                return 0.0, 0.0, "[MAP] Human logged — continuing scan..."
            return 0.0, 0.0, None

        if self.state == RobotState.CRUISE:
            if wall_near or (0 < dist_cm <= self.wall_stop_cm):
                self._enter(RobotState.STOP_AT_WALL, now)
                self.human_detected_this_stop = False
                return 0.0, 0.0, f"[SONAR] Wall at {dist_cm}cm — stopping"

            # TASK-07: corridor centering -- nudge away from whichever
            # side sonar is closer, so the robot doesn't hug one wall.
            omega = 0.0
            if dist_left > 0 and dist_right > 0:
                diff = dist_left - dist_right  # +ve = right is closer
                omega += max(-self.center_omega_max, min(self.center_omega_max, -diff * self.center_gain))

            # TASK-07: blend in A* waypoint heading if the planner gave us one.
            if target_heading_deg is not None:
                err = self._angle_diff(target_heading_deg, yaw_deg)
                omega += max(-self.path_omega_max, min(self.path_omega_max, err * self.path_gain))

            return self.v_cruise, omega, None

        if self.state == RobotState.STOP_AT_WALL:
            self._enter(RobotState.CREEP_TO_SCAN, now)
            return 0.0, 0.0, "[SCAN] Creep to 30-40cm (robot will rotate in place next)..."

        if self.state == RobotState.CREEP_TO_SCAN:
            # TASK-07: abort creep if the 30-40cm standoff isn't reached
            # within ~3s -- avoids the old "stuck at ~50cm, spinning
            # wheels forever" bug. We just go scan from wherever we are.
            if self._elapsed(now) >= self.creep_timeout_s:
                self._enter(RobotState.WIFI_SCAN, now)
                return 0.0, 0.0, f"[SCAN] Creep timeout ({self.creep_timeout_s:.0f}s) — scanning from {dist_cm}cm"
            if dist_cm < 0:
                return 0.0, 0.0, None
            if self.scan_standoff_min <= dist_cm <= self.scan_standoff_max:
                self._enter(RobotState.WIFI_SCAN, now)
                return 0.0, 0.0, f"[SCAN] Standoff {dist_cm}cm OK — WiFi scan..."
            if dist_cm > self.scan_standoff_max:
                return self.v_creep, 0.0, None
            return -self.v_creep * 0.5, 0.0, f"[SCAN] Too close ({dist_cm}cm) — reverse"

        if self.state == RobotState.WIFI_SCAN:
            if self._elapsed(now) >= 0.8:
                self._scan_center_yaw = yaw_deg
                self._scan_last_sample_yaw = yaw_deg
                self._scan_samples = [(0.0, dist_cm if dist_cm > 0 else 0)]

                # TASK-07: look at BOTH side sensors before deciding,
                # instead of short-circuiting on whichever is checked
                # first (old code always preferred LEFT even when RIGHT
                # was more open).
                left_open = dist_left if dist_left >= self.open_path_cm else -1
                right_open = dist_right if dist_right >= self.open_path_cm else -1

                if left_open > 0 or right_open > 0:
                    if left_open >= right_open:
                        self._target_heading = yaw_deg + 90.0
                        self.chosen_direction = "LEFT (sonar)"
                        self._enter(RobotState.TURN_TO_PATH, now)
                        return 0.0, 0.0, f"[SCAN] LEFT path open {dist_left}cm — turning"
                    self._target_heading = yaw_deg - 90.0
                    self.chosen_direction = "RIGHT (sonar)"
                    self._enter(RobotState.TURN_TO_PATH, now)
                    return 0.0, 0.0, f"[SCAN] RIGHT path open {dist_right}cm — turning"

                self._scan_phase = "left"
                self._enter(RobotState.SCAN_ROTATE, now)
                return 0.0, 0.0, "[SCAN] Side sensors blocked — robot rotating..."
            return 0.0, 0.0, None

        # --- Robot spins in place: scan LEFT 90° then RIGHT 90° ---
        if self.state == RobotState.SCAN_ROTATE:
            self._sample_sonar(yaw_deg, dist_cm)
            rel = self._rel_yaw(yaw_deg)

            if self._scan_phase == "left":
                # TASK-07: NO early-exit -- always complete the full
                # +/-90 deg sweep before choosing, so a wider opening a
                # few degrees further round isn't missed.
                if rel >= self.scan_side_deg:
                    self._scan_phase = "right"
                    self._scan_last_sample_yaw = yaw_deg
                    return 0.0, 0.0, "[SCAN] Robot rotating RIGHT..."
                return 0.0, self.omega_scan, None

            if self._scan_phase == "right":
                if rel <= -self.scan_side_deg:
                    heading, direction = self._pick_best_path()
                    self._target_heading = heading
                    self.chosen_direction = direction
                    self._enter(RobotState.TURN_TO_PATH, now)
                    return 0.0, 0.0, f"[SCAN] T/L junction → go {direction} ({self._scan_samples[-1][1] if self._scan_samples else 0}cm)"
                return 0.0, -self.omega_scan, None

        if self.state == RobotState.TURN_TO_PATH:
            if self._target_heading is None:
                self._enter(RobotState.CRUISE, now)
                return self.v_cruise, 0.0, "[ROBOT] Cruising forward"

            err = self._angle_diff(self._target_heading, yaw_deg)
            if abs(err) < self.align_tolerance_deg:
                self._target_heading = None
                self._enter(RobotState.CRUISE, now)
                d = self.chosen_direction or "forward"
                return self.v_cruise, 0.0, f"[ROBOT] Aligned — going {d}"
            return 0.0, math.copysign(min(abs(err) * 0.04, self.omega_scan), err), None

        return 0.0, 0.0, None

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        return (target - current + 180) % 360 - 180
