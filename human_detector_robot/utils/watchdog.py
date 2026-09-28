"""Stuck / oscillation watchdog -- Domain D4.

TASK-08 -- Stuck / Oscillation Watchdog

Two independent checks, both Python-only, no hardware involved:
  1. Pose watchdog: if the robot's (x, y) has barely moved for ~4s
     WHILE a wall/obstacle is present, force a turn opposite the
     exploration bias (break the left-right wobble / wedge-stuck case).
  2. State watchdog: if the FSM has been sitting in the same state for
     more than ~8s (excluding states that are legitimately slow, like
     ALERT_PAUSE), reset it back to CRUISE so the mission keeps moving
     instead of freezing for multiple minutes.
"""

from __future__ import annotations

import math

from utils.autonomous_fsm import AutonomousFSM, RobotState

# States allowed to legitimately sit for a while without being
# considered "stuck" by the state-timeout watchdog.
_EXEMPT_STATES = {RobotState.ALERT_PAUSE}


class StuckWatchdog:
    def __init__(
        self,
        pose_stuck_timeout_s: float = 4.0,
        pose_stuck_radius_m: float = 0.03,
        state_timeout_s: float = 8.0,
        recovery_omega: float = 0.35,
        recovery_turn_s: float = 1.0,
    ) -> None:
        self.pose_stuck_timeout_s = pose_stuck_timeout_s
        self.pose_stuck_radius_m = pose_stuck_radius_m
        self.state_timeout_s = state_timeout_s
        self.recovery_omega = recovery_omega
        self.recovery_turn_s = recovery_turn_s

        self._anchor_x: float = 0.0
        self._anchor_y: float = 0.0
        self._anchor_t: float = 0.0
        self._recovering_until: float = 0.0
        self._initialized = False

    def _reset_anchor(self, x: float, y: float, now: float) -> None:
        self._anchor_x, self._anchor_y = x, y
        self._anchor_t = now

    def check(
        self,
        now: float,
        x: float,
        y: float,
        wall_present: bool,
        fsm: AutonomousFSM,
        exploration_bias: str = "left",
    ) -> tuple[bool, float, float, str | None]:
        """Returns (override_active, v, omega, message). If
        override_active is False, the caller should use the FSM's own
        (v, omega) unchanged."""
        if not self._initialized:
            self._reset_anchor(x, y, now)
            self._initialized = True

        # Currently mid-recovery turn -- keep turning until it's done.
        if now < self._recovering_until:
            turn = self.recovery_omega if exploration_bias != "left" else -self.recovery_omega
            return True, 0.0, turn, "[WATCHDOG] Recovery turn in progress..."

        # --- Check 1: pose almost unchanged for pose_stuck_timeout_s ---
        moved = math.hypot(x - self._anchor_x, y - self._anchor_y)
        if moved > self.pose_stuck_radius_m:
            self._reset_anchor(x, y, now)
        elif wall_present and (now - self._anchor_t) >= self.pose_stuck_timeout_s:
            self._reset_anchor(x, y, now)
            self._recovering_until = now + self.recovery_turn_s
            turn = self.recovery_omega if exploration_bias != "left" else -self.recovery_omega
            return True, 0.0, turn, "[WATCHDOG] Pose stuck near wall — forcing turn"

        # --- Check 2: same FSM state too long -> reset to CRUISE ---
        if fsm.state not in _EXEMPT_STATES:
            if (now - fsm.state_entered_at) >= self.state_timeout_s:
                fsm.force_state(RobotState.CRUISE, now)
                self._reset_anchor(x, y, now)
                return True, fsm.v_cruise, 0.0, f"[WATCHDOG] State timeout ({self.state_timeout_s:.0f}s) — reset to CRUISE"

        return False, 0.0, 0.0, None
