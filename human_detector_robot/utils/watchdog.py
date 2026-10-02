"""Stuck / oscillation watchdog -- Domain D4.

TASK-08 -- Stuck / Oscillation Watchdog

Two independent checks, both Python-only, no hardware involved:
  1. Pose watchdog: if the robot's (x, y) has barely moved for ~4s
     WHILE it was being commanded to translate AND a wall/obstacle is
     present, force a turn opposite the exploration bias (break the
     left-right wobble / wedge-stuck case).
  2. State watchdog: if the FSM has been sitting in the same state for
     more than ~8s (excluding states that are legitimately slow, like
     ALERT_PAUSE), reset it back to CRUISE so the mission keeps moving
     instead of freezing for multiple minutes.

FIX (pose check, 2026-09): check 1 used to fire on "pose hasn't moved
for 4s + wall within wall_stop_cm", WITHOUT asking whether the robot had
even been told to move. A deliberate hold (ALERT_PAUSE, WIFI_SCAN settle,
SCAN_ROTATE, SWARM_YIELD, failsafe) freezes (x, y) by definition, so the
check fired every 4s for the whole run and sprayed ~1s recovery spins at
random-looking times (54 of them in a 4-minute log). The caller now passes
the FSM's commanded `commanded_v`; the pose window only ages while
|commanded_v| > 0, and is restarted on every idle tick.

The watchdog never touches hardware directly: it returns an override
(v, omega) that the controller substitutes for the FSM's own output
for one cycle, plus a human-readable HUD message. When no override is
active it returns the FSM command unchanged (as (False, 0, 0, None)).
"""

from __future__ import annotations

import math

from utils.autonomous_fsm import AutonomousFSM, RobotState

# States allowed to legitimately sit for a while without being
# considered "stuck" by the state-timeout watchdog.
_EXEMPT_STATES = {RobotState.ALERT_PAUSE}


class StuckWatchdog:
    """Detects pose-stuck and state-timeout conditions and prescribes
    short recovery maneuvers (TASK-08).

    Attributes:
        pose_stuck_timeout_s: Seconds without meaningful movement (with a
            wall present) before triggering a recovery turn (default 4.0).
            Only counts time spent while the caller was commanding motion.
        pose_stuck_radius_m: Movement below this distance (meters) counts
            as "not moved" (default 0.03 = 3cm).
        state_timeout_s: Seconds in one FSM state before force-reset to
            CRUISE (default 8.0).
        recovery_omega: Turn command (rad/s) of the recovery spin — sign
            picks the direction only; with firmware PIVOT_TURNS the actual
            pivot runs at PIVOT_TURN_V_MS = full speed (one side stopped).
        recovery_turn_s: Duration (seconds) of the recovery spin.
        _anchor_x/_anchor_y/_anchor_t: Pose anchor point and its timestamp.
        _recovering_until: If now < this, a recovery turn is still running.
        _initialized: Whether the anchor has been seeded once.
    """

    def __init__(
        self,
        pose_stuck_timeout_s: float = 4.0,
        pose_stuck_radius_m: float = 0.03,
        state_timeout_s: float = 8.0,
        recovery_omega: float = 0.35,
        recovery_turn_s: float = 1.0,
    ) -> None:
        """Initialize the watchdog thresholds and internal anchor state.

        Args:
            pose_stuck_timeout_s: Stuck duration before pose recovery.
            pose_stuck_radius_m: Movement radius that counts as "moving".
            state_timeout_s: FSM state duration before force-reset.
            recovery_omega: Recovery turn command (rad/s) — direction only,
                the firmware pivot sets the actual rate (see PIVOT_TURNS).
            recovery_turn_s: Recovery turn duration (seconds).
        """
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
        """(Re)seed the pose anchor point used by the stuck check.

        Args:
            x: Current robot X in meters.
            y: Current robot Y in meters.
            now: Current timestamp (epoch seconds).

        Side effects:
          _anchor_x/_anchor_y set to (x, y); _anchor_t set to now — the
          stuck timer effectively restarts.
        """
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
        commanded_v: float = 0.0,
    ) -> tuple[bool, float, float, str | None]:
        """Returns (override_active, v, omega, message). If
        override_active is False, the caller should use the FSM's own
        (v, omega) unchanged.

        Args:
            now: Current timestamp (epoch seconds).
            x: Robot world X in meters.
            y: Robot world Y in meters.
            wall_present: True when a nearby wall/obstacle is detected
                (from the filtered sonar readings).
            fsm: The FSM instance being supervised (read + force_state).
            exploration_bias: "left" or "right" — the recovery turn spins
                OPPOSITE this direction to break a wedge/wobble.
            commanded_v: Linear velocity the controller is about to send
                this tick, i.e. the FSM output AFTER the failsafe override
                and BEFORE this watchdog's own override. |commanded_v| > 0
                means "the robot is being asked to translate". Pass 0.0
                whenever motion is deliberately suppressed.

        Returns:
            (override_active, v, omega, message):
              override_active=False → ignore v/omega/message (use FSM's).
              override_active=True  → substitute (v, omega) this cycle;
                                      message is a HUD/log string.

        Algorithm:
          1. First call ever → seed the pose anchor from the current
             position and return no override.
          2. Mid-recovery (now < _recovering_until) → keep commanding the
             spin (v=0, omega=±recovery_omega) until the timer expires;
             sign is opposite the exploration bias.
          3. Pose check — ONLY while commanded_v != 0:
             a. |commanded_v| == 0 → the pose cannot move on purpose
                (ALERT_PAUSE / scan / yield / failsafe), so restart the
                anchor instead of judging a hold to be "stuck".
             b. moved > pose_stuck_radius_m → re-anchor (progressing).
             c. wall present AND anchor older than pose_stuck_timeout_s →
                re-anchor, schedule a recovery turn ending at
                now + recovery_turn_s, and return the spin override.
          4. State check: for non-exempt FSM states (ALERT_PAUSE is
             exempt), if now - fsm.state_entered_at >= state_timeout_s →
             fsm.force_state(CRUISE), re-anchor, and return a cruise
             override (v=v_cruise, omega=0) with a message.
          5. Otherwise → (False, 0, 0, None): no override needed.
        """
        if not self._initialized:
            self._reset_anchor(x, y, now)
            self._initialized = True

        # Currently mid-recovery turn -- keep turning until it's done.
        if now < self._recovering_until:
            turn = self.recovery_omega if exploration_bias != "left" else -self.recovery_omega
            return True, 0.0, turn, "[WATCHDOG] Recovery turn in progress..."

        # --- Check 1: pose almost unchanged for pose_stuck_timeout_s ---
        if abs(commanded_v) > 1e-6:
            moved = math.hypot(x - self._anchor_x, y - self._anchor_y)
            if moved > self.pose_stuck_radius_m:
                self._reset_anchor(x, y, now)
            elif wall_present and (now - self._anchor_t) >= self.pose_stuck_timeout_s:
                self._reset_anchor(x, y, now)
                self._recovering_until = now + self.recovery_turn_s
                turn = self.recovery_omega if exploration_bias != "left" else -self.recovery_omega
                return True, 0.0, turn, "[WATCHDOG] Pose stuck near wall — forcing turn"
        else:
            # Not commanded to translate this tick — a frozen pose is the
            # INTENDED behaviour, not a wedge. Restart the window so the
            # 4s timer only measures time spent actually trying to move.
            self._reset_anchor(x, y, now)

        # --- Check 2: same FSM state too long -> reset to CRUISE ---
        if fsm.state not in _EXEMPT_STATES:
            if (now - fsm.state_entered_at) >= self.state_timeout_s:
                fsm.force_state(RobotState.CRUISE, now)
                self._reset_anchor(x, y, now)
                return True, fsm.v_cruise, 0.0, f"[WATCHDOG] State timeout ({self.state_timeout_s:.0f}s) — reset to CRUISE"

        return False, 0.0, 0.0, None
