"""Sensor cleaning utilities — Domain D1 (Pose & Sensor Clean).

TASK-01 -- Sonar (Ultrasonic) Median Filter
TASK-02 -- Yaw Jump Guard

Both are pure-Python, no hardware/firmware changes. They sit between the
raw UDP telemetry dict and the FSM / mapping code, so the rest of the
system never sees an invalid -1, a 2cm glitch, a 400cm spike, or a yaw
that teleports from 0 deg to -173 deg in one sample.

Design notes:
  - SonarMedianFilter: median-of-window rejects single-sample spikes;
    a jump-limiter additionally rejects samples too far from the last
    trusted median; invalid (<=0) samples keep the last good value.
  - YawJumpGuard: rate-limits how far the believed heading may move per
    update, so one corrupted IMU sample can never teleport the pose.
  - RobotSensorFilters bundles one filter per sensor plus one yaw guard
    into a single apply() call for the controllers.
"""

from __future__ import annotations

import time
from collections import deque


class SonarMedianFilter:
    """Cleans one ultrasonic (sonar) sensor's readings.

    Rules (TASK-01):
      - keeps the last `window` samples
      - a raw value <= 0 is "no echo / invalid" -- never fed into the
        filter and never treated as a real wall or a real opening
      - a sample that jumps more than `max_jump_cm` from the last
        ACCEPTED median is rejected as a spike (e.g. a stray 2cm or
        400cm echo) instead of being trusted
      - `.value` / `update()` return the median of the accepted window

    Attributes:
        window: Number of samples in the rolling median window.
        max_jump_cm: Maximum allowed jump from the last accepted median.
        _samples: Rolling deque of accepted raw samples (maxlen=window).
        _last_median: Last computed median (-1 until the first valid sample).
    """

    def __init__(self, window: int = 5, max_jump_cm: float = 80.0) -> None:
        """Initialize the filter.

        Args:
            window: Rolling window size (odd sizes give a true middle
                sample; even sizes average the two middle samples).
            max_jump_cm: Spike-rejection threshold in centimeters.
        """
        self.window = window
        self.max_jump_cm = max_jump_cm
        self._samples: deque[int] = deque(maxlen=window)
        self._last_median: int = -1

    def update(self, raw_cm: int) -> int:
        """Feed one raw sonar sample, get the cleaned value back.

        Args:
            raw_cm: Raw distance in cm from telemetry (-1/0 = invalid echo).

        Returns:
            The current filtered distance in cm: the median of the accepted
            window, or the previous median when the input is rejected,
            or -1 while no valid sample has ever been accepted.

        Algorithm:
          1. raw <= 0 (or None) → invalid: return _last_median unchanged
             (never treat -1 as a wall, never treat 0 as an opening).
          2. |raw - _last_median| > max_jump_cm (with a known median) →
             spike: reject, return _last_median.
          3. Otherwise append to the deque (old samples drop off).
          4. Sort the window and take the middle value (average the two
             middles for even lengths).
          5. Store as _last_median and return it.
        """
        if raw_cm is None or raw_cm <= 0:
            # invalid echo -- do NOT treat as a wall (0/-1) or an opening.
            # Just keep reporting the last known-good median.
            return self._last_median

        if self._last_median > 0 and abs(raw_cm - self._last_median) > self.max_jump_cm:
            # sudden spike far from what we trusted before -- drop it
            return self._last_median

        self._samples.append(raw_cm)
        ordered = sorted(self._samples)
        n = len(ordered)
        median = ordered[n // 2] if n % 2 == 1 else (ordered[n // 2 - 1] + ordered[n // 2]) // 2
        self._last_median = median
        return median

    @property
    def value(self) -> int:
        """Last accepted median without feeding a new sample (-1 if none)."""
        return self._last_median

    def reset(self) -> None:
        """Clear the window and median (back to 'no data' state).

        Side effects:
          _samples emptied; _last_median → -1.
        """
        self._samples.clear()
        self._last_median = -1


class YawJumpGuard:
    """Cleans MPU6050 yaw so a bad IMU sample can't teleport the robot's
    believed heading (TASK-02).

    Rules:
      - the change applied per update() is capped to `max_deg_per_s * dt`
      - a raw sample that implies a bigger jump than physically possible
        in that dt is NOT teleported to -- the heading only moves by the
        capped amount toward it, so a stationary robot's yaw may drift
        slowly but will never jump instantly.

    Attributes:
        max_deg_per_s: Maximum heading change allowed per second.
        _yaw: Current guarded heading (deg, [0,360)); None until first sample.
        _last_t: Timestamp of the previous update (None until first sample).
    """

    def __init__(self, max_deg_per_s: float = 90.0) -> None:
        """Initialize the guard.

        Args:
            max_deg_per_s: Slew-rate limit in degrees per second.
        """
        self.max_deg_per_s = max_deg_per_s
        self._yaw: float | None = None
        self._last_t: float | None = None

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Shortest signed difference (a - b) in [-180, 180).

        Args:
            a: First angle in degrees.
            b: Second angle in degrees.

        Returns:
            Signed angular difference, correct across the 0/360 wrap.
        """
        return (a - b + 180.0) % 360.0 - 180.0

    def update(self, raw_yaw_deg: float, now: float | None = None) -> float:
        """Feed a raw MPU yaw sample; receive the slew-limited heading.

        Args:
            raw_yaw_deg: Raw heading in degrees from telemetry.
            now: Timestamp (defaults to time.time()).

        Returns:
            The guarded heading in [0, 360).

        Algorithm:
          1. First sample → adopt it directly (no history to guard),
             record timestamp, return normalized to [0,360).
          2. dt = max(now - last_t, 1e-3) — never divide by zero even if
             two samples arrive in the same microsecond.
          3. diff = shortest angular difference (raw - current).
          4. max_step = max_deg_per_s * dt; clamp |diff| to max_step so
             the heading SLEWS toward the reading instead of jumping.
          5. current = (current + clamped diff) % 360; store and return.
        """
        now = now if now is not None else time.time()

        if self._yaw is None or self._last_t is None:
            self._yaw = raw_yaw_deg % 360.0
            self._last_t = now
            return self._yaw

        dt = max(now - self._last_t, 1e-3)
        self._last_t = now

        diff = self._angle_diff(raw_yaw_deg, self._yaw)
        max_step = self.max_deg_per_s * dt
        if abs(diff) > max_step:
            diff = max_step if diff > 0 else -max_step

        self._yaw = (self._yaw + diff) % 360.0
        return self._yaw

    @property
    def value(self) -> float:
        """Current guarded heading (0.0 if no sample received yet)."""
        return self._yaw if self._yaw is not None else 0.0

    def reset(self) -> None:
        """Forget heading and timestamp — next update() re-initializes."""
        self._yaw = None
        self._last_t = None


class RobotSensorFilters:
    """Convenience bundle: one SonarMedianFilter per sensor + one
    YawJumpGuard, so main_controller.py / swarm_controller.py only need
    to hold one object per robot.

    Attributes:
        front: Median filter for the front ultrasonic.
        left: Median filter for the left ultrasonic.
        right: Median filter for the right ultrasonic (-1 feeds are ignored
            and it simply reports -1 until valid samples arrive).
        yaw: YawJumpGuard for the MPU6050 heading.
    """

    def __init__(self, window: int = 5, max_jump_cm: float = 80.0, max_yaw_deg_per_s: float = 90.0) -> None:
        """Create the three sonar filters and the yaw guard.

        Args:
            window: Sonar rolling-median window size.
            max_jump_cm: Sonar spike-rejection threshold (cm).
            max_yaw_deg_per_s: Yaw slew-rate limit (deg/s).
        """
        self.front = SonarMedianFilter(window, max_jump_cm)
        self.left = SonarMedianFilter(window, max_jump_cm)
        self.right = SonarMedianFilter(window, max_jump_cm)
        self.yaw = YawJumpGuard(max_yaw_deg_per_s)

    def apply(self, dist_front_raw: int, dist_left_raw: int, dist_right_raw: int, yaw_raw: float) -> tuple[int, int, int, float]:
        """Filters one telemetry sample. Returns (front, left, right, yaw).

        Args:
            dist_front_raw: Raw front sonar cm (-1 = invalid).
            dist_left_raw: Raw left sonar cm (-1 = invalid).
            dist_right_raw: Raw right sonar cm (-1 = invalid/no sensor).
            yaw_raw: Raw MPU heading in degrees.

        Returns:
            (front, left, right, yaw): all four values after their
            respective filters — safe to feed straight to the FSM,
            TentativeMap, occupancy grid, and HUD.

        Algorithm:
          Run each raw input through its own filter in a fixed order and
          return the tuple. Stateless with respect to the inputs (all
          history lives inside the child filters).
        """
        f = self.front.update(dist_front_raw)
        l = self.left.update(dist_left_raw)
        r = self.right.update(dist_right_raw)
        y = self.yaw.update(yaw_raw)
        return f, l, r, y
