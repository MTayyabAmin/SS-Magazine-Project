"""Sensor cleaning utilities — Domain D1 (Pose & Sensor Clean).

TASK-01 -- Sonar (Ultrasonic) Median Filter
TASK-02 -- Yaw Jump Guard

Both are pure-Python, no hardware/firmware changes. They sit between the
raw UDP telemetry dict and the FSM / mapping code, so the rest of the
system never sees an invalid -1, a 2cm glitch, a 400cm spike, or a yaw
that teleports from 0 deg to -173 deg in one sample.
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
    """

    def __init__(self, window: int = 5, max_jump_cm: float = 80.0) -> None:
        self.window = window
        self.max_jump_cm = max_jump_cm
        self._samples: deque[int] = deque(maxlen=window)
        self._last_median: int = -1

    def update(self, raw_cm: int) -> int:
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
        return self._last_median

    def reset(self) -> None:
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
    """

    def __init__(self, max_deg_per_s: float = 90.0) -> None:
        self.max_deg_per_s = max_deg_per_s
        self._yaw: float | None = None
        self._last_t: float | None = None

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return (a - b + 180.0) % 360.0 - 180.0

    def update(self, raw_yaw_deg: float, now: float | None = None) -> float:
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
        return self._yaw if self._yaw is not None else 0.0

    def reset(self) -> None:
        self._yaw = None
        self._last_t = None


class RobotSensorFilters:
    """Convenience bundle: one SonarMedianFilter per sensor + one
    YawJumpGuard, so main_controller.py / swarm_controller.py only need
    to hold one object per robot."""

    def __init__(self, window: int = 5, max_jump_cm: float = 80.0, max_yaw_deg_per_s: float = 90.0) -> None:
        self.front = SonarMedianFilter(window, max_jump_cm)
        self.left = SonarMedianFilter(window, max_jump_cm)
        self.right = SonarMedianFilter(window, max_jump_cm)
        self.yaw = YawJumpGuard(max_yaw_deg_per_s)

    def apply(self, dist_front_raw: int, dist_left_raw: int, dist_right_raw: int, yaw_raw: float) -> tuple[int, int, int, float]:
        """Filters one telemetry sample. Returns (front, left, right, yaw)."""
        f = self.front.update(dist_front_raw)
        l = self.left.update(dist_left_raw)
        r = self.right.update(dist_right_raw)
        y = self.yaw.update(yaw_raw)
        return f, l, r, y
