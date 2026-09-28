"""WiFi RSSI human sensing — compares live RSSI to empty-room baseline.

TASK-10 (D5): RSSI is now purely a HINT source, never a brake by itself.
`consecutive_drops` is tracked so main_controller.py can decide whether
a run of drops (plus a wall in front) justifies actually stopping, per
the task's rule:
    Stop motors for WiFi only if YOLO also confirms, OR
    3 consecutive drops AND a wall in front.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WifiHumanSensor:
    rssi_drop_threshold_dbm: float = 8.0
    baseline_rssi: float | None = None
    _samples: list[float] = field(default_factory=list)
    consecutive_drops: int = 0

    def calibrate(self, rssi: float) -> None:
        """Empty room baseline — wall ke peeche koi human nahi."""
        self._samples.append(rssi)
        if len(self._samples) > 30:
            self._samples.pop(0)
        self.baseline_rssi = sum(self._samples) / len(self._samples)
        print(f"[WIFI] Baseline RSSI calibrated: {self.baseline_rssi:.1f} dBm")

    def update(self, rssi: float) -> str | None:
        if self.baseline_rssi is None:
            self.calibrate(rssi)
            return None

        drop = self.baseline_rssi - rssi
        if drop >= self.rssi_drop_threshold_dbm:
            self.consecutive_drops += 1
            return "HUMAN_POSSIBLE_BEHIND_WALL"
        # TASK-10: a single non-drop resets the streak -- only a
        # SUSTAINED run of drops should ever be able to influence motion.
        self.consecutive_drops = 0
        return None

    def reset(self) -> None:
        self.baseline_rssi = None
        self._samples.clear()
        self.consecutive_drops = 0

