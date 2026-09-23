"""WiFi RSSI human sensing — compares live RSSI to empty-room baseline."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WifiHumanSensor:
    rssi_drop_threshold_dbm: float = 8.0
    baseline_rssi: float | None = None
    _samples: list[float] = field(default_factory=list)

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
            return "HUMAN_POSSIBLE_BEHIND_WALL"
        return None

    def reset(self) -> None:
        self.baseline_rssi = None
        self._samples.clear()
