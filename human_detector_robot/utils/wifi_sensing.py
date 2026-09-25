"""WiFi RSSI human sensing — compares live RSSI to empty-room baseline.

This module implements a simple but effective WiFi-based human presence
detection system. It works on the principle that a human body absorbs
and scatters 2.4GHz WiFi signals, causing a measurable drop in RSSI
(Received Signal Strength Indicator) compared to an empty room.

Algorithm Overview:
  1. CALIBRATION: Collect RSSI samples when the room is empty to establish
     a baseline average.
  2. DETECTION: Compare each new RSSI reading to the baseline.
     If the drop exceeds a threshold (default 8 dBm), flag as
     "HUMAN_POSSIBLE_BEHIND_WALL".
  3. This is especially useful for detecting humans behind walls where
     the camera cannot see.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WifiHumanSensor:
    """WiFi RSSI-based human presence sensor.

    Attributes:
        rssi_drop_threshold_dbm: Minimum RSSI drop (in dBm) from baseline
                                 to trigger a human detection. Default 8.0 dBm.
        baseline_rssi: The calibrated empty-room average RSSI in dBm.
                       None if calibration hasn't been performed yet.
        _samples: Rolling window of RSSI samples used for baseline averaging.
    """

    rssi_drop_threshold_dbm: float = 8.0
    baseline_rssi: float | None = None
    _samples: list[float] = field(default_factory=list)

    def calibrate(self, rssi: float) -> None:
        """Update the empty-room baseline with a new RSSI sample.

        Args:
            rssi: Current RSSI reading in dBm (negative value, e.g. -45.0).

        Algorithm:
          1. Append the new sample to the rolling window.
          2. Keep at most 30 samples (removes oldest if exceeded).
          3. Compute the average of all samples as the new baseline.
          4. Print the calibrated baseline value.

        Note:
          Call this repeatedly when the room is confirmed empty to improve
          baseline accuracy over time. The rolling window smooths out
          natural RSSI fluctuations.
        """
        self._samples.append(rssi)
        if len(self._samples) > 30:
            self._samples.pop(0)
        self.baseline_rssi = sum(self._samples) / len(self._samples)
        print(f"[WIFI] Baseline RSSI calibrated: {self.baseline_rssi:.1f} dBm")

    def update(self, rssi: float) -> str | None:
        """Compare a new RSSI reading to the baseline and detect human presence.

        Args:
            rssi: Current RSSI reading in dBm.

        Returns:
            Detection string 'HUMAN_POSSIBLE_BEHIND_WALL' if a significant
            drop is detected, None otherwise.

        Algorithm:
          1. If baseline hasn't been calibrated yet, auto-calibrate with the
             first reading and return None (no detection on first sample).
          2. Compute the drop = baseline_rssi - current_rssi.
             (Positive drop means signal got weaker.)
          3. If drop >= threshold (default 8 dBm), return detection string.
          4. Otherwise return None (no human detected).
        """
        if self.baseline_rssi is None:
            self.calibrate(rssi)
            return None

        drop = self.baseline_rssi - rssi
        if drop >= self.rssi_drop_threshold_dbm:
            return "HUMAN_POSSIBLE_BEHIND_WALL"
        return None

    def reset(self) -> None:
        """Reset the sensor by clearing the baseline and all stored samples.

        Algorithm:
          1. Set baseline_rssi to None (uncalibrated state).
          2. Clear the _samples list.
          3. Next update() call will auto-calibrate from scratch.
        """
        self.baseline_rssi = None
        self._samples.clear()
