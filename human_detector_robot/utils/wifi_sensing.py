"""WiFi RSSI human sensing — compares live RSSI to empty-room baseline.

TASK-10 (D5): RSSI is now purely a HINT source, never a brake by itself.
`consecutive_drops` is tracked so main_controller.py can decide whether
a run of drops (plus a wall in front) justifies actually stopping, per
the task's rule:
    Stop motors for WiFi only if YOLO also confirms, OR
    3 consecutive drops AND a wall in front.

Detection Principle:
  A human body absorbs/scatters 2.4GHz WiFi energy, so a sustained RSSI
  drop below the calibrated empty-room baseline indicates presence —
  useful for detecting people behind walls where the camera can't see.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WifiHumanSensor:
    """WiFi RSSI-based human presence sensor (hint source, TASK-10).

    Attributes:
        rssi_drop_threshold_dbm: Minimum drop (dBm) from baseline to count
            as a sensing event. Default 8.0.
        baseline_rssi: Calibrated empty-room average RSSI (dBm), None until
            the first calibration call.
        _samples: Rolling window (max 30) of calibration RSSI samples.
        consecutive_drops: TASK-10 — number of consecutive update() calls
            that exceeded the threshold. Reset to 0 on any non-drop. The
            controller escalates this to a motor-stopping alert only when
            the streak reaches 3 AND a wall is ahead (or CV also confirms).
    """

    rssi_drop_threshold_dbm: float = 8.0
    baseline_rssi: float | None = None
    _samples: list[float] = field(default_factory=list)
    consecutive_drops: int = 0

    def calibrate(self, rssi: float) -> None:
        """Empty room baseline — wall ke peeche koi human nahi.

        Args:
            rssi: Current RSSI reading in dBm (negative, e.g. -45.0).

        Algorithm:
          1. Append the sample to the rolling window (max 30; oldest popped).
          2. baseline_rssi = mean(window samples).
          3. Print the new baseline for operator visibility.

        Note:
          Call repeatedly while the room is confirmed empty; the rolling
          window averages out natural RSSI jitter.
        """
        self._samples.append(rssi)
        if len(self._samples) > 30:
            self._samples.pop(0)
        self.baseline_rssi = sum(self._samples) / len(self._samples)
        print(f"[WIFI] Baseline RSSI calibrated: {self.baseline_rssi:.1f} dBm")

    def update(self, rssi: float) -> str | None:
        """Compare a new RSSI reading against the baseline and detect presence.

        Args:
            rssi: Current RSSI reading in dBm.

        Returns:
            'HUMAN_POSSIBLE_BEHIND_WALL' when the drop meets/exceeds the
            threshold; None otherwise (including the first sample, which
            auto-calibrates instead of detecting).

        Algorithm:
          1. If baseline is None → calibrate with this sample, return None.
          2. drop = baseline_rssi - rssi (positive = signal weakened).
          3. If drop >= rssi_drop_threshold_dbm:
               consecutive_drops += 1; return the hint string.
          4. Else: consecutive_drops = 0 (streak broken); return None.
             A single healthy reading kills the streak — only a SUSTAINED
             run of drops can ever influence motion (TASK-10).
        """
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
        """Clear baseline, samples, and drop streak (e.g. for --calibrate-wifi).

        Side effects:
          baseline_rssi → None, _samples cleared, consecutive_drops → 0.
          The next update() call will start a fresh calibration.
        """
        self.baseline_rssi = None
        self._samples.clear()
        self.consecutive_drops = 0
