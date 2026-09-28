"""Unique human/survivor tracker -- Domain D5.

TASK-09 -- Unique Human Tracker

Problem it fixes: one physical person could be logged 18+ times at
nearly the same coordinate, because the old code only had a TIME
cooldown (HUMAN_MARK_COOLDOWN_S) -- if the robot lingered near a
survivor for more than that window, every detection created a new pin.

Fix: a new pin is only created if there is no existing track within
`merge_radius_m` (~80cm) of the new detection. If one exists, it is
updated in place (never duplicated). Priority when merging:
  standing > fallen > generic person > wifi
so a WiFi hint never downgrades/overwrites a confirmed CV detection,
but a later CV confirmation IS allowed to upgrade an earlier WiFi-only
guess at the same spot.
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field

_PRIORITY = {
    "standing": 3,
    "cv_standing": 3,
    "fallen": 2,
    "cv_fallen": 2,
    "cv": 1,
    "person": 1,
    "wifi": 0,
}


@dataclass
class HumanTrack:
    id: int
    cls: str
    conf: float
    x: float
    y: float
    timestamp: float


class HumanTracker:
    def __init__(self, merge_radius_m: float = 0.8) -> None:
        self.merge_radius_m = merge_radius_m
        self._tracks: list[HumanTrack] = []
        self._id_counter = itertools.count(1)

    @staticmethod
    def _priority(cls: str) -> int:
        return _PRIORITY.get(cls, 0)

    def _find_nearby(self, x: float, y: float) -> HumanTrack | None:
        best: HumanTrack | None = None
        best_d = self.merge_radius_m
        for t in self._tracks:
            d = math.hypot(t.x - x, t.y - y)
            if d <= best_d:
                best = t
                best_d = d
        return best

    def observe(self, cls: str, conf: float, x: float, y: float, now: float | None = None) -> tuple[HumanTrack, bool]:
        """Reports one detection at world position (x, y) meters.

        Returns (track, is_new). Caller should only draw a NEW map pin /
        increment the rescue log when `is_new` is True -- a merge into an
        existing track means "same person, already logged".
        """
        now = now if now is not None else time.time()
        existing = self._find_nearby(x, y)

        if existing is None:
            track = HumanTrack(id=next(self._id_counter), cls=cls, conf=conf, x=x, y=y, timestamp=now)
            self._tracks.append(track)
            return track, True

        # Spatial cooldown IS the merge radius above -- a detection near
        # an existing track is "the same person", regardless of how much
        # time has passed (unlike the old time-only cooldown).
        if self._priority(cls) >= self._priority(existing.cls):
            existing.cls = cls
            existing.conf = conf
        existing.x = (existing.x + x) / 2.0
        existing.y = (existing.y + y) / 2.0
        existing.timestamp = now
        return existing, False

    @property
    def count(self) -> int:
        return len(self._tracks)

    def all_tracks(self) -> list[HumanTrack]:
        return list(self._tracks)
