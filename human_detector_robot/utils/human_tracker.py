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

Design notes:
  - Spatial cooldown IS the merge radius: proximity alone means "same
    person", regardless of elapsed time (unlike the old time-only rule).
  - observe() returns (track, is_new) so callers only draw a pin /
    increment the rescue log when is_new is True.
  - Merged positions are averaged (simple centroid) to reduce jitter.
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
    """One deduplicated survivor sighting.

    Attributes:
        id: Monotonic unique track id (1, 2, 3, ...).
        cls: Detection class ("standing", "fallen", "cv", "person", "wifi", ...).
        conf: Confidence of the most recent observation merged in.
        x: World X position in meters (running average of merges).
        y: World Y position in meters (running average of merges).
        timestamp: Time (epoch seconds) of the latest observation.
    """

    id: int
    cls: str
    conf: float
    x: float
    y: float
    timestamp: float


class HumanTracker:
    """Maintains the deduplicated set of unique human tracks (TASK-09).

    Attributes:
        merge_radius_m: Spatial merge radius — detections within this
            distance of an existing track are considered the same person.
        _tracks: All known tracks (append-only except in-place merges).
        _id_counter: itertools counter producing unique track ids.
    """

    def __init__(self, merge_radius_m: float = 0.8) -> None:
        """Initialize an empty tracker.

        Args:
            merge_radius_m: Distance threshold (meters) below which two
                detections are merged into one track (default 0.8m).
        """
        self.merge_radius_m = merge_radius_m
        self._tracks: list[HumanTrack] = []
        self._id_counter = itertools.count(1)

    @staticmethod
    def _priority(cls: str) -> int:
        """Priority score for a detection class (higher = more authoritative).

        Args:
            cls: Detection class name.

        Returns:
            Integer priority: standing/cv_standing=3, fallen/cv_fallen=2,
            cv/person=1, wifi=0, unknown=0.

        Algorithm:
          Look cls up in _PRIORITY; default 0 for unrecognized classes so
          unknown labels never beat a known one on merge.
        """
        return _PRIORITY.get(cls, 0)

    def _find_nearby(self, x: float, y: float) -> HumanTrack | None:
        """Find the closest existing track within merge_radius_m.

        Args:
            x: Detection world X in meters.
            y: Detection world Y in meters.

        Returns:
            The nearest track whose distance to (x, y) is <= merge_radius_m,
            or None if no track is close enough.

        Algorithm:
          1. best_d starts at merge_radius_m (the acceptance threshold).
          2. For each track compute Euclidean distance; any track with
             d <= best_d becomes the new best and tightens best_d.
          3. Result: the single closest qualifying track (or None).
        """
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

        Args:
            cls: Detection class ("standing", "fallen", "cv", "wifi", ...).
            conf: Detection confidence (0..1), stored on merge only if the
                new class's priority >= the existing track's priority.
            x: Detection world X in meters.
            y: Detection world Y in meters.
            now: Timestamp (epoch seconds); defaults to time.time().

        Returns:
            (track, is_new): the affected HumanTrack, and True only when a
            brand-new track was created.

        Algorithm:
          1. now = provided time or time.time().
          2. Look for an existing track within merge_radius_m.
          3. None found → create a new HumanTrack with the next id, append
             it, return (track, True).
          4. Otherwise merge (spatial cooldown IS the merge radius — a
             detection near an existing track is "the same person",
             regardless of how much time has passed, unlike the old
             time-only cooldown):
             a. If new class priority >= existing priority, upgrade the
                track's cls/conf (WiFi can't downgrade a CV hit, but CV
                can upgrade a WiFi guess).
             b. Position = midpoint of old and new (centroid smoothing).
             c. timestamp = now.
          5. Return (existing track, False).
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
        """Number of unique tracks (i.e. unique humans detected)."""
        return len(self._tracks)

    def all_tracks(self) -> list[HumanTrack]:
        """Snapshot of all tracks for safe iteration/rendering.

        Returns:
            A new list containing copies of the track references (so the
            caller can iterate without racing list mutations).
        """
        return list(self._tracks)
