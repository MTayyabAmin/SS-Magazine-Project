"""Occupancy grid built from ultrasonic (sonar) rays — Domain D2.

TASK-04 -- Occupancy Grid from Ultrasonics
TASK-05 -- Visited Cells + Frontiers

This is what A* (TASK-06) plans on top of. It is a completely separate,
Python-only structure from the ASCII TentativeMap -- that one is for
humans to read in a terminal; this one is for the planner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

FREE = 0
OCCUPIED = 1
UNKNOWN = 2
VISITED = 3  # a FREE cell the robot has physically passed through

# TASK-04: only trust a sonar reading in this physical range. Below 15cm
# the HC-SR04 datasheet is unreliable; above 300cm echoes get noisy.
MIN_VALID_CM = 15
MAX_VALID_CM = 300

_GLYPH = {FREE: ".", OCCUPIED: "#", UNKNOWN: " ", VISITED: ","}


@dataclass
class OccupancyGrid:
    cell_size_m: float = 0.05  # 5cm cells, per TASK-04
    width: int = 200
    height: int = 200
    _cells: dict[tuple[int, int], int] = field(default_factory=dict)

    def _origin(self) -> tuple[int, int]:
        return self.width // 2, self.height // 2

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        ox, oy = self._origin()
        cx = int(round(x_m / self.cell_size_m)) + ox
        cy = oy - int(round(y_m / self.cell_size_m))
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        ox, oy = self._origin()
        x_m = (cx - ox) * self.cell_size_m
        y_m = (oy - cy) * self.cell_size_m
        return x_m, y_m

    def get(self, cx: int, cy: int) -> int:
        return self._cells.get((cx, cy), UNKNOWN)

    def _set(self, cx: int, cy: int, state: int) -> None:
        if not (0 <= cx < self.width and 0 <= cy < self.height):
            return
        current = self._cells.get((cx, cy), UNKNOWN)
        # An OCCUPIED cell is a wall -- a later FREE ray grazing the same
        # cell (from a different angle) should not erase it.
        if current == OCCUPIED and state == FREE:
            return
        self._cells[(cx, cy)] = state

    def mark_ray(self, robot_x_m: float, robot_y_m: float, yaw_deg: float, dist_cm: int) -> None:
        """TASK-04: walk the sonar ray cell-by-cell -- FREE along the way,
        OCCUPIED at the hit point. A reading outside [15, 300]cm is
        treated as unusable and ignored entirely (never marks anything)."""
        if dist_cm < MIN_VALID_CM or dist_cm > MAX_VALID_CM:
            return

        yaw_rad = math.radians(yaw_deg)
        dist_m = dist_cm / 100.0
        steps = max(1, int(dist_m / self.cell_size_m))

        for i in range(steps):
            frac = i / steps
            fx = robot_x_m + dist_m * frac * math.cos(yaw_rad)
            fy = robot_y_m + dist_m * frac * math.sin(yaw_rad)
            cx, cy = self.world_to_cell(fx, fy)
            self._set(cx, cy, FREE)

        hx = robot_x_m + dist_m * math.cos(yaw_rad)
        hy = robot_y_m + dist_m * math.sin(yaw_rad)
        hcx, hcy = self.world_to_cell(hx, hy)
        self._set(hcx, hcy, OCCUPIED)

    def mark_visited(self, x_m: float, y_m: float) -> None:
        """TASK-05: mark the robot's own cell as VISITED (so it isn't
        re-explored / re-picked as a frontier)."""
        cx, cy = self.world_to_cell(x_m, y_m)
        if self._cells.get((cx, cy), UNKNOWN) != OCCUPIED:
            self._cells[(cx, cy)] = VISITED

    def frontiers(self) -> list[tuple[int, int]]:
        """TASK-05: UNKNOWN cells that touch a FREE/VISITED cell --
        these are the candidate "next place to explore" targets."""
        result: set[tuple[int, int]] = set()
        for (cx, cy), state in self._cells.items():
            if state not in (FREE, VISITED):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (cx + dx, cy + dy)
                if not (0 <= n[0] < self.width and 0 <= n[1] < self.height):
                    continue
                if self._cells.get(n, UNKNOWN) == UNKNOWN:
                    result.add(n)
        return list(result)

    def nearest_frontier(self, x_m: float, y_m: float) -> tuple[float, float] | None:
        """TASK-05: next exploration goal = nearest frontier cell,
        converted back into world (x, y) meters. None if the whole
        reachable area has already been explored."""
        frs = self.frontiers()
        if not frs:
            return None
        cx0, cy0 = self.world_to_cell(x_m, y_m)
        best = min(frs, key=lambda c: (c[0] - cx0) ** 2 + (c[1] - cy0) ** 2)
        return self.cell_to_world(*best)

    def render_ascii(self, robot_x_m: float, robot_y_m: float, radius_cells: int = 30) -> str:
        """Small ASCII crop centered on the robot, for the HUD (TASK-12)."""
        rcx, rcy = self.world_to_cell(robot_x_m, robot_y_m)
        lines = []
        for gy in range(rcy - radius_cells, rcy + radius_cells + 1):
            row = []
            for gx in range(rcx - radius_cells, rcx + radius_cells + 1):
                if gx == rcx and gy == rcy:
                    row.append("@")
                else:
                    row.append(_GLYPH[self._cells.get((gx, gy), UNKNOWN)])
            lines.append("".join(row))
        return "\n".join(lines)

    def save(self, path: Path, robot_x_m: float = 0.0, robot_y_m: float = 0.0) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        header = [
            "================================================================",
            "  ARK-5 OCCUPANCY GRID (planner map -- TASK-04/05)",
            "  Legend: '.'=free  '#'=occupied/wall  ','=visited  ' '=unknown  '@'=robot",
            f"  Cells known: {len(self._cells)} | Frontiers: {len(self.frontiers())}",
            "================================================================",
            "",
        ]
        body = self.render_ascii(robot_x_m, robot_y_m)
        path.write_text("\n".join(header) + body + "\n", encoding="utf-8")
