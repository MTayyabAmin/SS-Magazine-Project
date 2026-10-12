"""Occupancy grid built from ultrasonic (sonar) rays — Domain D2.

TASK-04 -- Occupancy Grid from Ultrasonics
TASK-05 -- Visited Cells + Frontiers

This is what A* (TASK-06) plans on top of. It is a completely separate,
Python-only structure from the ASCII TentativeMap -- that one is for
humans to read in a terminal; this one is for the planner.

Cell states (stored sparsely in a dict; unlisted cells are UNKNOWN):
  FREE     '.' — confirmed empty along a sonar ray
  OCCUPIED '#' — wall at the ray endpoint
  UNKNOWN  ' ' — never observed
  VISITED  ',' — a FREE cell the robot physically passed through

Key invariants:
  - OCCUPIED always wins over a later FREE graze (walls are not erased
    by rays from another angle).
  - Only readings inside [MIN_VALID_CM, MAX_VALID_CM] mark anything —
    HC-SR04 data outside that band is noise.
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

# ASCII glyph used when rendering the grid for files/HUD
_GLYPH = {FREE: ".", OCCUPIED: "#", UNKNOWN: " ", VISITED: ","}


@dataclass
class OccupancyGrid:
    """Sparse 2D occupancy grid in world meters (5cm cells by default).

    The grid is centered on the world origin: cell (width//2, height//2)
    equals world (0, 0), and world +Y maps to SMALLER grid rows (screen
    convention, same as TentativeMap).

    Attributes:
        cell_size_m: Edge length of one cell in meters (default 0.05).
        width: Grid width in cells (x direction).
        height: Grid height in cells (y direction).
        _cells: Sparse mapping (cx, cy) → state constant; absent = UNKNOWN.
    """

    cell_size_m: float = 0.05  # 5cm cells, per TASK-04
    width: int = 200
    height: int = 200
    _cells: dict[tuple[int, int], int] = field(default_factory=dict)

    def _origin(self) -> tuple[int, int]:
        """Grid coordinates of world (0, 0) — the geometric center.

        Returns:
            (width // 2, height // 2).
        """
        return self.width // 2, self.height // 2

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Convert world meters to grid cell indices.

        Args:
            x_m: World X in meters (right-positive).
            y_m: World Y in meters (forward-positive).

        Returns:
            (cx, cy) integer cell indices (may lie outside the grid for
            distant points — callers bounds-check as needed).

        Algorithm:
          cx = round(x / cell) + origin_x;
          cy = origin_y - round(y / cell)  (Y axis is flipped).
        """
        ox, oy = self._origin()
        cx = int(round(x_m / self.cell_size_m)) + ox
        cy = oy - int(round(y_m / self.cell_size_m))
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        """Convert grid cell indices back to world meters.

        Args:
            cx: Grid column index.
            cy: Grid row index.

        Returns:
            (x_m, y_m) world coordinates of the cell CENTER-ish (cell
            origin corner, consistent with world_to_cell rounding).

        Algorithm:
          x = (cx - origin_x) * cell; y = (origin_y - cy) * cell.
        """
        ox, oy = self._origin()
        x_m = (cx - ox) * self.cell_size_m
        y_m = (oy - cy) * self.cell_size_m
        return x_m, y_m

    def get(self, cx: int, cy: int) -> int:
        """Read a cell's state.

        Args:
            cx: Grid column index.
            cy: Grid row index.

        Returns:
            State constant (FREE/OCCUPIED/VISITED) or UNKNOWN if never set.
        """
        return self._cells.get((cx, cy), UNKNOWN)

    def _set(self, cx: int, cy: int, state: int) -> None:
        """Write a cell's state with wall-priority protection.

        Args:
            cx: Grid column index.
            cy: Grid row index.
            state: New state constant to store.

        Algorithm:
          1. Ignore out-of-bounds coordinates.
          2. If the cell is currently OCCUPIED and the new state is FREE,
             skip the write — a grazing ray from another angle must not
             erase a confirmed wall.
          3. Otherwise store the new state.
        """
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
        treated as unusable and ignored entirely (never marks anything).

        Args:
            robot_x_m: Robot X position in meters.
            robot_y_m: Robot Y position in meters.
            yaw_deg: Ray direction in degrees (world convention).
            dist_cm: Filtered sonar distance in centimeters.

        Algorithm:
          1. Range gate: dist outside [MIN_VALID_CM, MAX_VALID_CM] → return.
          2. Convert yaw to radians and dist to meters.
          3. steps = max(1, dist / cell_size) — one step per cell of length.
          4. For i in [0, steps): interpolate the point at fraction i/steps
             along the ray and mark its cell FREE (clearance).
          5. Mark the endpoint cell OCCUPIED (the wall hit).
        """
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
        re-explored / re-picked as a frontier).

        Args:
            x_m: Robot X in meters.
            y_m: Robot Y in meters.

        Algorithm:
          1. Convert position to a cell.
          2. If that cell is not OCCUPIED (you can't stand in a wall),
             set its state to VISITED.
        """
        cx, cy = self.world_to_cell(x_m, y_m)
        if self._cells.get((cx, cy), UNKNOWN) != OCCUPIED:
            self._cells[(cx, cy)] = VISITED

    def frontiers(self) -> list[tuple[int, int]]:
        """TASK-05: UNKNOWN cells that touch a FREE/VISITED cell --
        these are the candidate "next place to explore" targets.

        Returns:
            Deduplicated list of (cx, cy) frontier cells (4-connected
            neighbourhood).

        Algorithm:
          1. For every FREE/VISITED cell:
          2. Check its 4 orthogonal neighbours; skip out-of-bounds.
          3. If any neighbour is UNKNOWN, add it to the result set.
          4. Return list(set) — each frontier cell appears once.
        """
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
        reachable area has already been explored.

        Args:
            x_m: Robot X in meters.
            y_m: Robot Y in meters.

        Returns:
            World (x, y) of the closest frontier cell, or None when no
            frontier cells exist.

        Algorithm:
          1. Collect all frontiers; return None if empty.
          2. Convert the robot position to a cell.
          3. Pick the frontier minimizing squared cell distance
             (squared Euclidean == true Euclidean for argmin).
          4. Convert the winning cell back to world meters.
        """
        frs = self.frontiers()
        if not frs:
            return None
        cx0, cy0 = self.world_to_cell(x_m, y_m)
        best = min(frs, key=lambda c: (c[0] - cx0) ** 2 + (c[1] - cy0) ** 2)
        return self.cell_to_world(*best)

    def render_ascii(self, robot_x_m: float, robot_y_m: float, radius_cells: int = 30) -> str:
        """Small ASCII crop centered on the robot, for the HUD (TASK-12).

        Args:
            robot_x_m: Robot X in meters (crop center).
            robot_y_m: Robot Y in meters (crop center).
            radius_cells: Half-size of the crop in cells (default 30 → 61x61).

        Returns:
            Multi-line string of glyphs with '@' at the robot cell.

        Algorithm:
          1. Convert the robot position to a cell.
          2. For each row in [cy-radius, cy+radius] and column in
             [cx-radius, cx+radius]: '@' at the robot cell, otherwise the
             glyph of the cell state (missing cells render as space).
          3. Join rows with newlines.
        """
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
        """Write the grid crop (with a header) to a UTF-8 text file.

        Args:
            path: Destination file path (parents created as needed).
            robot_x_m: Robot X for the crop center.
            robot_y_m: Robot Y for the crop center.

        Algorithm:
          1. mkdir parents.
          2. Header lines: title, legend, known-cell and frontier counts.
          3. Append the render_ascii() crop.
          4. Write the joined text.
        """
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
