"""A* path planner over the OccupancyGrid -- Domain D3.

TASK-06 -- A* Planner

4- or 8-connected, inflates OCCUPIED cells by 1 cell so the planned path
doesn't hug walls. Grid is tiny (a few hundred cells at most for a room),
so this is fine on a laptop CPU -- no external deps.
"""

from __future__ import annotations

import heapq
import math

from utils.occupancy_grid import OccupancyGrid, OCCUPIED

Cell = tuple[int, int]


def _inflate_occupied(grid: OccupancyGrid, radius_cells: int = 1) -> set[Cell]:
    """OCCUPIED cells plus a `radius_cells` safety margin around each --
    treated as blocked for planning purposes."""
    occ = {c for c, s in grid._cells.items() if s == OCCUPIED}
    inflated: set[Cell] = set(occ)
    for (cx, cy) in occ:
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                inflated.add((cx + dx, cy + dy))
    return inflated


def astar(
    grid: OccupancyGrid,
    start_cell: Cell,
    goal_cell: Cell,
    connectivity: int = 8,
    inflate_radius: int = 1,
) -> list[Cell] | None:
    """Returns the grid-cell path (start..goal inclusive), or None if no
    path exists -- caller (main_controller / FSM) should then fall back
    to the existing rotate-and-scan behavior (TASK-06 requirement)."""
    blocked = _inflate_occupied(grid, inflate_radius)
    if goal_cell in blocked:
        return None
    if start_cell == goal_cell:
        return [start_cell]

    neighbors = (
        [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        if connectivity == 8
        else [(-1, 0), (1, 0), (0, -1), (0, 1)]
    )

    def h(a: Cell, b: Cell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: list[tuple[float, float, Cell]] = [(h(start_cell, goal_cell), 0.0, start_cell)]
    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start_cell: 0.0}
    closed: set[Cell] = set()

    # Cap iterations so a huge/degenerate grid can't hang the main loop.
    max_iters = 20000
    iters = 0

    while open_heap and iters < max_iters:
        iters += 1
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == goal_cell:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return path

        for dx, dy in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt in blocked or nxt in closed:
                continue
            if not (0 <= nxt[0] < grid.width and 0 <= nxt[1] < grid.height):
                continue
            step_cost = math.hypot(dx, dy)
            tentative_g = g + step_cost
            if tentative_g < g_score.get(nxt, float("inf")):
                g_score[nxt] = tentative_g
                came_from[nxt] = current
                heapq.heappush(open_heap, (tentative_g + h(nxt, goal_cell), tentative_g, nxt))

    return None  # no path found (or search exhausted) -- fall back to FSM scan


def path_to_waypoints(grid: OccupancyGrid, path_cells: list[Cell]) -> list[tuple[float, float]]:
    """Grid-cell path -> world-frame (x, y) meters waypoints."""
    return [grid.cell_to_world(cx, cy) for (cx, cy) in path_cells]


def next_heading_deg(current_xy: tuple[float, float], waypoint_xy: tuple[float, float]) -> float:
    """Bearing (degrees, same convention as MPU yaw: 0=+X axis) from the
    robot's current position to the next waypoint it should steer toward."""
    dx = waypoint_xy[0] - current_xy[0]
    dy = waypoint_xy[1] - current_xy[1]
    return math.degrees(math.atan2(dy, dx)) % 360.0
