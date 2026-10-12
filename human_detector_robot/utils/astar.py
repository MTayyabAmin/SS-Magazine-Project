"""A* path planner over the OccupancyGrid -- Domain D3.

TASK-06 -- A* Planner

4- or 8-connected, inflates OCCUPIED cells by 1 cell so the planned path
doesn't hug walls. Grid is tiny (a few hundred cells at most for a room),
so this is fine on a laptop CPU -- no external deps.

Integration:
  main_controller plans from the robot's cell to the nearest frontier,
  converts the cell path to world waypoints, and hands each waypoint's
  bearing to the FSM as target_heading_deg. If astar() returns None the
  controller simply falls back to the FSM's rotate-and-scan behavior.
"""

from __future__ import annotations

import heapq
import math

from utils.occupancy_grid import OccupancyGrid, OCCUPIED

Cell = tuple[int, int]


def _inflate_occupied(grid: OccupancyGrid, radius_cells: int = 1) -> set[Cell]:
    """OCCUPIED cells plus a `radius_cells` safety margin around each --
    treated as blocked for planning purposes.

    Args:
        grid: The occupancy grid to read walls from.
        radius_cells: Inflation radius in cells (default 1 → 3x3 block
            per wall cell, keeping the robot one cell off walls).

    Returns:
        Set of all blocked cell coordinates (walls + margins).

    Algorithm:
      1. Collect every cell whose state == OCCUPIED.
      2. For each wall cell, add all (dx, dy) offsets in
         [-radius, +radius]² — a square (Chebyshev) dilation.
      3. Return the union as a set.
    """
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
    to the existing rotate-and-scan behavior (TASK-06 requirement).

    Args:
        grid: Occupancy grid to plan over.
        start_cell: Robot's current cell (cx, cy).
        goal_cell: Target cell (e.g. nearest frontier).
        connectivity: 8 (default) or 4 — neighbour set for expansion.
        inflate_radius: Wall inflation radius in cells (default 1).

    Returns:
        List of cells from start to goal inclusive, or None when the goal
        is blocked, unreachable, or the search budget is exhausted.

    Algorithm:
      1. blocked = inflated OCCUPIED set; if the goal itself is blocked →
         None (can't plan into a wall).
      2. start == goal → return [start] immediately.
      3. Standard A* with:
         - g = cumulative step cost (hypot(dx,dy): 1.0 orthogonal,
           ~1.414 diagonal for 8-connectivity),
         - h = Euclidean distance to goal (admissible/consistent),
         - open list as a min-heap keyed on f = g + h,
         - closed set to avoid re-expanding.
      4. Expand neighbours: skip blocked, closed, and out-of-bounds cells;
         relax g and push improved entries.
      5. On reaching the goal, reconstruct the path via came_from and
         return it start→goal ordered.
      6. Give up after max_iters (20000) expansions so a degenerate grid
         can never hang the control loop; exhaustion → None.
    """
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
        """Euclidean heuristic (admissible — never overestimates)."""
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
    """Grid-cell path -> world-frame (x, y) meters waypoints.

    Args:
        grid: The grid whose coordinate mapping is used.
        path_cells: Ordered cells from astar() (start..goal).

    Returns:
        List of world (x, y) tuples, one per cell, in the same order.

    Algorithm:
      Map each cell through grid.cell_to_world().
    """
    return [grid.cell_to_world(cx, cy) for (cx, cy) in path_cells]


def next_heading_deg(current_xy: tuple[float, float], waypoint_xy: tuple[float, float]) -> float:
    """Bearing (degrees, same convention as MPU yaw: 0=+X axis) from the
    robot's current position to the next waypoint it should steer toward.

    Args:
        current_xy: Robot world position (x, y) in meters.
        waypoint_xy: Target waypoint (x, y) in meters.

    Returns:
        Absolute bearing in [0, 360) degrees.

    Algorithm:
      1. dx = wx - cx; dy = wy - cy.
      2. degrees(atan2(dy, dx)) — atan2 gives the correct quadrant.
      3. Modulo 360 to keep the result in [0, 360).
    """
    dx = waypoint_xy[0] - current_xy[0]
    dy = waypoint_xy[1] - current_xy[1]
    return math.degrees(math.atan2(dy, dx)) % 360.0
