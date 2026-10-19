"""Simulation-only navigation: occupancy grid, A* planning and route following.

Coordinates match the simulator: metres, origin at the room's bottom-left
corner, x to the east, y to the north. Grid cell (col, row) covers
[col*res, (col+1)*res) x [row*res, (row+1)*res).

The planner works on a *virtual* map that is known exactly. A planned route
says nothing about real-world collision safety: a real robot would need
localisation and much better obstacle sensing (see README).

Navigation never changes the robot's state. The controller asks the
``Navigator`` for a motion command on each step and applies all of its
normal safety checks before moving.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from enum import StrEnum

from src.config import NavigationConfig
from src.models import Pose
from src.simulator import World, circle_collision, normalize_angle

Cell = tuple[int, int]  # (col, row)
Point = tuple[float, float]

# 8-connected moves: (d_col, d_row, cost in cells). Order is fixed for determinism.
_MOVES = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
)


@dataclass(frozen=True)
class OccupancyGrid:
    resolution_m: float
    cols: int
    rows: int
    inflation_radius_m: float
    occupied: frozenset[Cell]  # cells that overlap an obstacle (the raw map)
    blocked: frozenset[Cell]   # cells where the robot centre may not go (obstacles + walls, inflated)
    # Extra cost multiplier (>= 1) for entering each free cell; higher near obstacles.
    cell_cost: dict[Cell, float] = field(default_factory=dict, compare=False, repr=False)

    @property
    def width_m(self) -> float:
        return self.cols * self.resolution_m

    @property
    def height_m(self) -> float:
        return self.rows * self.resolution_m

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.cols and 0 <= cell[1] < self.rows

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.blocked

    def cell_of(self, x: float, y: float) -> Cell | None:
        """Cell containing a point, or None if the point is outside the room."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        if not (0 <= x <= self.width_m and 0 <= y <= self.height_m):
            return None
        col = min(_floor_div(x, self.resolution_m), self.cols - 1)
        row = min(_floor_div(y, self.resolution_m), self.rows - 1)
        return (col, row)

    def centre(self, cell: Cell) -> Point:
        return ((cell[0] + 0.5) * self.resolution_m, (cell[1] + 0.5) * self.resolution_m)


def _floor_div(value: float, res: float) -> int:
    # The tiny epsilon stops 0.6 / 0.05 = 11.999... landing in the wrong cell.
    return math.floor(value / res + 1e-9)


def _ceil_div(value: float, res: float) -> int:
    return math.ceil(value / res - 1e-9)


def build_occupancy_grid(world: World, robot_radius_m: float, config: NavigationConfig) -> OccupancyGrid:
    """Rasterise the room.

    A cell is *blocked* when a circle of radius (robot radius + safety margin)
    centred on the cell would overlap a wall or obstacle. This reuses the
    simulator's own collision test, so the grid and the simulator agree.
    """
    res = config.resolution_m
    cols = round(world.width_m / res)
    rows = round(world.height_m / res)
    if abs(cols * res - world.width_m) > 1e-9 or abs(rows * res - world.height_m) > 1e-9:
        raise ValueError(f"Room size must be a whole number of {res} m grid cells")
    inflation = robot_radius_m + config.safety_margin_m

    occupied: set[Cell] = set()
    for o in world.obstacles:
        for col in range(_floor_div(o.x, res), min(_ceil_div(o.x_max, res), cols)):
            for row in range(_floor_div(o.y, res), min(_ceil_div(o.y_max, res), rows)):
                occupied.add((col, row))

    blocked: set[Cell] = set()
    cell_cost: dict[Cell, float] = {}
    for col in range(cols):
        for row in range(rows):
            x, y = (col + 0.5) * res, (row + 0.5) * res
            if circle_collision(world, x, y, inflation) is not None:
                blocked.add((col, row))
                continue
            shortfall = max(0.0, config.preferred_clearance_m - clearance_m(world, x, y))
            cell_cost[(col, row)] = 1.0 + config.clearance_weight * shortfall / config.preferred_clearance_m
    return OccupancyGrid(res, cols, rows, inflation, frozenset(occupied), frozenset(blocked), cell_cost)


def clearance_m(world: World, x: float, y: float) -> float:
    """Distance from a point to the nearest wall or obstacle surface."""
    nearest = min(x, world.width_m - x, y, world.height_m - y)
    for o in world.obstacles:
        dx = max(o.x - x, 0.0, x - o.x_max)
        dy = max(o.y - y, 0.0, y - o.y_max)
        nearest = min(nearest, math.hypot(dx, dy))
    return nearest


class PlanStatus(StrEnum):
    OK = "OK"
    INVALID_START = "INVALID_START"
    INVALID_GOAL = "INVALID_GOAL"
    UNREACHABLE = "UNREACHABLE"


@dataclass(frozen=True)
class PlanResult:
    status: PlanStatus
    reason: str
    start: Point
    goal: Point
    cells: tuple[Cell, ...] = ()
    waypoints: tuple[Point, ...] = ()  # points the robot drives through, ending exactly at the goal

    @property
    def ok(self) -> bool:
        return self.status is PlanStatus.OK

    @property
    def length_m(self) -> float:
        points = (self.start, *self.waypoints)
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def plan_path(grid: OccupancyGrid, start: Point, goal: Point) -> PlanResult:
    """A* from start to goal on the inflated grid (8-connected, no corner cutting)."""
    start_cell = grid.cell_of(*start)
    if start_cell is None:
        return PlanResult(PlanStatus.INVALID_START, "Start is outside the room", start, goal)
    if not grid.is_free(start_cell):
        return PlanResult(PlanStatus.INVALID_START,
                          "Robot is too close to a wall or obstacle to plan from here", start, goal)
    goal_cell = grid.cell_of(*goal)
    if goal_cell is None:
        return PlanResult(PlanStatus.INVALID_GOAL,
                          f"Goal ({goal[0]}, {goal[1]}) is outside the room "
                          f"(0-{grid.width_m:g} m by 0-{grid.height_m:g} m)", start, goal)
    if not grid.is_free(goal_cell):
        return PlanResult(PlanStatus.INVALID_GOAL,
                          f"Goal ({goal[0]:.2f}, {goal[1]:.2f}) is inside or too close to a wall or obstacle "
                          f"(needs {grid.inflation_radius_m:.2f} m clearance)", start, goal)

    cells = _astar(grid, start_cell, goal_cell)
    if cells is None:
        return PlanResult(PlanStatus.UNREACHABLE,
                          "No collision-free route on the grid: the goal is cut off by obstacles "
                          "or gaps are too narrow for the robot", start, goal)
    waypoints = _waypoints(grid, cells, start, goal)
    return PlanResult(PlanStatus.OK, f"Route found: {len(cells)} cells", start, goal, tuple(cells), waypoints)


def _astar(grid: OccupancyGrid, start: Cell, goal: Cell) -> list[Cell] | None:
    def heuristic(c: Cell) -> float:  # octile distance: admissible for 8-connected moves
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    counter = 0  # tie-breaker keeps the expansion order deterministic
    open_heap: list[tuple[float, float, int, Cell]] = [(heuristic(start), 0.0, counter, start)]
    came_from: dict[Cell, Cell] = {}
    best_g: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while open_heap:
        _, g, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        closed.add(current)
        for dc, dr, cost in _MOVES:
            nxt = (current[0] + dc, current[1] + dr)
            if not grid.is_free(nxt) or nxt in closed:
                continue
            # No corner cutting: a diagonal move needs both side cells free too.
            if dc and dr and not (grid.is_free((current[0] + dc, current[1])) and grid.is_free((current[0], current[1] + dr))):
                continue
            new_g = g + cost * grid.cell_cost.get(nxt, 1.0)
            if new_g < best_g.get(nxt, math.inf) - 1e-12:
                best_g[nxt] = new_g
                came_from[nxt] = current
                counter += 1
                heapq.heappush(open_heap, (new_g + heuristic(nxt), new_g, counter, nxt))
    return None


def _waypoints(grid: OccupancyGrid, cells: list[Cell], start: Point, goal: Point) -> tuple[Point, ...]:
    """Cell centres where the path changes direction, then the exact goal.

    The start-cell centre is kept as the first waypoint so every driven
    segment stays inside the chain of free cells.
    """
    corners = [cells[0]]
    for prev, cur, nxt in zip(cells, cells[1:], cells[2:]):
        if (cur[0] - prev[0], cur[1] - prev[1]) != (nxt[0] - cur[0], nxt[1] - cur[1]):
            corners.append(cur)
    if len(cells) > 1:
        corners.append(cells[-1])
    points = [grid.centre(c) for c in corners]
    if math.dist(points[0], start) < 1e-9:
        points.pop(0)
    if not points or math.dist(points[-1], goal) > 1e-9:
        points.append(goal)
    return tuple(points)


# ---------------------------------------------------------------- following

class RouteStatus(StrEnum):
    ACTIVE = "ACTIVE"
    ARRIVED = "ARRIVED"
    BLOCKED = "BLOCKED"


@dataclass(frozen=True)
class MotionRequest:
    """What the navigator would like the controller to do on this step."""

    linear_mps: float = 0.0
    angular_rps: float = 0.0
    arrived: bool = False
    blocked_reason: str | None = None


class Navigator:
    """Follows a planned route. It only *requests* motion; the controller decides."""

    def __init__(self, plan: PlanResult, label: str, config: NavigationConfig) -> None:
        if not plan.ok:
            raise ValueError(f"Cannot follow a failed plan: {plan.reason}")
        self.plan = plan
        self.label = label
        self.config = config
        self.index = 0
        self.status = RouteStatus.ACTIVE
        self.blocked_reason: str | None = None

    @property
    def goal(self) -> Point:
        return self.plan.goal

    @property
    def remaining_waypoints(self) -> tuple[Point, ...]:
        return self.plan.waypoints[self.index:]

    def block(self, reason: str) -> None:
        self.status = RouteStatus.BLOCKED
        self.blocked_reason = reason

    def replace_plan(self, plan: PlanResult) -> None:
        if not plan.ok:
            raise ValueError(plan.reason)
        self.plan = plan
        self.index = 0
        self.status = RouteStatus.ACTIVE
        self.blocked_reason = None

    def request(self, pose: Pose, max_linear_mps: float, max_angular_rps: float, dt_s: float) -> MotionRequest:
        if self.status is RouteStatus.BLOCKED:
            return MotionRequest(blocked_reason=self.blocked_reason)
        waypoints = self.plan.waypoints
        tol = self.config.waypoint_tolerance_m
        while self.index < len(waypoints) and math.dist((pose.x, pose.y), waypoints[self.index]) <= tol:
            self.index += 1
        if self.index >= len(waypoints):
            self.status = RouteStatus.ARRIVED
            return MotionRequest(arrived=True)

        tx, ty = waypoints[self.index]
        distance = math.dist((pose.x, pose.y), (tx, ty))
        error = normalize_angle(math.atan2(ty - pose.y, tx - pose.x) - pose.heading_rad)
        angular = max(-max_angular_rps, min(max_angular_rps, self.config.heading_gain * error))
        if abs(error) > math.radians(self.config.rotate_in_place_deg):
            return MotionRequest(0.0, angular)  # turn on the spot towards the next waypoint
        # Never overshoot the waypoint in one step.
        linear = min(max_linear_mps, distance / dt_s)
        return MotionRequest(linear, angular)
