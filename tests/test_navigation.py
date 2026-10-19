"""Occupancy grid, A* planning and navigation through the controller (simulation only)."""

import math

import pytest

from src.config import NavigationConfig, SimulationConfig
from src.models import Obstacle, Pose, RobotState
from src.navigation import (
    MotionRequest,
    Navigator,
    OccupancyGrid,
    PlanStatus,
    build_occupancy_grid,
    clearance_m,
    plan_path,
)
from src.scenarios import DESTINATIONS, SCENARIOS, build_controller
from src.simulator import World, circle_collision
from tests.helpers import fixed_clock, make_geometric, make_scripted

S = RobotState
NAV = NavigationConfig()
RADIUS = 0.15
EMPTY = World(4.0, 3.0, ())
BOX_A = Obstacle("Box A", 2.6, 1.2, 0.4, 0.6)


def grid_for(*obstacles, config=NAV):
    return build_occupancy_grid(World(4.0, 3.0, tuple(obstacles)), RADIUS, config)


def route_points(plan):
    return (plan.start, *plan.waypoints)


def assert_route_clear(world, plan, radius=RADIUS, step=0.005):
    """Every point along the planned polyline keeps the robot body clear."""
    points = route_points(plan)
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist((x1, y1), (x2, y2)) / step))
        for i in range(n + 1):
            x, y = x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n
            assert circle_collision(world, x, y, radius) is None, f"route touches something at ({x:.3f}, {y:.3f})"


def drive(c, limit=5000):
    for _ in range(limit):
        if c.state is not S.MOVING:
            return
        c.tick()
        assert c.sim.collision_at(c.sim.pose) is None
    raise AssertionError("robot never stopped")


# ------------------------------------------------------------------ grid

def test_grid_matches_room_size_and_resolution():
    g = grid_for()
    assert (g.cols, g.rows, g.resolution_m) == (80, 60, 0.05)
    assert (g.width_m, g.height_m) == pytest.approx((4.0, 3.0))
    assert g.inflation_radius_m == pytest.approx(0.20)


def test_room_must_be_whole_number_of_cells():
    with pytest.raises(ValueError):
        build_occupancy_grid(World(4.03, 3.0, ()), RADIUS, NAV)


def test_safety_margin_must_cover_half_cell_diagonal():
    with pytest.raises(ValueError):
        NavigationConfig(resolution_m=0.1, safety_margin_m=0.05)


@pytest.mark.parametrize(
    "point, cell",
    [((0.0, 0.0), (0, 0)), ((0.6, 1.5), (12, 30)), ((0.049, 0.051), (0, 1)),
     ((4.0, 3.0), (79, 59)), ((3.999, 2.999), (79, 59))],
)
def test_cell_lookup_and_boundaries(point, cell):
    assert grid_for().cell_of(*point) == cell


@pytest.mark.parametrize("point", [(-0.01, 1.0), (4.01, 1.0), (1.0, -0.001), (1.0, 3.001), (math.nan, 1.0), (1.0, math.inf)])
def test_points_outside_room_have_no_cell(point):
    assert grid_for().cell_of(*point) is None


def test_cell_centre_round_trip():
    g = grid_for()
    for cell in [(0, 0), (12, 30), (79, 59)]:
        assert g.cell_of(*g.centre(cell)) == cell


def test_occupied_cells_cover_obstacle_exactly():
    g = grid_for(BOX_A)
    expected = {(c, r) for c in range(52, 60) for r in range(24, 36)}
    assert g.occupied == expected


def test_inflation_blocks_wall_band_and_obstacle_surround():
    g = grid_for(BOX_A)
    # Centre within 0.20 m of the west wall is blocked; col 4 (centre 0.225 m) is free.
    assert not g.is_free((3, 30)) and g.is_free((4, 30))
    # Box A starts at x = 2.6: col 47 centre is 2.375 m (0.225 m away) free, col 48 (0.175 m) blocked.
    assert g.is_free((47, 30)) and not g.is_free((48, 30))
    assert g.occupied <= g.blocked


def test_grid_agrees_with_simulator_collision_test():
    world = SCENARIOS["obstacle_ahead"].world
    g = build_occupancy_grid(world, RADIUS, NAV)
    for col in range(g.cols):
        for row in range(g.rows):
            x, y = g.centre((col, row))
            hit = circle_collision(world, x, y, g.inflation_radius_m)
            assert ((col, row) in g.blocked) == (hit is not None)


# --------------------------------------------------------------- planning

def test_open_path_is_straight_and_short():
    plan = plan_path(grid_for(), (1.0, 1.5), (3.0, 1.5))
    assert plan.ok
    assert plan.waypoints[-1] == (3.0, 1.5)
    assert plan.length_m == pytest.approx(2.0, abs=0.1)
    assert_route_clear(EMPTY, plan)


def test_route_goes_around_obstacle_without_touching_it():
    world = World(4.0, 3.0, (BOX_A,))
    plan = plan_path(build_occupancy_grid(world, RADIUS, NAV), (0.6, 1.5), (3.5, 1.5))
    assert plan.ok
    assert all(build_occupancy_grid(world, RADIUS, NAV).is_free(c) for c in plan.cells)
    assert plan.length_m > 2.9 + 0.1  # had to detour
    assert_route_clear(world, plan)


@pytest.mark.parametrize("key", list(SCENARIOS))
@pytest.mark.parametrize("name", list(DESTINATIONS))
def test_predefined_destinations_have_clear_routes(key, name):
    world = SCENARIOS[key].world
    start = SCENARIOS[key].start_pose
    plan = plan_path(build_occupancy_grid(world, RADIUS, NAV), (start.x, start.y), DESTINATIONS[name])
    assert plan.ok, plan.reason
    assert_route_clear(world, plan)


def test_wall_across_room_makes_goal_unreachable():
    wall = Obstacle("Partition", 2.0, 0.0, 0.1, 3.0)
    plan = plan_path(grid_for(wall), (1.0, 1.5), (3.0, 1.5))
    assert plan.status is PlanStatus.UNREACHABLE
    assert plan.waypoints == ()


def test_gap_narrower_than_inflated_footprint_is_blocked():
    # 0.35 m gap: wider than the robot (0.30 m) but narrower than robot + margins (0.40 m).
    lower, upper = Obstacle("Lower", 2.0, 0.0, 0.1, 1.325), Obstacle("Upper", 2.0, 1.675, 0.1, 1.325)
    assert plan_path(grid_for(lower, upper), (1.0, 1.5), (3.0, 1.5)).status is PlanStatus.UNREACHABLE


def test_gap_wide_enough_is_used():
    lower, upper = Obstacle("Lower", 2.0, 0.0, 0.1, 1.25), Obstacle("Upper", 2.0, 1.75, 0.1, 1.25)
    world = World(4.0, 3.0, (lower, upper))
    plan = plan_path(build_occupancy_grid(world, RADIUS, NAV), (1.0, 1.5), (3.0, 1.5))
    assert plan.ok
    assert_route_clear(world, plan)


def test_no_diagonal_corner_cutting():
    # 2x2 grid where the two side cells of the diagonal are blocked.
    g = OccupancyGrid(1.0, 2, 2, 0.0, frozenset(), frozenset({(1, 0), (0, 1)}))
    assert plan_path(g, (0.5, 0.5), (1.5, 1.5)).status is PlanStatus.UNREACHABLE


@pytest.mark.parametrize(
    "goal",
    [(5.0, 1.5), (-0.1, 1.5), (2.0, 3.2), (math.nan, 1.0), (1.0, math.inf),  # outside / not a number
     (2.8, 1.5),                                                              # inside Box A
     (2.45, 1.5),                                                             # inflated zone around Box A
     (0.1, 1.5), (2.0, 2.95), (4.0, 1.5), (0.0, 0.0)],                        # wall band and exact boundary
)
def test_invalid_goals_are_rejected(goal):
    plan = plan_path(grid_for(BOX_A), (0.6, 1.5), goal)
    assert plan.status is PlanStatus.INVALID_GOAL
    assert plan.reason


@pytest.mark.parametrize("start", [(0.1, 1.5), (2.8, 1.5), (-1.0, 1.0)])
def test_invalid_starts_are_rejected(start):
    assert plan_path(grid_for(BOX_A), start, (1.0, 1.0)).status is PlanStatus.INVALID_START


def test_goal_in_start_cell():
    plan = plan_path(grid_for(), (1.01, 1.01), (1.02, 1.02))
    assert plan.ok
    assert plan.waypoints[-1] == (1.02, 1.02)


def test_planning_is_deterministic():
    g = grid_for(BOX_A, Obstacle("Shelf", 1.2, 2.4, 1.0, 0.4))
    assert plan_path(g, (0.6, 1.5), (3.3, 2.3)) == plan_path(g, (0.6, 1.5), (3.3, 2.3))


def test_obstacle_changes_change_the_plan():
    before = plan_path(grid_for(), (1.0, 1.5), (3.0, 1.5))
    blocker = Obstacle("New box", 1.8, 1.2, 0.4, 0.6)
    after = plan_path(grid_for(blocker), (1.0, 1.5), (3.0, 1.5))
    assert before.ok and after.ok
    assert after.cells != before.cells
    assert_route_clear(World(4.0, 3.0, (blocker,)), after)
    sealed = plan_path(grid_for(blocker, Obstacle("Partition", 2.5, 0.0, 0.1, 3.0)), (1.0, 1.5), (3.0, 1.5))
    assert sealed.status is PlanStatus.UNREACHABLE


def test_clearance_cost_keeps_route_away_from_obstacles():
    world = World(4.0, 3.0, (BOX_A,))
    near = plan_path(build_occupancy_grid(world, RADIUS, NavigationConfig(clearance_weight=0.0)), (0.6, 1.5), (3.5, 1.5))
    far = plan_path(build_occupancy_grid(world, RADIUS, NAV), (0.6, 1.5), (3.5, 1.5))
    assert min(clearance_m(world, x, y) for x, y in far.waypoints[:-1]) >= \
        min(clearance_m(world, x, y) for x, y in near.waypoints[:-1])


# ------------------------------------------------- navigation via controller

def test_planning_only_while_idle_and_never_starts_motion():
    c = make_geometric()
    plan = c.plan_route((3.0, 1.5), "Test")
    assert plan.ok and c.navigator is not None
    assert c.state is S.IDLE
    c.run_ticks(50)
    assert c.sim.distance_travelled_m == 0.0
    c.start()
    with pytest.raises(ValueError):
        c.plan_route((1.0, 1.0), "Other")


def test_failed_plan_keeps_manual_mode():
    c = make_geometric()
    assert not c.plan_route((5.0, 1.0), "Nowhere").ok
    assert c.navigator is None


@pytest.mark.parametrize("key", list(SCENARIOS))
@pytest.mark.parametrize("name", list(DESTINATIONS))
def test_drive_to_predefined_destination(key, name):
    c = build_controller(key, clock=fixed_clock)
    assert c.plan_route(DESTINATIONS[name], name).ok
    assert c.start().accepted
    drive(c)
    assert c.state is S.STOPPED
    assert "Destination reached" in c.state_reason
    assert math.dist((c.sim.pose.x, c.sim.pose.y), DESTINATIONS[name]) <= NAV.waypoint_tolerance_m
    c.reset()
    assert not c.start().accepted  # already there; never restarts by itself


def test_goal_close_to_wall_ends_in_safe_obstacle_stop():
    # (3.5, 1.5) is a valid grid goal, but driving straight at it brings the east
    # wall inside the 0.5 m stop threshold first. Safety wins: latched stop, no collision.
    c = build_controller("wall_approach", clock=fixed_clock)
    assert c.plan_route((3.5, 1.5), "Near wall").ok
    c.start()
    drive(c)
    assert c.state is S.OBSTACLE_DETECTED


def test_emergency_stop_during_navigation():
    c = build_controller("obstacle_ahead", clock=fixed_clock)
    c.plan_route(DESTINATIONS["Kitchen"], "Kitchen")
    c.start()
    c.run_ticks(30)
    c.emergency_stop()
    pose = c.sim.pose
    for cmd in (c.start, c.resume):
        assert not cmd().accepted
    c.run_ticks(100)
    assert c.sim.pose == pose
    c.reset()
    c.run_ticks(100)
    assert c.state is S.IDLE and c.sim.pose == pose  # reset never resumes the route
    assert c.start().accepted  # explicit Start continues to the same goal
    drive(c)
    assert "Destination reached" in c.state_reason


def test_obstacle_stop_during_navigation_is_latched():
    c, sensor = make_scripted([2.0])
    c.plan_route((3.0, 1.5), "Goal")
    c.start()
    c.run_ticks(10)
    sensor.set(0.3)
    c.tick()
    assert c.state is S.OBSTACLE_DETECTED
    pose = c.sim.pose
    sensor.set(2.0)
    c.run_ticks(100)
    assert c.state is S.OBSTACLE_DETECTED and c.sim.pose == pose
    assert c.resume().accepted
    drive(c)
    assert "Destination reached" in c.state_reason


def test_sensor_fault_during_navigation():
    c, sensor = make_scripted([2.0])
    c.plan_route((3.0, 1.5), "Goal")
    c.start()
    c.run_ticks(10)
    sensor.set(math.nan)
    pose = c.sim.pose
    c.run_ticks(20)
    assert c.state is S.FAULT and c.sim.pose == pose


def test_obstacle_moved_into_route_triggers_replan():
    c = make_geometric()
    c.plan_route((3.3, 2.3), "Kitchen")
    c.start()
    c.run_ticks(20)
    old_cells = c.navigator.plan.cells
    crate = Obstacle("Crate", 2.2, 1.6, 0.4, 0.4)  # sits on the original route
    c.set_obstacles([crate])
    assert c.state is S.MOVING  # a new route exists, so the run continues
    assert c.navigator.plan.cells != old_cells
    assert_route_clear(World(4.0, 3.0, (crate,)), c.navigator.plan)
    assert any("re-planned" in e.message for e in c.log.entries)
    drive(c)
    assert "Destination reached: Kitchen" in c.state_reason


def test_obstacle_change_that_blocks_every_route_stops_robot():
    c = make_geometric()
    c.plan_route((3.3, 1.5), "Goal")
    c.start()
    c.run_ticks(5)
    c.set_obstacles([Obstacle("Partition", 2.5, 0.0, 0.1, 3.0)])
    assert c.state is S.STOPPED
    assert "Route blocked" in c.state_reason
    c.reset()
    result = c.start()
    assert not result.accepted and "blocked" in result.message.lower()


def test_navigation_requests_are_clamped_to_motion_limits():
    class Reckless(Navigator):
        def request(self, pose, max_linear_mps, max_angular_rps, dt_s):
            return MotionRequest(linear_mps=50.0, angular_rps=math.inf)

    c = make_geometric()
    plan = plan_path(c.occupancy_grid(), (0.6, 1.5), (3.0, 1.5))
    c.navigator = Reckless(plan, "Reckless", NAV)
    c.start()
    before = c.sim.pose
    c.tick()
    step = c.speed_mps * c.config.motion.time_step_s
    assert math.dist((before.x, before.y), (c.sim.pose.x, c.sim.pose.y)) <= step + 1e-12
    assert c.sim.pose.heading_rad == before.heading_rad  # infinite turn request replaced by 0


def test_clear_route_returns_to_manual_mode():
    c = make_geometric()
    c.plan_route((3.0, 1.5), "Goal")
    c.start()
    with pytest.raises(ValueError):
        c.clear_route()
    c.pause()
    c.clear_route()
    assert c.navigator is None


def test_navigation_does_not_bypass_transition_table():
    # Arrival uses MOVING -> STOPPED, which already exists; no new transitions were added.
    from src.controller import ALLOWED_TRANSITIONS
    assert S.STOPPED in ALLOWED_TRANSITIONS[S.MOVING]
    assert ALLOWED_TRANSITIONS[S.STOPPED] == frozenset({S.IDLE, S.EMERGENCY_STOP, S.FAULT})


def test_custom_config_navigation():
    cfg = SimulationConfig(navigation=NavigationConfig(resolution_m=0.1, safety_margin_m=0.08))
    c = make_geometric(config=cfg, pose=Pose(0.6, 1.5, 0.0))
    assert c.plan_route((3.0, 1.0), "Coarse").ok
    c.start()
    drive(c)
    assert "Destination reached" in c.state_reason
