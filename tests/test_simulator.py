import math
import random

import pytest

from src.config import RobotConfig, SensorConfig
from src.models import Obstacle, Pose, SensorFaultMode
from src.sensors.simulated_ultrasonic import SimulatedUltrasonicSensor
from src.simulator import (
    Simulator,
    World,
    circle_collision,
    integrate_pose,
    normalize_angle,
    ray_distance,
    ray_rect_distance,
)

ROOM = World(4.0, 3.0, ())
BOX = Obstacle("Box", 2.0, 1.0, 0.5, 1.0)


# ------------------------------------------------------------- motion

def test_straight_motion_east():
    p = integrate_pose(Pose(1.0, 1.0, 0.0), 0.2, 0.0, 0.1)
    assert p.x == pytest.approx(1.02)
    assert p.y == pytest.approx(1.0)
    assert p.heading_rad == pytest.approx(0.0)


def test_straight_motion_north():
    p = integrate_pose(Pose(1.0, 1.0, math.pi / 2), 0.5, 0.0, 1.0)
    assert p.x == pytest.approx(1.0)
    assert p.y == pytest.approx(1.5)


def test_heading_update_and_wraparound():
    p = integrate_pose(Pose(1.0, 1.0, math.radians(170)), 0.0, math.radians(20), 1.0)
    assert p.heading_rad == pytest.approx(math.radians(-170))
    assert (p.x, p.y) == (1.0, 1.0)


@pytest.mark.parametrize(
    "angle, expected",
    [(0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi), (2 * math.pi, 0.0),
     (-math.pi / 2, -math.pi / 2), (7.0, 7.0 - 2 * math.pi)],
)
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


def test_movement_is_deterministic():
    def run():
        pose = Pose(0.5, 0.5, 0.3)
        for _ in range(500):
            pose = integrate_pose(pose, 0.2, 0.1, 0.1)
        return pose

    assert run() == run()


def test_simulator_tracks_distance_and_time():
    sim = Simulator(ROOM, RobotConfig(), Pose(1.0, 1.0, 0.0))
    sim.move_to(sim.predict(0.2, 0.0, 0.5))
    sim.advance_time(0.5)
    assert sim.distance_travelled_m == pytest.approx(0.1)
    assert sim.sim_time_s == pytest.approx(0.5)
    assert sim.step_count == 1
    assert sim.trail[-1] == pytest.approx((1.1, 1.0))


# ---------------------------------------------------------- geometry

@pytest.mark.parametrize(
    "angle, expected",
    [(0.0, 3.0), (math.pi / 2, 2.0), (math.pi, 1.0), (-math.pi / 2, 1.0)],
)
def test_ray_to_walls(angle, expected):
    assert ray_distance(ROOM, 1.0, 1.0, angle) == pytest.approx(expected)


def test_ray_hits_obstacle_before_wall():
    world = World(4.0, 3.0, (BOX,))
    assert ray_distance(world, 1.0, 1.5, 0.0) == pytest.approx(1.0)


def test_ray_misses_obstacle_to_the_side_and_behind():
    world = World(4.0, 3.0, (BOX,))
    assert ray_distance(world, 1.0, 2.5, 0.0) == pytest.approx(3.0)  # passes above the box
    assert ray_distance(world, 3.0, 1.5, 0.0) == pytest.approx(1.0)  # box is behind
    assert ray_rect_distance(3.0, 1.5, 1.0, 0.0, BOX) is None


def test_ray_starting_inside_obstacle_is_zero():
    assert ray_rect_distance(2.2, 1.5, 1.0, 0.0, BOX) == 0.0


def test_diagonal_ray_distance():
    # From (1,1) at 45 degrees, the ray reaches the north wall (y=3) after 2*sqrt(2).
    assert ray_distance(ROOM, 1.0, 1.0, math.pi / 4) == pytest.approx(2 * math.sqrt(2))


def test_circle_collision_with_walls():
    assert circle_collision(ROOM, 0.1, 1.5, 0.15) == "west wall"
    assert circle_collision(ROOM, 3.9, 1.5, 0.15) == "east wall"
    assert circle_collision(ROOM, 2.0, 0.1, 0.15) == "south wall"
    assert circle_collision(ROOM, 2.0, 2.9, 0.15) == "north wall"
    assert circle_collision(ROOM, 0.15, 1.5, 0.15) is None  # exactly touching is allowed


def test_circle_collision_with_obstacle():
    world = World(4.0, 3.0, (BOX,))
    assert circle_collision(world, 1.9, 1.5, 0.15) == "Box"
    assert circle_collision(world, 1.85, 1.5, 0.15) is None  # touching
    assert circle_collision(world, 1.5, 1.5, 0.15) is None


def test_simulator_refuses_to_move_into_wall():
    sim = Simulator(ROOM, RobotConfig(), Pose(3.8, 1.5, 0.0))
    with pytest.raises(ValueError, match="east wall"):
        sim.move_to(Pose(3.9, 1.5, 0.0))
    assert sim.pose.x == 3.8


def test_start_pose_must_not_overlap():
    with pytest.raises(ValueError):
        Simulator(ROOM, RobotConfig(), Pose(0.05, 1.5, 0.0))


@pytest.mark.parametrize(
    "obstacle",
    [Obstacle("Out", 3.8, 1.0, 0.5, 0.5), Obstacle("Neg", -0.1, 1.0, 0.5, 0.5),
     Obstacle("Flat", 1.0, 1.0, 0.0, 0.5), Obstacle("NaN", math.nan, 1.0, 0.5, 0.5), Obstacle(" ", 1, 1, 1, 1)],
)
def test_world_rejects_invalid_obstacles(obstacle):
    with pytest.raises(ValueError):
        World(4.0, 3.0, (obstacle,))


def test_world_rejects_duplicate_names():
    with pytest.raises(ValueError):
        World(4.0, 3.0, (BOX, BOX))


# ------------------------------------------------------ simulated sensor

def make_sensor(world=ROOM, pose=Pose(1.0, 1.5, 0.0), **cfg):
    sim = Simulator(world, RobotConfig(radius_m=0.15), pose)
    return sim, SimulatedUltrasonicSensor(sim, SensorConfig(**cfg))


def test_sensor_reads_geometry_from_front_of_robot():
    _, sensor = make_sensor(World(4.0, 3.0, (BOX,)))
    reading = sensor.read()
    assert reading.source == "SIMULATED"
    assert reading.value_m == pytest.approx(2.0 - 1.15)


def test_sensor_beam_sees_obstacle_slightly_off_centre():
    post = Obstacle("Post", 2.0, 1.57, 0.05, 0.06)  # above the centre ray, inside the 15 degree beam
    _, narrow = make_sensor(World(4.0, 3.0, (post,)), ray_count=1)
    _, wide = make_sensor(World(4.0, 3.0, (post,)), ray_count=5)
    assert narrow.read().value_m == pytest.approx(3.0 - 0.15)  # centre ray misses: sees wall
    assert wide.read().value_m < 1.0


def test_sensor_reports_no_echo_beyond_max_range():
    _, sensor = make_sensor(pose=Pose(0.5, 1.5, 0.0), max_range_m=2.0)
    assert sensor.read().value_m is None


def test_sensor_noise_is_seeded_and_repeatable():
    _, a = make_sensor(noise_std_m=0.01)
    _, b = make_sensor(noise_std_m=0.01)
    values_a = [a.read().value_m for _ in range(5)]
    assert values_a == [b.read().value_m for _ in range(5)]
    assert len(set(values_a)) > 1


def test_sensor_noise_default_is_zero():
    _, sensor = make_sensor()
    assert sensor.read().value_m == sensor.read().value_m == pytest.approx(2.85)


def test_injected_rng():
    sim = Simulator(ROOM, RobotConfig(), Pose(1.0, 1.5, 0.0))
    sensor = SimulatedUltrasonicSensor(sim, SensorConfig(noise_std_m=0.01), rng=random.Random(42))
    expected = 2.85 + random.Random(42).gauss(0.0, 0.01)
    assert sensor.read().value_m == pytest.approx(expected)


@pytest.mark.parametrize(
    "mode, check",
    [
        (SensorFaultMode.NO_ECHO, lambda v: v is None),
        (SensorFaultMode.NOT_A_NUMBER, lambda v: math.isnan(v)),
        (SensorFaultMode.INFINITE, lambda v: math.isinf(v)),
        (SensorFaultMode.NEGATIVE, lambda v: v < 0),
        (SensorFaultMode.ABOVE_MAX_RANGE, lambda v: v > 4.0),
        (SensorFaultMode.BELOW_MIN_RANGE, lambda v: v < 0.02),
    ],
)
def test_fault_injection_values(mode, check):
    _, sensor = make_sensor()
    sensor.set_fault_mode(mode)
    assert check(sensor.read().value_m)
    sensor.set_fault_mode(SensorFaultMode.NONE)
    assert sensor.read().value_m == pytest.approx(2.85)
