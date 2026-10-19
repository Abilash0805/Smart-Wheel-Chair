"""Deterministic 2-D top-down robot simulator.

The room is a rectangle from (0, 0) to (width, height). The robot is a circle
that moves like a differential-drive (unicycle) robot. Time only advances when
the caller asks for a step, so runs are fully repeatable.

This module must never import or call real hardware drivers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from src.config import RobotConfig
from src.models import Obstacle, Pose

# Distances smaller than this are treated as zero to absorb float rounding.
EPSILON = 1e-9


@dataclass(frozen=True)
class World:
    width_m: float
    height_m: float
    obstacles: tuple[Obstacle, ...] = ()

    def __post_init__(self) -> None:
        if not (math.isfinite(self.width_m) and math.isfinite(self.height_m)):
            raise ValueError("Room size must be finite")
        if self.width_m <= 0 or self.height_m <= 0:
            raise ValueError("Room size must be positive")
        names = [o.name for o in self.obstacles]
        if len(set(names)) != len(names):
            raise ValueError("Obstacle names must be unique")
        for obstacle in self.obstacles:
            validate_obstacle(obstacle, self.width_m, self.height_m)

    def with_obstacles(self, obstacles: tuple[Obstacle, ...]) -> World:
        return replace(self, obstacles=tuple(obstacles))


def validate_obstacle(obstacle: Obstacle, room_width: float, room_height: float) -> None:
    values = (obstacle.x, obstacle.y, obstacle.width, obstacle.height)
    if not obstacle.name.strip():
        raise ValueError("Every obstacle needs a name")
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError(f"{obstacle.name}: position and size must be finite numbers")
    if obstacle.width <= 0 or obstacle.height <= 0:
        raise ValueError(f"{obstacle.name}: width and height must be positive")
    if obstacle.x < 0 or obstacle.y < 0 or obstacle.x_max > room_width or obstacle.y_max > room_height:
        raise ValueError(
            f"{obstacle.name}: must lie completely inside the room "
            f"(0-{room_width:g} m by 0-{room_height:g} m)"
        )


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle to the interval (-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2 * math.pi)
    if wrapped <= 0:
        wrapped += 2 * math.pi
    return wrapped - math.pi


def integrate_pose(pose: Pose, linear_mps: float, angular_rps: float, dt_s: float) -> Pose:
    """Advance a pose by one time step (forward Euler).

    Position uses the heading at the start of the step, then the heading is
    updated. Simple and exactly repeatable, which is what the tests need.
    """
    x = pose.x + linear_mps * math.cos(pose.heading_rad) * dt_s
    y = pose.y + linear_mps * math.sin(pose.heading_rad) * dt_s
    heading = normalize_angle(pose.heading_rad + angular_rps * dt_s)
    return Pose(x, y, heading)


def ray_rect_distance(ox: float, oy: float, dx: float, dy: float, rect: Obstacle) -> float | None:
    """Distance along a unit ray to an axis-aligned rectangle (slab method).

    Returns 0.0 if the ray starts inside the rectangle, None if it misses.
    """
    t_min, t_max = -math.inf, math.inf
    for origin, direction, low, high in ((ox, dx, rect.x, rect.x_max), (oy, dy, rect.y, rect.y_max)):
        if abs(direction) < EPSILON:
            if origin < low or origin > high:
                return None
            continue
        t1 = (low - origin) / direction
        t2 = (high - origin) / direction
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return None
    if t_max < 0:
        return None  # rectangle is behind the ray
    return max(t_min, 0.0)


def ray_wall_distance(ox: float, oy: float, dx: float, dy: float, width: float, height: float) -> float:
    """Distance along a unit ray from a point inside the room to the nearest wall."""
    candidates = []
    if dx > EPSILON:
        candidates.append((width - ox) / dx)
    elif dx < -EPSILON:
        candidates.append(-ox / dx)
    if dy > EPSILON:
        candidates.append((height - oy) / dy)
    elif dy < -EPSILON:
        candidates.append(-oy / dy)
    return max(0.0, min(candidates))


def ray_distance(world: World, ox: float, oy: float, angle_rad: float) -> float:
    """Distance from (ox, oy) along angle_rad to the first wall or obstacle."""
    dx, dy = math.cos(angle_rad), math.sin(angle_rad)
    nearest = ray_wall_distance(ox, oy, dx, dy, world.width_m, world.height_m)
    for obstacle in world.obstacles:
        hit = ray_rect_distance(ox, oy, dx, dy, obstacle)
        if hit is not None and hit < nearest:
            nearest = hit
    return nearest


def circle_collision(world: World, x: float, y: float, radius: float) -> str | None:
    """Name of the wall or obstacle the circle would overlap, else None.

    Exactly touching (distance == radius) is not treated as overlapping.
    """
    if x - radius < -EPSILON:
        return "west wall"
    if x + radius > world.width_m + EPSILON:
        return "east wall"
    if y - radius < -EPSILON:
        return "south wall"
    if y + radius > world.height_m + EPSILON:
        return "north wall"
    for obstacle in world.obstacles:
        nearest_x = min(max(x, obstacle.x), obstacle.x_max)
        nearest_y = min(max(y, obstacle.y), obstacle.y_max)
        if math.hypot(x - nearest_x, y - nearest_y) < radius - EPSILON:
            return obstacle.name
    return None


class Simulator:
    """Holds the virtual robot's pose, the room, and the simulation clock."""

    def __init__(self, world: World, robot: RobotConfig, start_pose: Pose) -> None:
        self.robot = robot
        self.world = world
        hit = circle_collision(world, start_pose.x, start_pose.y, robot.radius_m)
        if hit is not None:
            raise ValueError(f"Start pose overlaps {hit}")
        self.pose = Pose(start_pose.x, start_pose.y, normalize_angle(start_pose.heading_rad))
        self.sim_time_s = 0.0
        self.step_count = 0
        self.distance_travelled_m = 0.0
        self.trail: list[tuple[float, float]] = [(self.pose.x, self.pose.y)]

    def sensor_origin(self, pose: Pose | None = None) -> tuple[float, float]:
        """The sensor sits on the front edge of the robot, facing the heading."""
        p = pose or self.pose
        return (
            p.x + self.robot.radius_m * math.cos(p.heading_rad),
            p.y + self.robot.radius_m * math.sin(p.heading_rad),
        )

    def predict(self, linear_mps: float, angular_rps: float, dt_s: float) -> Pose:
        return integrate_pose(self.pose, linear_mps, angular_rps, dt_s)

    def collision_at(self, pose: Pose) -> str | None:
        return circle_collision(self.world, pose.x, pose.y, self.robot.radius_m)

    def move_to(self, pose: Pose) -> None:
        hit = self.collision_at(pose)
        if hit is not None:
            raise ValueError(f"Refusing to move into {hit}")
        self.distance_travelled_m += math.hypot(pose.x - self.pose.x, pose.y - self.pose.y)
        self.pose = pose
        self.trail.append((pose.x, pose.y))
        if len(self.trail) > 2000:
            del self.trail[: len(self.trail) - 2000]

    def advance_time(self, dt_s: float) -> None:
        self.sim_time_s += dt_s
        self.step_count += 1

    def set_world(self, world: World) -> None:
        self.world = world
