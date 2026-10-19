"""Predefined, resettable demonstration scenarios."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime

from src.config import SimulationConfig
from src.controller import RobotController
from src.event_log import EventLog, Level
from src.models import Obstacle, Pose
from src.sensors.simulated_ultrasonic import SimulatedUltrasonicSensor
from src.simulator import Simulator, World

ROOM_WIDTH_M = 4.0
ROOM_HEIGHT_M = 3.0


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    description: str
    world: World
    start_pose: Pose


SCENARIOS: dict[str, Scenario] = {
    s.key: s
    for s in (
        Scenario(
            key="obstacle_ahead",
            title="1. Obstacle ahead",
            description=(
                "The robot drives toward a box. It should stop when the box is at or inside "
                "the stop threshold, and stay stopped until you press Resume."
            ),
            world=World(
                ROOM_WIDTH_M,
                ROOM_HEIGHT_M,
                (
                    Obstacle("Box A", 2.6, 1.2, 0.4, 0.6),
                    Obstacle("Shelf", 1.2, 2.4, 1.0, 0.4),
                ),
            ),
            start_pose=Pose(0.6, 1.5, 0.0),
        ),
        Scenario(
            key="wall_approach",
            title="2. Drive to the wall",
            description="No obstacles. The robot drives toward the east wall and stops at the threshold.",
            world=World(ROOM_WIDTH_M, ROOM_HEIGHT_M, ()),
            start_pose=Pose(0.6, 1.5, 0.0),
        ),
        Scenario(
            key="blind_spot",
            title="3. Sensor blind spot (limitation demo)",
            description=(
                "A thin post sits slightly to the side of the robot's path. One forward-facing "
                "ultrasonic beam does not see it, so the simulator blocks the collision and "
                "raises a FAULT. This shows why one sensor is not enough for real-world safety."
            ),
            world=World(ROOM_WIDTH_M, ROOM_HEIGHT_M, (Obstacle("Post", 2.2, 1.25, 0.08, 0.12),)),
            start_pose=Pose(0.6, 1.5, 0.0),
        ),
    )
}

DEFAULT_SCENARIO = "obstacle_ahead"

# Predefined destinations in the simulated room (metres). The labels are just
# names for demo positions; they are reachable in every scenario above
# (checked by tests/test_navigation.py).
DESTINATIONS: dict[str, tuple[float, float]] = {
    "Kitchen": (3.3, 2.3),
    "Living Room": (3.3, 0.7),
    "Bedroom": (1.0, 0.7),
    "Expo Goal": (2.0, 2.0),
}


def build_controller(
    scenario_key: str = DEFAULT_SCENARIO,
    config: SimulationConfig | None = None,
    clock: Callable[[], datetime] | None = None,
) -> RobotController:
    """Create a fresh simulation for a scenario. The robot always starts IDLE."""
    scenario = SCENARIOS[scenario_key]
    config = config or SimulationConfig()
    simulator = Simulator(scenario.world, config.robot, scenario.start_pose)
    sensor = SimulatedUltrasonicSensor(simulator, config.sensor)
    log = EventLog(clock=clock)
    controller = RobotController(simulator, sensor, config, log)
    log.add(
        Level.INFO, "SCENARIO",
        f"Loaded scenario '{scenario.title}'. Start pose ({scenario.start_pose.x:.2f}, "
        f"{scenario.start_pose.y:.2f}) heading {math.degrees(scenario.start_pose.heading_rad):.0f} deg",
        simulator.sim_time_s,
    )
    controller.observe()
    return controller
