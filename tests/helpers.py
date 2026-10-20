"""Shared test helpers and fakes. Tests never touch real hardware."""

from __future__ import annotations

from collections.abc import Iterable
from datetime import datetime, timezone

from src.config import SimulationConfig
from src.controller import RobotController
from src.event_log import EventLog
from src.models import Obstacle, Pose, SensorReading
from src.sensors.base import DistanceSensor
from src.sensors.simulated_ultrasonic import SimulatedUltrasonicSensor
from src.simulator import Simulator, World


def fixed_clock() -> datetime:
    return datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc)


class ScriptedSensor(DistanceSensor):
    """Fake sensor returning a fixed sequence of raw values, then repeating the last one."""

    source = "TEST"
    name = "scripted test sensor"

    def __init__(self, values: Iterable[object]) -> None:
        self.values = list(values)
        self.reads = 0

    def set(self, *values: object) -> None:
        self.values = list(values)

    def read(self) -> SensorReading:
        index = min(self.reads, len(self.values) - 1)
        self.reads += 1
        return SensorReading(self.values[index], self.source, self.name, 0.0)  # type: ignore[arg-type]


EMPTY_ROOM = World(4.0, 3.0, ())
START = Pose(0.6, 1.5, 0.0)


def make_scripted(values: Iterable[object] = (2.0,), config: SimulationConfig | None = None,
                  world: World = EMPTY_ROOM, pose: Pose = START) -> tuple[RobotController, ScriptedSensor]:
    config = config or SimulationConfig()
    sim = Simulator(world, config.robot, pose)
    sensor = ScriptedSensor(values)
    return RobotController(sim, sensor, config, EventLog(clock=fixed_clock)), sensor


def make_geometric(obstacles: tuple[Obstacle, ...] = (), config: SimulationConfig | None = None,
                   pose: Pose = START) -> RobotController:
    config = config or SimulationConfig()
    sim = Simulator(World(4.0, 3.0, obstacles), config.robot, pose)
    sensor = SimulatedUltrasonicSensor(sim, config.sensor)
    controller = RobotController(sim, sensor, config, EventLog(clock=fixed_clock))
    controller.observe()
    return controller


class FakeSerial:
    """Hands out queued chunks. Writing is a test failure."""

    def __init__(self, *chunks: bytes) -> None:
        self.chunks = list(chunks)
        self.closed = False
        self.fail_with: Exception | None = None

    def read_available(self) -> bytes:
        if self.fail_with:
            raise self.fail_with
        return self.chunks.pop(0) if self.chunks else b""

    def push(self, *chunks: bytes) -> None:
        self.chunks.extend(chunks)

    def close(self) -> None:
        self.closed = True

    def write(self, data) -> None:  # pragma: no cover - must never run
        raise AssertionError("telemetry must never write to the device")
