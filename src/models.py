"""Shared data types: states, poses, obstacles, readings and results."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum


class RobotState(StrEnum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    PAUSED = "PAUSED"
    OBSTACLE_DETECTED = "OBSTACLE_DETECTED"
    STOPPED = "STOPPED"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    FAULT = "FAULT"


class Command(StrEnum):
    START = "START"
    PAUSE = "PAUSE"
    RESUME = "RESUME"
    STOP = "STOP"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RESET = "RESET"


class Verdict(StrEnum):
    """Safety verdict for one sensor reading. Only CLEAR permits motion."""

    CLEAR = "CLEAR"
    OBSTACLE = "OBSTACLE"
    INVALID = "INVALID"


class FaultKind(StrEnum):
    SENSOR_INVALID = "SENSOR_INVALID"
    COLLISION_PREVENTED = "COLLISION_PREVENTED"


class SensorFaultMode(StrEnum):
    """Failures that can be injected into the simulated sensor for demos."""

    NONE = "NONE"
    NO_ECHO = "NO_ECHO"
    NOT_A_NUMBER = "NOT_A_NUMBER"
    INFINITE = "INFINITE"
    NEGATIVE = "NEGATIVE"
    ABOVE_MAX_RANGE = "ABOVE_MAX_RANGE"
    BELOW_MIN_RANGE = "BELOW_MIN_RANGE"


SENSOR_FAULT_DESCRIPTIONS: dict[SensorFaultMode, str] = {
    SensorFaultMode.NONE: "No fault - readings come from the virtual room geometry",
    SensorFaultMode.NO_ECHO: "Missing reading (no echo / disconnected sensor)",
    SensorFaultMode.NOT_A_NUMBER: "Corrupted value: NaN",
    SensorFaultMode.INFINITE: "Corrupted value: infinity",
    SensorFaultMode.NEGATIVE: "Impossible value: -1.0 m",
    SensorFaultMode.ABOVE_MAX_RANGE: "Out of range: 12.0 m (above maximum)",
    SensorFaultMode.BELOW_MIN_RANGE: "Out of range: 0.0 m (below minimum)",
}


@dataclass(frozen=True)
class Pose:
    """Robot position (metres) and heading (radians, 0 = +x/east, CCW positive)."""

    x: float
    y: float
    heading_rad: float

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.heading_rad)


@dataclass(frozen=True)
class Obstacle:
    """Axis-aligned rectangular obstacle. (x, y) is the lower-left corner."""

    name: str
    x: float
    y: float
    width: float
    height: float

    @property
    def x_max(self) -> float:
        return self.x + self.width

    @property
    def y_max(self) -> float:
        return self.y + self.height


@dataclass(frozen=True)
class SensorReading:
    """One raw distance measurement.

    ``value_m`` is whatever the sensor produced. It is deliberately NOT
    trusted here: a faulty driver may return None, NaN, infinity, negative
    or out-of-range numbers. The safety module decides whether it is valid.
    """

    value_m: float | None
    source: str
    sensor_name: str
    sim_time_s: float


@dataclass(frozen=True)
class SafetyAssessment:
    verdict: Verdict
    distance_m: float | None
    reason: str


@dataclass(frozen=True)
class CommandResult:
    command: Command
    accepted: bool
    message: str
    state: RobotState


@dataclass(frozen=True)
class FaultInfo:
    kind: FaultKind
    detail: str
    sim_time_s: float


@dataclass(frozen=True)
class StateInfo:
    symbol: str
    label: str
    explanation: str


STATE_INFO: dict[RobotState, StateInfo] = {
    RobotState.IDLE: StateInfo(
        "○", "IDLE - READY",
        "Robot is stationary and ready. Press Start to begin moving (the path must be clear).",
    ),
    RobotState.MOVING: StateInfo(
        "▶", "MOVING",
        "Robot is driving forward. It will stop automatically if the sensor reports an "
        "obstacle at or inside the stop threshold, or an invalid reading.",
    ),
    RobotState.PAUSED: StateInfo(
        "⏸", "PAUSED BY USER",
        "Motion halted by the user. Press Resume to continue (the path must be clear).",
    ),
    RobotState.OBSTACLE_DETECTED: StateInfo(
        "⚠", "OBSTACLE STOP (LATCHED)",
        "An obstacle was detected at or inside the stop threshold. The stop stays latched "
        "even if the obstacle moves away. Press Resume once the path is clear.",
    ),
    RobotState.STOPPED: StateInfo(
        "■", "STOPPED",
        "Run ended by the Stop command. Press Reset to return to IDLE before starting again.",
    ),
    RobotState.EMERGENCY_STOP: StateInfo(
        "⛔", "EMERGENCY STOP",
        "Software emergency stop is active. All movement commands are rejected until Reset. "
        "(This is a simulated control, not a substitute for a physical emergency stop.)",
    ),
    RobotState.FAULT: StateInfo(
        "✖", "FAULT",
        "A fault was detected. The robot will not move. Remove the cause, let the simulation "
        "record enough consecutive valid readings, then press Reset.",
    ),
}
