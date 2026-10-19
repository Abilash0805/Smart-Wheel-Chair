"""Configuration for the MobilityBot simulation.

Units: metres, seconds, and degrees (degrees only at the configuration
boundary; the simulator works in radians internally).

IMPORTANT: every default below is a *simulation* value. None of them has been
measured on the real robot. Values marked "PLACEHOLDER" must be replaced with
measurements before they mean anything for physical hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class SensorConfig:
    """Simulated HC-SR04-style ultrasonic sensor parameters."""

    # Commonly published nominal HC-SR04 range is about 2 cm to 400 cm.
    # NOT verified for your module; real usable range is often shorter.
    min_range_m: float = 0.02
    max_range_m: float = 4.00
    # Commonly quoted effective beam angle (~15 degrees). The real beam
    # pattern depends on the module, target shape and surface material.
    beam_width_deg: float = 15.0
    # The beam is approximated by this many straight rays spread evenly
    # across the beam width. The reading is the nearest ray hit.
    ray_count: int = 5
    # Gaussian noise standard deviation. 0 = deterministic (default for tests).
    noise_std_m: float = 0.0
    noise_seed: int = 0

    def __post_init__(self) -> None:
        _require_finite(self.min_range_m, "min_range_m")
        _require_finite(self.max_range_m, "max_range_m")
        _require_finite(self.beam_width_deg, "beam_width_deg")
        _require_finite(self.noise_std_m, "noise_std_m")
        if self.min_range_m < 0:
            raise ValueError("min_range_m must be >= 0")
        if self.max_range_m <= self.min_range_m:
            raise ValueError("max_range_m must be greater than min_range_m")
        if not 0 <= self.beam_width_deg < 180:
            raise ValueError("beam_width_deg must be in [0, 180)")
        if self.ray_count < 1:
            raise ValueError("ray_count must be at least 1")
        if self.noise_std_m < 0:
            raise ValueError("noise_std_m must be >= 0")


@dataclass(frozen=True)
class RobotConfig:
    """Virtual robot body. The robot is modelled as a circle."""

    # PLACEHOLDER: measure your chassis. The sensor is assumed to sit on the
    # front edge of this circle, facing along the robot's heading.
    radius_m: float = 0.15

    def __post_init__(self) -> None:
        _require_finite(self.radius_m, "radius_m")
        if self.radius_m <= 0:
            raise ValueError("radius_m must be > 0")


@dataclass(frozen=True)
class MotionConfig:
    """Simulated motion parameters."""

    speed_mps: float = 0.20
    turn_rate_dps: float = 0.0
    # One simulation step ("tick") advances simulated time by this much.
    time_step_s: float = 0.10
    # Upper limit accepted by the simulator. This is a simulation guard,
    # not a measured capability of the motors.
    max_speed_mps: float = 0.50
    max_turn_rate_dps: float = 90.0

    def __post_init__(self) -> None:
        for name in ("speed_mps", "turn_rate_dps", "time_step_s", "max_speed_mps", "max_turn_rate_dps"):
            _require_finite(getattr(self, name), name)
        if self.time_step_s <= 0:
            raise ValueError("time_step_s must be > 0")
        if not 0 <= self.speed_mps <= self.max_speed_mps:
            raise ValueError(f"speed_mps must be between 0 and {self.max_speed_mps}")
        if abs(self.turn_rate_dps) > self.max_turn_rate_dps:
            raise ValueError(f"turn_rate_dps must be within +/-{self.max_turn_rate_dps}")


@dataclass(frozen=True)
class SafetyConfig:
    """Simulated safety policy.

    The stop threshold is a conservative simulation setting. It has NOT been
    validated against a real stopping distance, sensor latency or braking.
    """

    stop_threshold_m: float = 0.50
    # Number of consecutive valid sensor readings (simulation steps) required
    # after a fault before Reset is permitted.
    fault_recovery_readings: int = 3

    def __post_init__(self) -> None:
        _require_finite(self.stop_threshold_m, "stop_threshold_m")
        if self.stop_threshold_m <= 0:
            raise ValueError("stop_threshold_m must be > 0")
        if self.fault_recovery_readings < 1:
            raise ValueError("fault_recovery_readings must be at least 1")


@dataclass(frozen=True)
class SimulationConfig:
    """All configuration in one place, with cross-field validation."""

    sensor: SensorConfig = field(default_factory=SensorConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    def __post_init__(self) -> None:
        validate_threshold(self.safety.stop_threshold_m, self.sensor)


def validate_threshold(threshold_m: float, sensor: SensorConfig) -> None:
    """A stop threshold is only meaningful inside the sensor's valid range."""
    _require_finite(threshold_m, "stop threshold")
    if not sensor.min_range_m < threshold_m < sensor.max_range_m:
        raise ValueError(
            f"Stop threshold {threshold_m:.2f} m must be inside the sensor's valid range "
            f"({sensor.min_range_m:.2f} m to {sensor.max_range_m:.2f} m)"
        )


def _require_finite(value: float, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
