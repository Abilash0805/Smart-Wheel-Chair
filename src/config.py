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
class NavigationConfig:
    """Grid path planning and route following (simulation only)."""

    # Size of one occupancy-grid cell. The room size must be a whole number of cells.
    resolution_m: float = 0.05
    # Extra clearance added to the robot radius when inflating obstacles and walls.
    # Must be at least half a cell diagonal (resolution * sqrt(2) / 2) so that the
    # straight segments between free cell centres keep the robot body clear.
    safety_margin_m: float = 0.05
    # A waypoint (and the goal) counts as reached within this distance.
    waypoint_tolerance_m: float = 0.04
    # If the heading error is larger than this, the robot turns on the spot first.
    rotate_in_place_deg: float = 20.0
    # Turn rate per radian of heading error (1/s).
    heading_gain: float = 2.0
    # Planner preference for keeping away from obstacles: cells with less than
    # preferred_clearance_m to the nearest surface cost up to (1 + clearance_weight)
    # times more. 0 disables it (pure shortest path).
    preferred_clearance_m: float = 0.5
    clearance_weight: float = 2.0

    def __post_init__(self) -> None:
        for name in ("resolution_m", "safety_margin_m", "waypoint_tolerance_m", "rotate_in_place_deg", "heading_gain",
                     "preferred_clearance_m", "clearance_weight"):
            _require_finite(getattr(self, name), name)
        if self.resolution_m <= 0:
            raise ValueError("resolution_m must be > 0")
        if self.safety_margin_m < self.resolution_m * math.sqrt(2) / 2:
            raise ValueError("safety_margin_m must be at least half a grid-cell diagonal")
        if self.waypoint_tolerance_m <= 0:
            raise ValueError("waypoint_tolerance_m must be > 0")
        if not 0 < self.rotate_in_place_deg < 180:
            raise ValueError("rotate_in_place_deg must be between 0 and 180")
        if self.heading_gain <= 0:
            raise ValueError("heading_gain must be > 0")
        if self.preferred_clearance_m <= 0 or self.clearance_weight < 0:
            raise ValueError("preferred_clearance_m must be > 0 and clearance_weight >= 0")


@dataclass(frozen=True)
class TelemetryConfig:
    """Read-only hardware telemetry (host side). No electrical values live here."""

    # Must match whatever the device firmware uses. A software setting, not a wiring fact.
    baud_rate: int = 115200
    # No valid message for this long -> data is STALE and is not shown as a reading.
    stale_after_s: float = 1.0
    # Reconnect back-off after the serial link drops: doubles from initial up to max.
    reconnect_initial_s: float = 0.5
    reconnect_max_s: float = 5.0
    # Longest accepted line (bytes, excluding the newline). Longer input is discarded.
    max_line_bytes: int = 512

    def __post_init__(self) -> None:
        if isinstance(self.baud_rate, bool) or not isinstance(self.baud_rate, int) or self.baud_rate <= 0:
            raise ValueError("baud_rate must be a positive integer")
        for name in ("stale_after_s", "reconnect_initial_s", "reconnect_max_s"):
            _require_finite(getattr(self, name), name)
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")
        if self.reconnect_max_s < self.reconnect_initial_s:
            raise ValueError("reconnect_max_s must be >= reconnect_initial_s")
        if self.max_line_bytes < 64:
            raise ValueError("max_line_bytes must be at least 64")


@dataclass(frozen=True)
class SimulationConfig:
    """All configuration in one place, with cross-field validation."""

    sensor: SensorConfig = field(default_factory=SensorConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)

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
