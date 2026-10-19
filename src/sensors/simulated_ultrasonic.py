"""Simulated HC-SR04-style ultrasonic distance sensor.

The distance comes from ray casting in the virtual room: several rays are
spread across the beam width and the nearest hit is reported, roughly like
the first echo of a real ultrasonic pulse. This is a simplified model; real
ultrasonic sensors also miss soft, thin or angled surfaces.
"""

from __future__ import annotations

import math
import random
from dataclasses import replace

from src.config import SensorConfig
from src.models import SensorFaultMode, SensorReading
from src.sensors.base import DistanceSensor
from src.simulator import Simulator, ray_distance

# Raw values produced by each injected fault.
FAULT_VALUES: dict[SensorFaultMode, float | None] = {
    SensorFaultMode.NO_ECHO: None,
    SensorFaultMode.NOT_A_NUMBER: math.nan,
    SensorFaultMode.INFINITE: math.inf,
    SensorFaultMode.NEGATIVE: -1.0,
    SensorFaultMode.ABOVE_MAX_RANGE: 12.0,
    SensorFaultMode.BELOW_MIN_RANGE: 0.0,
}


class SimulatedUltrasonicSensor(DistanceSensor):
    source = "SIMULATED"
    name = "Simulated ultrasonic (HC-SR04 style)"

    def __init__(self, simulator: Simulator, config: SensorConfig, rng: random.Random | None = None) -> None:
        self.simulator = simulator
        self.config = config
        self.rng = rng if rng is not None else random.Random(config.noise_seed)
        self.fault_mode = SensorFaultMode.NONE

    def set_fault_mode(self, mode: SensorFaultMode) -> None:
        self.fault_mode = SensorFaultMode(mode)

    def set_noise_std(self, noise_std_m: float) -> None:
        # SensorConfig validates the new value.
        self.config = replace(self.config, noise_std_m=float(noise_std_m))

    def ray_angles(self) -> list[float]:
        heading = self.simulator.pose.heading_rad
        n = self.config.ray_count
        if n == 1:
            return [heading]
        half = math.radians(self.config.beam_width_deg) / 2
        return [heading - half + i * (2 * half) / (n - 1) for i in range(n)]

    def true_distance_m(self) -> float:
        """Geometric distance to the nearest surface inside the beam (no noise, no faults)."""
        ox, oy = self.simulator.sensor_origin()
        return min(ray_distance(self.simulator.world, ox, oy, angle) for angle in self.ray_angles())

    def read(self) -> SensorReading:
        if self.fault_mode is not SensorFaultMode.NONE:
            value = FAULT_VALUES[self.fault_mode]
        else:
            value = self.true_distance_m()
            if self.config.noise_std_m > 0:
                value += self.rng.gauss(0.0, self.config.noise_std_m)
            if value > self.config.max_range_m:
                # Nothing within range: a real HC-SR04 driver typically times
                # out and reports no echo, so the simulation does the same.
                value = None
        return SensorReading(
            value_m=value,
            source=self.source,
            sensor_name=self.name,
            sim_time_s=self.simulator.sim_time_s,
        )
