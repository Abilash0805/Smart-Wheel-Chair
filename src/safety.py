"""Safety rules: decide whether a sensor reading permits motion.

Policy (simple stop-on-obstacle rule, simulation only):

* Only a reading that is a real, finite number inside the sensor's valid
  range AND strictly greater than the stop threshold is CLEAR.
* A valid reading at or inside the threshold is an OBSTACLE.
* Everything else (missing, NaN, infinity, wrong type, out of range) is
  INVALID. An invalid reading is never treated as a clear path.
"""

from __future__ import annotations

import math

from src.config import SensorConfig
from src.models import SafetyAssessment, SensorReading, Verdict


def assess_reading(reading: SensorReading | None, threshold_m: float, sensor: SensorConfig) -> SafetyAssessment:
    if reading is None:
        return SafetyAssessment(Verdict.INVALID, None, "No sensor reading available")

    value = reading.value_m
    if value is None:
        return SafetyAssessment(Verdict.INVALID, None, "Missing reading (no echo received)")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return SafetyAssessment(Verdict.INVALID, None, f"Reading is not a number ({value!r})")

    value = float(value)
    if not math.isfinite(value):
        return SafetyAssessment(Verdict.INVALID, None, f"Non-finite reading ({value})")
    if value < sensor.min_range_m:
        return SafetyAssessment(
            Verdict.INVALID, None,
            f"Reading {value:.3f} m is below the minimum valid range ({sensor.min_range_m:.2f} m); "
            "an obstacle may be very close or the sensor is faulty",
        )
    if value > sensor.max_range_m:
        return SafetyAssessment(
            Verdict.INVALID, None,
            f"Reading {value:.3f} m is above the maximum valid range ({sensor.max_range_m:.2f} m)",
        )
    if value <= threshold_m:
        return SafetyAssessment(
            Verdict.OBSTACLE, value,
            f"Obstacle at {value:.3f} m (stop threshold {threshold_m:.2f} m)",
        )
    return SafetyAssessment(Verdict.CLEAR, value, f"Path clear: {value:.3f} m > {threshold_m:.2f} m")
