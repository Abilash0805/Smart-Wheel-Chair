import math

import pytest

from src.config import SafetyConfig, SensorConfig, SimulationConfig, validate_threshold
from src.models import SensorReading, Verdict
from src.safety import assess_reading

SENSOR = SensorConfig()  # valid range 0.02 m to 4.00 m
THRESHOLD = 0.50


def assess(value):
    return assess_reading(SensorReading(value, "TEST", "test", 0.0), THRESHOLD, SENSOR)


def test_reading_exactly_at_threshold_is_obstacle():
    assert assess(0.50).verdict is Verdict.OBSTACLE


def test_reading_inside_threshold_is_obstacle():
    assert assess(0.49).verdict is Verdict.OBSTACLE
    assert assess(0.02).verdict is Verdict.OBSTACLE  # minimum valid range


def test_reading_just_beyond_threshold_is_clear():
    a = assess(0.5000001)
    assert a.verdict is Verdict.CLEAR
    assert a.distance_m == pytest.approx(0.5000001)


def test_maximum_valid_range_is_clear():
    assert assess(4.0).verdict is Verdict.CLEAR


def test_integer_reading_is_accepted():
    assert assess(2).verdict is Verdict.CLEAR


@pytest.mark.parametrize(
    "value",
    [None, math.nan, math.inf, -math.inf, -1.0, -0.0001, 0.0, 0.0199, 4.0001, 12.0, "1.5", True, False, [1.0], object()],
)
def test_invalid_values_are_never_clear(value):
    a = assess(value)
    assert a.verdict is Verdict.INVALID
    assert a.distance_m is None
    assert a.reason


def test_missing_reading_object_is_invalid():
    assert assess_reading(None, THRESHOLD, SENSOR).verdict is Verdict.INVALID


@pytest.mark.parametrize("threshold", [0.0, 0.02, 4.0, 5.0, -1.0, math.nan, math.inf])
def test_threshold_must_be_inside_sensor_range(threshold):
    with pytest.raises(ValueError):
        validate_threshold(threshold, SENSOR)


def test_simulation_config_rejects_bad_threshold():
    with pytest.raises(ValueError):
        SimulationConfig(safety=SafetyConfig(stop_threshold_m=5.0))


def test_configs_reject_bad_values():
    with pytest.raises(ValueError):
        SensorConfig(min_range_m=1.0, max_range_m=0.5)
    with pytest.raises(ValueError):
        SensorConfig(noise_std_m=-0.1)
    with pytest.raises(ValueError):
        SafetyConfig(fault_recovery_readings=0)
