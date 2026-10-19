"""Event log, scenarios, visualisation and the no-hardware guard."""

import ast
import csv
import io
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import pytest

from src.event_log import EventLog, Level
from src.models import RobotState
from src.scenarios import SCENARIOS, build_controller
from src.visualization import render_room_svg
from tests.helpers import fixed_clock

ROOT = Path(__file__).resolve().parent.parent
HARDWARE_MODULES = {"RPi", "gpiozero", "pigpio", "smbus", "smbus2", "spidev", "serial", "pyfirmata",
                    "board", "busio", "digitalio", "machine", "lgpio", "wiringpi"}


def test_event_log_sequence_clock_and_csv():
    log = EventLog(clock=fixed_clock, max_entries=3)
    for i in range(5):
        log.add(Level.INFO, "TEST", f"event {i}", i * 0.1)
    assert [e.seq for e in log.entries] == [3, 4, 5]  # bounded
    assert log.latest(1)[0].message == "event 4"
    assert log.entries[0].wall_time == datetime(2026, 1, 1, 12, 0, 0, tzinfo=fixed_clock().tzinfo)
    rows = list(csv.reader(io.StringIO(log.to_csv())))
    assert rows[0] == ["seq", "wall_time", "sim_time_s", "level", "category", "message"]
    assert rows[1][5] == "event 2"


@pytest.mark.parametrize("key", list(SCENARIOS))
def test_scenarios_build_idle(key):
    c = build_controller(key, clock=fixed_clock)
    assert c.state is RobotState.IDLE
    assert c.last_assessment is not None


@pytest.mark.parametrize(
    "key, final_state",
    [("obstacle_ahead", RobotState.OBSTACLE_DETECTED), ("wall_approach", RobotState.OBSTACLE_DETECTED),
     ("blind_spot", RobotState.FAULT)],
)
def test_scenario_outcomes(key, final_state):
    c = build_controller(key, clock=fixed_clock)
    assert c.start().accepted
    for _ in range(1000):
        if c.state is not RobotState.MOVING:
            break
        c.tick()
    assert c.state is final_state


def test_svg_is_well_formed_and_labelled():
    c = build_controller("obstacle_ahead", clock=fixed_clock)
    svg = render_room_svg(c)
    root = ET.fromstring(svg)
    assert root.get("role") == "img"
    assert "SIMULATED" in svg and "Box A" in svg and "IDLE" in svg
    c.emergency_stop()
    assert svg.count("EMERGENCY STOP") == 0
    assert render_room_svg(c).count("EMERGENCY STOP") >= 2  # header and in-map banner


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            names.add(node.module.split(".")[0])
    return names


def test_no_hardware_libraries_are_imported():
    files = [ROOT / "app.py", *ROOT.joinpath("src").rglob("*.py")]
    for path in files:
        bad = _imports(path) & HARDWARE_MODULES
        assert not bad, f"{path.name} imports hardware modules {bad}"


def test_core_does_not_depend_on_streamlit():
    for path in ROOT.joinpath("src").rglob("*.py"):
        assert "streamlit" not in _imports(path), path.name


@pytest.mark.parametrize("key", ["obstacle_ahead", "wall_approach"])
@pytest.mark.parametrize("turn_dps", [-45.0, -20.0, 10.0, 30.0])
def test_turning_runs_always_stop_without_collision(key, turn_dps):
    c = build_controller(key, clock=fixed_clock)
    c.set_motion(0.2, turn_dps)
    assert c.start().accepted
    for _ in range(3000):
        if c.state is not RobotState.MOVING:
            break
        c.tick()
        assert c.sim.collision_at(c.sim.pose) is None
    assert c.state in (RobotState.OBSTACLE_DETECTED, RobotState.FAULT)
