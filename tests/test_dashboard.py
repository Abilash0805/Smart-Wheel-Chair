"""Dashboard smoke tests using Streamlit's built-in AppTest (no browser, no hardware)."""

from pathlib import Path

import pytest

pytest.importorskip("streamlit")
from streamlit.testing.v1 import AppTest  # noqa: E402

from src.models import RobotState, SensorFaultMode  # noqa: E402

APP = Path(__file__).resolve().parent.parent / "app.py"


def button(at, label_start):
    return next(b for b in at.button if b.label.startswith(label_start))


@pytest.fixture
def app():
    at = AppTest.from_file(str(APP), default_timeout=30)
    at.run()
    assert not at.exception
    return at


def state(at):
    return at.session_state.controller.state


def test_dashboard_loads_in_simulation_mode(app):
    assert any("SIMULATION MODE" in i.value for i in app.info)
    assert state(app) is RobotState.IDLE


def test_reruns_do_not_start_motion(app):
    for _ in range(5):
        app.run()
    assert state(app) is RobotState.IDLE
    assert app.session_state.controller.sim.step_count == 0


def test_start_step_and_obstacle_latch(app):
    button(app, "▶ Start").click().run()
    assert state(app) is RobotState.MOVING
    for _ in range(8):
        button(app, "Step ×10").click().run()
    assert state(app) is RobotState.OBSTACLE_DETECTED
    pose = app.session_state.controller.sim.pose
    app.run()
    button(app, "Step ×10").click().run()
    assert state(app) is RobotState.OBSTACLE_DETECTED
    assert app.session_state.controller.sim.pose == pose


def test_emergency_stop_blocks_start_until_reset(app):
    button(app, "▶ Start").click().run()
    button(app, "🛑 EMERGENCY STOP").click().run()
    assert state(app) is RobotState.EMERGENCY_STOP
    assert any("EMERGENCY STOP" in e.value for e in app.error)
    button(app, "▶ Start").click().run()
    assert state(app) is RobotState.EMERGENCY_STOP
    assert any("REJECTED" in w.value for w in app.warning)
    button(app, "↺ Reset").click().run()
    assert state(app) is RobotState.IDLE


def test_fault_injection_from_sidebar(app):
    app.sidebar.selectbox(key="fault_mode").set_value(SensorFaultMode.NOT_A_NUMBER.value).run()
    assert state(app) is RobotState.FAULT
    assert any("FAULT" in e.value for e in app.error)


def test_scenario_reload_refused_during_emergency_stop(app):
    button(app, "🛑 EMERGENCY STOP").click().run()
    controller = app.session_state.controller
    button(app, "🔄 Load").click().run()
    assert app.session_state.controller is controller
    assert state(app) is RobotState.EMERGENCY_STOP


def test_scenario_reload_resets_settings(app):
    app.sidebar.slider(key="threshold").set_value(1.0).run()
    assert app.session_state.controller.stop_threshold_m == 1.0
    button(app, "🔄 Load").click().run()
    assert app.session_state.controller.stop_threshold_m == 0.5
    assert app.session_state.threshold == 0.5
