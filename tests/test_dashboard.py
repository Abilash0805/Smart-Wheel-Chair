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


def enable_auto_run(at):
    at.sidebar.toggle(key="auto_run").set_value(True).run()


def test_auto_run_advances_time_but_never_starts_motion(app):
    enable_auto_run(app)
    for _ in range(5):
        app.session_state.last_auto_tick = 0.0  # pretend the refresh interval has elapsed
        app.run()
    controller = app.session_state.controller
    assert controller.sim.step_count > 0  # auto-run really ticked
    assert state(app) is RobotState.IDLE
    assert controller.sim.distance_travelled_m == 0.0


def test_auto_run_never_releases_obstacle_latch(app):
    button(app, "▶ Start").click().run()
    enable_auto_run(app)
    for _ in range(40):
        app.session_state.last_auto_tick = 0.0
        app.run()
        if state(app) is not RobotState.MOVING:
            break
    assert state(app) is RobotState.OBSTACLE_DETECTED
    pose = app.session_state.controller.sim.pose
    for _ in range(10):
        app.session_state.last_auto_tick = 0.0
        app.run()
    assert state(app) is RobotState.OBSTACLE_DETECTED
    assert app.session_state.controller.sim.pose == pose


def plan(at, destination, x=None, y=None):
    at.sidebar.selectbox(key="destination").set_value(destination).run()
    if x is not None:
        at.sidebar.number_input(key="custom_x").set_value(x).run()
        at.sidebar.number_input(key="custom_y").set_value(y).run()
    button(at, "🧭 Plan route").click().run()


def test_plan_predefined_route_shows_on_map_without_moving(app):
    plan(app, "Kitchen")
    c = app.session_state.controller
    assert c.navigator is not None and c.navigator.label == "Kitchen"
    assert state(app) is RobotState.IDLE
    svg = next(m.value for m in app.markdown if m.value.startswith("<svg"))
    assert "GOAL: Kitchen" in svg and "Planned route" in svg and "No-go zone" in svg
    for _ in range(3):
        app.run()
    assert c.sim.distance_travelled_m == 0.0


def test_dashboard_drives_route_to_destination(app):
    plan(app, "Bedroom")
    button(app, "▶ Start").click().run()
    for _ in range(10):
        button(app, "Step ×10").click().run()
        if state(app) is not RobotState.MOVING:
            break
    assert state(app) is RobotState.STOPPED
    assert "Destination reached: Bedroom" in app.session_state.controller.state_reason


def test_invalid_custom_goal_is_reported(app):
    plan(app, "Custom goal", 2.8, 1.5)  # inside Box A
    assert app.session_state.controller.navigator is None
    assert any("No route" in w.value for w in app.warning)


def test_valid_custom_goal(app):
    plan(app, "Custom goal", 1.5, 0.6)
    assert app.session_state.controller.navigator.goal == (1.5, 0.6)


def test_planning_refused_while_moving(app):
    button(app, "▶ Start").click().run()
    plan(app, "Kitchen")
    assert app.session_state.controller.navigator is None
    assert state(app) is RobotState.MOVING
    assert any("Route not planned" in w.value for w in app.warning)
