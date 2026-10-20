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


# ------------------------------------------------------------ hardware mode

HW = "Hardware telemetry (read-only)"
MOTION_LABELS = ("Start", "Pause", "Resume", "Stop", "Reset", "EMERGENCY", "Step", "Plan route")


def telemetry_line(seq=1, dist=0.85, valid=True):
    import json
    return json.dumps({"v": 1, "seq": seq, "t_ms": seq, "dist_m": dist, "dist_valid": valid,
                       "status": "IDLE", "link": "OK"}).encode() + b"\n"


def hardware_mode(at):
    at.sidebar.radio(key="mode").set_value(HW).run()
    assert not at.exception


def connect_fake(at, stream, port="FAKE1"):
    at.session_state["serial_opener_factory"] = lambda port_name, baud: (lambda: stream)
    at.sidebar.text_input(key="hw_port").set_value(port).run()
    button(at, "🔌 Connect").click().run()
    assert not at.exception


def test_app_startup_and_hardware_mode_do_not_load_serial_library():
    import subprocess
    import sys as _sys
    script = (
        "import sys\n"
        "from streamlit.testing.v1 import AppTest\n"
        f"at = AppTest.from_file({str(APP)!r}, default_timeout=30).run()\n"
        "assert not at.exception\n"
        "print('after_start', 'serial' in sys.modules)\n"
        f"at.sidebar.radio(key='mode').set_value({HW!r}).run()\n"
        "assert not at.exception\n"
        "print('after_hw_mode', 'serial' in sys.modules)\n"
    )
    out = subprocess.run([_sys.executable, "-c", script], capture_output=True, text=True, timeout=120,
                         cwd=str(APP.parent))
    assert out.returncode == 0, out.stderr
    assert "after_start False" in out.stdout and "after_hw_mode False" in out.stdout


def test_hardware_mode_shows_no_simulated_data_and_no_motion_controls(app):
    hardware_mode(app)
    assert any("READ-ONLY" in w.value for w in app.warning)
    assert not any(m.value.startswith("<svg") for m in app.markdown)  # no simulated room map
    labels = [b.label for b in app.button]
    assert not any(word in label for label in labels for word in MOTION_LABELS), labels
    assert any("NOT CONNECTED" in i.value for i in app.info)
    assert not any("SIMULATED" in str(m.value) for m in app.metric)


def test_connect_requires_a_port(app):
    hardware_mode(app)
    button(app, "🔌 Connect").click().run()
    assert "hw_provider" not in app.session_state
    assert any("serial port" in w.value.lower() for w in app.warning)


def test_hardware_mode_shows_valid_fake_telemetry(app):
    from tests.helpers import FakeSerial
    hardware_mode(app)
    connect_fake(app, FakeSerial(telemetry_line(seq=1, dist=0.85)))
    app.run()
    assert any("RECEIVING VALID TELEMETRY" in s.value for s in app.success)
    metrics = {m.label: m.value for m in app.metric}
    assert metrics["Distance (HARDWARE)"] == "0.850 m"
    assert metrics["Sensor validity"] == "✔ VALID"
    assert metrics["Sequence #"] == "1"


def test_hardware_mode_never_shows_invalid_or_missing_readings(app):
    from tests.helpers import FakeSerial
    hardware_mode(app)
    connect_fake(app, FakeSerial(b"not json\n", telemetry_line(seq=1, dist=None, valid=False)))
    for _ in range(2):
        app.run()
    metrics = {m.label: m.value for m in app.metric}
    assert metrics["Distance (HARDWARE)"] == "—"
    assert metrics["Sensor validity"] == "✖ INVALID"
    assert metrics["Messages rejected"] == "1"


def test_waiting_for_data_shows_no_reading(app):
    from tests.helpers import FakeSerial
    hardware_mode(app)
    connect_fake(app, FakeSerial())
    metrics = {m.label: m.value for m in app.metric}
    assert metrics["Distance (HARDWARE)"] == "—"
    assert any("WAITING FOR VALID DATA" in w.value for w in app.warning)


def test_port_open_failure_is_reported(app):
    hardware_mode(app)

    def failing_opener():
        raise OSError("could not open port 'COM7': FileNotFoundError")

    app.session_state["serial_opener_factory"] = lambda port_name, baud: failing_opener
    app.sidebar.text_input(key="hw_port").set_value("COM7").run()
    button(app, "🔌 Connect").click().run()
    assert any("CONNECTION ERROR" in e.value and "COM7" in e.value for e in app.error)


def test_leaving_hardware_mode_closes_port_and_simulation_still_works(app):
    from tests.helpers import FakeSerial
    stream = FakeSerial(telemetry_line())
    hardware_mode(app)
    connect_fake(app, stream)
    app.sidebar.radio(key="mode").set_value("Simulation").run()
    assert stream.closed
    assert app.session_state["hw_provider"] is None
    assert any("SIMULATION MODE" in i.value for i in app.info)
    button(app, "▶ Start").click().run()
    assert state(app) is RobotState.MOVING


def test_hardware_baud_rate_defaults_to_config(app):
    from src.config import TelemetryConfig
    hardware_mode(app)
    assert app.sidebar.selectbox(key="hw_baud").value == TelemetryConfig().baud_rate == 115200


def test_simulation_settings_survive_a_trip_to_hardware_mode(app):
    app.sidebar.slider(key="threshold").set_value(0.8).run()
    app.sidebar.slider(key="speed").set_value(0.3).run()
    app.sidebar.selectbox(key="fault_mode").set_value(SensorFaultMode.NO_ECHO.value).run()
    hardware_mode(app)
    app.run()
    app.sidebar.radio(key="mode").set_value("Simulation").run()
    assert not app.exception
    c = app.session_state.controller
    # What the sliders show must match what the controller actually uses.
    assert app.sidebar.slider(key="threshold").value == c.stop_threshold_m == 0.8
    assert app.sidebar.slider(key="speed").value == c.speed_mps == 0.3
    assert app.sidebar.selectbox(key="fault_mode").value == SensorFaultMode.NO_ECHO.value
    assert app.sidebar.selectbox(key="scenario_select").value == "obstacle_ahead"
    assert app.sidebar.toggle(key="auto_run").value is False
