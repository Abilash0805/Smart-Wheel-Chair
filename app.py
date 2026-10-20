"""MobilityBot dashboard: Simulation mode and read-only Hardware Telemetry mode.

Run with:  streamlit run app.py

Safety rules for this file:
* Robot commands are sent ONLY from button callbacks. Streamlit reruns the
  script many times; re-rendering must never start motion or clear a latch.
* All robot state lives in one RobotController stored in st.session_state.
  This file only displays it and forwards user actions.
* Auto-run only calls tick(), which can never start movement.
* Hardware Telemetry mode is opt-in and READ-ONLY: it displays validated
  messages from a serial port and sends nothing. The serial library is only
  loaded when the user presses Connect. Simulated values never appear there.
"""

from __future__ import annotations

import time

import pandas as pd
import streamlit as st

from src.config import SimulationConfig, TelemetryConfig
from src.controller import RobotController
from src.event_log import Level
from src.models import (
    SENSOR_FAULT_DESCRIPTIONS,
    STATE_INFO,
    Command,
    Obstacle,
    RobotState,
    SensorFaultMode,
    Verdict,
)
from src.hardware.serial_port import SerialUnavailableError, list_serial_ports, make_serial_opener
from src.navigation import RouteStatus
from src.scenarios import DEFAULT_SCENARIO, DESTINATIONS, SCENARIOS, build_controller
from src.telemetry import ConnectionState, SensorValidity, SerialTelemetryProvider, SimulationTelemetryProvider
from src.visualization import render_room_svg

AUTO_RUN_INTERVAL_S = 0.5
AUTO_RUN_TICKS = 5  # 5 steps x 0.1 s = 0.5 s of simulated time per refresh (about real time)
LOG_ROWS = 100

VERDICT_LABEL = {
    Verdict.CLEAR: "✔ CLEAR - path free beyond threshold",
    Verdict.OBSTACLE: "⚠ OBSTACLE - at or inside threshold",
    Verdict.INVALID: "✖ INVALID - cannot be trusted",
}
LEVEL_LABEL = {Level.INFO: "ℹ INFO", Level.WARNING: "⚠ WARNING", Level.CRITICAL: "⛔ CRITICAL"}
OBSTACLE_COLUMNS = ["name", "x", "y", "width", "height"]
CUSTOM_GOAL = "Custom goal"

MODE_SIMULATION = "Simulation"
MODE_HARDWARE = "Hardware telemetry (read-only)"
HARDWARE_REFRESH_S = 0.5
BAUD_RATES = [9600, 19200, 38400, 57600, 115200]
CONNECTION_LABEL = {
    ConnectionState.DISCONNECTED: "○ NOT CONNECTED",
    ConnectionState.ERROR: "✖ CONNECTION ERROR",
    ConnectionState.WAITING_FOR_DATA: "… PORT OPEN - WAITING FOR VALID DATA",
    ConnectionState.CONNECTED: "✔ RECEIVING VALID TELEMETRY",
    ConnectionState.STALE: "⚠ DATA STALE - NOT TRUSTED",
    ConnectionState.RECONNECTING: "↻ LINK LOST - RECONNECTING",
    ConnectionState.SIMULATED: "🧪 SIMULATED - no hardware",
}
VALIDITY_LABEL = {
    SensorValidity.VALID: "✔ VALID",
    SensorValidity.INVALID: "✖ INVALID",
    SensorValidity.STALE: "⚠ STALE - not trusted",
    SensorValidity.NO_DATA: "— NO DATA",
}

ss = st.session_state


# --------------------------------------------------------------------- state

def ctrl() -> RobotController:
    return ss.controller


def load_scenario(key: str) -> None:
    """Build a fresh simulation and put every setting back to its default."""
    defaults = SimulationConfig()
    ss.controller = build_controller(key)
    ss.scenario_key = key  # plain (non-widget) key, so it survives mode switches
    ss.scenario_select = key
    ss.threshold = defaults.safety.stop_threshold_m
    ss.speed = defaults.motion.speed_mps
    ss.turn_rate = defaults.motion.turn_rate_dps
    ss.noise_cm = defaults.sensor.noise_std_m * 100
    ss.fault_mode = SensorFaultMode.NONE.value
    ss.auto_run = False
    ss.destination = next(iter(DESTINATIONS))
    ss.custom_x, ss.custom_y = 2.0, 1.0
    ss.show_grid = True
    ss.obstacle_base = obstacles_to_df(ss.controller.sim.world.obstacles)
    ss.editor_version = ss.get("editor_version", 0) + 1
    ss.flash = ("info", f"Loaded scenario: {SCENARIOS[key].title}. Robot is IDLE.")


def sync_simulation_widgets() -> None:
    """Restore simulation widget values that Streamlit dropped while they were not drawn.

    Streamlit forgets a widget's value on runs where the widget is not shown
    (e.g. while in Hardware mode). Without this, sliders would come back at their
    minimum while the controller still used the real values. The controller is
    the source of truth, so missing values are taken from it.
    """
    c = ctrl()
    restore = {
        "scenario_select": ss.scenario_key,
        "threshold": c.stop_threshold_m,
        "speed": c.speed_mps,
        "turn_rate": c.turn_rate_dps,
        "noise_cm": getattr(getattr(c.sensor, "config", None), "noise_std_m", 0.0) * 100,
        "fault_mode": getattr(c.sensor, "fault_mode", SensorFaultMode.NONE).value,
        "auto_run": False,
        "destination": next(iter(DESTINATIONS)),
        "custom_x": 2.0,
        "custom_y": 1.0,
        "show_grid": True,
    }
    for key, value in restore.items():
        if key not in ss:
            ss[key] = value


def obstacles_to_df(obstacles: tuple[Obstacle, ...]) -> pd.DataFrame:
    return pd.DataFrame(
        [{"name": o.name, "x": o.x, "y": o.y, "width": o.width, "height": o.height} for o in obstacles],
        columns=OBSTACLE_COLUMNS,
    )


def df_to_obstacles(df: pd.DataFrame) -> list[Obstacle]:
    obstacles = []
    for i, row in enumerate(df.to_dict("records"), start=1):
        name = row.get("name")
        name = str(name).strip() if isinstance(name, str) else ""
        numbers = [row.get(c) for c in ("x", "y", "width", "height")]
        if any(v is None or pd.isna(v) for v in numbers):
            raise ValueError(f"Row {i}: fill in x, y, width and height")
        obstacles.append(Obstacle(name or f"Obstacle {i}", *(float(v) for v in numbers)))
    return obstacles


# ----------------------------------------------------------------- callbacks

def on_command(command: Command) -> None:
    result = ctrl().handle(command)
    ss.flash = ("success" if result.accepted else "warning", f"{command.value}: {result.message}")


def on_step(count: int) -> None:
    ctrl().run_ticks(count)


def on_load_scenario() -> None:
    state = ctrl().state
    if state in (RobotState.MOVING, RobotState.EMERGENCY_STOP, RobotState.FAULT):
        ss.flash = (
            "warning",
            f"Scenario not reloaded: robot is {state}. Stop the robot, or clear the "
            "emergency stop / fault with Reset first.",
        )
        return
    load_scenario(ss.scenario_select)


def on_threshold() -> None:
    ctrl().set_stop_threshold(float(ss.threshold))


def on_motion() -> None:
    ctrl().set_motion(float(ss.speed), float(ss.turn_rate))


def on_noise() -> None:
    ctrl().set_sensor_noise(float(ss.noise_cm) / 100)


def on_fault_mode() -> None:
    ctrl().inject_sensor_fault(SensorFaultMode(ss.fault_mode))


def on_auto_run() -> None:
    ss.last_auto_tick = 0.0


def on_plan_route() -> None:
    if ss.destination == CUSTOM_GOAL:
        goal, label = (float(ss.custom_x), float(ss.custom_y)), f"Custom ({ss.custom_x:.2f}, {ss.custom_y:.2f})"
    else:
        goal, label = DESTINATIONS[ss.destination], ss.destination
    try:
        result = ctrl().plan_route(goal, label)
    except ValueError as exc:
        ss.flash = ("warning", f"Route not planned: {exc}")
        return
    if result.ok:
        ss.flash = ("success", f"Route to {label} planned ({result.length_m:.2f} m). "
                               "The robot will NOT move until you press Start.")
    else:
        ss.flash = ("warning", f"No route to {label}: {result.reason}")


def on_clear_route() -> None:
    try:
        ctrl().clear_route()
        ss.flash = ("info", "Route cleared. Start now drives straight using the speed and turn sliders.")
    except ValueError as exc:
        ss.flash = ("warning", f"Route not cleared: {exc}")


def on_mode_change() -> None:
    # Leaving hardware mode always closes the port; nothing keeps reading in the background.
    if ss.mode != MODE_HARDWARE and ss.get("hw_provider") is not None:
        ss.hw_provider.disconnect()
        ss.hw_provider = None


def on_list_ports() -> None:
    try:
        ports = list_serial_ports()
    except SerialUnavailableError as exc:
        ss.hw_flash = ("warning", str(exc))
        return
    ss.hw_flash = ("info", "Ports found: " + (", ".join(f"{d} ({desc})" for d, desc in ports) or "none"))


def on_connect() -> None:
    if ss.get("hw_provider") is not None:
        ss.hw_provider.disconnect()
        ss.hw_provider = None
    factory = ss.get("serial_opener_factory") or make_serial_opener  # tests inject a fake here
    port, baud = str(ss.get("hw_port", "")), int(ss.get("hw_baud", TelemetryConfig().baud_rate))
    try:
        opener = factory(port, baud)
    except ValueError as exc:
        ss.hw_flash = ("warning", str(exc))
        return
    provider = SerialTelemetryProvider(opener, TelemetryConfig(baud_rate=baud), SimulationConfig().sensor,
                                       port.strip())
    provider.connect()
    ss.hw_provider = provider
    ss.hw_flash = ("info", f"Connect requested for {port.strip()} (read-only). See the connection status below.")


def on_disconnect() -> None:
    if ss.get("hw_provider") is not None:
        ss.hw_provider.disconnect()
    ss.hw_flash = ("info", "Disconnected. The port is closed.")


# ------------------------------------------------------------------- layout

def render_header() -> None:
    st.title("🤖 MobilityBot - Autonomous Mobility Prototype")
    st.info(
        "**🧪 SIMULATION MODE.** Every reading on this page is **SIMULATED** from a virtual room. "
        "No hardware is connected or controlled. The on-screen Emergency Stop is a software control, "
        "not a substitute for a physical emergency stop. Simulated results do not prove real-world safety.",
        icon="ℹ️",
    )


def render_sidebar() -> None:
    sb = st.sidebar
    sb.header("Demo scenario")
    sb.selectbox("Scenario", options=list(SCENARIOS), format_func=lambda k: SCENARIOS[k].title, key="scenario_select")
    sb.caption(SCENARIOS[ss.scenario_select].description)
    sb.button("🔄 Load / reset scenario", on_click=on_load_scenario, width="stretch",
              help="Restores the room, robot position and all settings. Not allowed while moving, "
                   "in emergency stop or in fault.")

    sb.header("Simulation")
    sb.toggle("Auto-run (live)", key="auto_run", on_change=on_auto_run,
              help=f"Advances the simulation {AUTO_RUN_TICKS} steps every {AUTO_RUN_INTERVAL_S} s. "
                   "Auto-run only advances time; it never starts movement on its own.")
    motion = ctrl().config.motion
    sb.slider("Speed (m/s)", 0.05, motion.max_speed_mps, step=0.05, key="speed", on_change=on_motion)
    sb.slider("Turn rate (deg/s, + = left)", -45.0, 45.0, step=5.0, key="turn_rate", on_change=on_motion)

    sb.header("Navigation (simulated)")
    sb.selectbox("Destination", options=[*DESTINATIONS, CUSTOM_GOAL], key="destination")
    if ss.destination == CUSTOM_GOAL:
        world = ctrl().sim.world
        cx, cy = sb.columns(2)
        cx.number_input("x (m)", 0.0, world.width_m, step=0.05, format="%.2f", key="custom_x")
        cy.number_input("y (m)", 0.0, world.height_m, step=0.05, format="%.2f", key="custom_y")
    else:
        gx, gy = DESTINATIONS[ss.destination]
        sb.caption(f"{ss.destination} is at ({gx:.2f}, {gy:.2f}) m in the simulated room.")
    plan_col, clear_col = sb.columns(2)
    plan_col.button("🧭 Plan route", on_click=on_plan_route, width="stretch",
                    help="Plans from the robot's current position. Only while IDLE. Does not start motion.")
    clear_col.button("✖ Clear route", on_click=on_clear_route, width="stretch",
                     help="Back to manual straight-line driving. Not allowed while moving.")
    sb.toggle("Show planning grid (no-go zones)", key="show_grid")

    sb.header("Safety")
    sb.slider("Stop threshold (m)", 0.10, 1.50, step=0.05, key="threshold", on_change=on_threshold,
              help="Simulation setting only. Not validated against a real stopping distance.")

    sb.header("Sensor fault injection")
    sb.selectbox("Injected fault", options=[m.value for m in SensorFaultMode],
                 format_func=lambda v: SENSOR_FAULT_DESCRIPTIONS[SensorFaultMode(v)],
                 key="fault_mode", on_change=on_fault_mode)
    sb.slider("Measurement noise (cm, std dev)", 0.0, 5.0, step=0.5, key="noise_cm", on_change=on_noise,
              help="0 = deterministic. Noise uses a fixed random seed so runs are repeatable.")


def render_state_banner(c: RobotController) -> None:
    info = STATE_INFO[c.state]
    body = f"### {info.symbol} {info.label}\n{info.explanation}\n\n**Why:** {c.state_reason}"
    if c.state is RobotState.EMERGENCY_STOP:
        st.error(body, icon="🛑")
    elif c.state is RobotState.FAULT:
        st.error(body, icon="✖️")
    elif c.state is RobotState.OBSTACLE_DETECTED:
        st.warning(body, icon="⚠️")
    else:
        st.info(body, icon="ℹ️")


def render_controls(c: RobotController) -> None:
    cols = st.columns([1, 1, 1, 1, 1, 1.6])
    cols[0].button("▶ Start", on_click=on_command, args=(Command.START,), width="stretch")
    cols[1].button("⏸ Pause", on_click=on_command, args=(Command.PAUSE,), width="stretch")
    cols[2].button("⏯ Resume", on_click=on_command, args=(Command.RESUME,), width="stretch")
    cols[3].button("■ Stop", on_click=on_command, args=(Command.STOP,), width="stretch")
    cols[4].button("↺ Reset", on_click=on_command, args=(Command.RESET,), width="stretch")
    cols[5].button("🛑 EMERGENCY STOP", on_click=on_command, args=(Command.EMERGENCY_STOP,),
                   type="primary", width="stretch")

    step_cols = st.columns([1, 1, 4])
    step_cols[0].button("Step ×1", on_click=on_step, args=(1,), width="stretch",
                        help=f"Advance {c.config.motion.time_step_s:g} s of simulated time")
    step_cols[1].button("Step ×10", on_click=on_step, args=(10,), width="stretch",
                        help=f"Advance {10 * c.config.motion.time_step_s:g} s of simulated time")
    allowed = ", ".join(cmd.value.replace("_", " ").title() for cmd in c.allowed_commands())
    step_cols[2].markdown(f"**Commands accepted now:** {allowed or 'none'}  \n"
                          "_Start and Resume also require a clear, valid sensor reading._")

    # Kept until the next action (not popped) so auto-run refreshes do not hide it.
    flash = ss.get("flash")
    if flash:
        kind, text = flash
        label = {"success": "Last action", "warning": "Last action REJECTED", "info": "Last action"}[kind]
        {"success": st.success, "warning": st.warning, "info": st.info}[kind](f"**{label}:** {text}")


def render_sensor_panel(c: RobotController) -> None:
    st.subheader("Sensor (SIMULATED)")
    snap = SimulationTelemetryProvider(lambda: c).snapshot()
    st.caption(f"Data source: **{snap.source}** · {CONNECTION_LABEL[snap.connection]}")
    a, r = c.last_assessment, c.last_reading
    value = f"{a.distance_m:.2f} m" if a and a.distance_m is not None else "—"
    st.metric("Forward distance", value, help="Nearest surface inside the simulated ultrasonic beam")
    if a:
        st.markdown(f"**Verdict:** {VERDICT_LABEL[a.verdict]}")
        st.caption(a.reason)
    if r:
        st.caption(f"Raw value: `{r.value_m!r}` · source: **{r.source}** · t = {r.sim_time_s:.1f} s")
    st.markdown(f"**Stop threshold:** {c.stop_threshold_m:.2f} m")

    if c.state is RobotState.FAULT and c.active_fault:
        needed = c.config.safety.fault_recovery_readings
        st.markdown(f"**Active fault:** `{c.active_fault.kind}`")
        st.progress(min(c.recovery_valid_count / needed, 1.0),
                    text=f"Recovery: {c.recovery_valid_count}/{needed} consecutive valid readings"
                         + (" - Reset permitted" if c.fault_recovery_ready else ""))

    render_navigation_panel(c)

    pose = c.sim.pose
    st.subheader("Robot pose")
    st.markdown(
        f"x = **{pose.x:.2f} m**, y = **{pose.y:.2f} m**, heading = **{pose.heading_deg:.0f}°**  \n"
        f"Speed setting {c.speed_mps:.2f} m/s, turn {c.turn_rate_dps:+.0f}°/s"
    )


def render_navigation_panel(c: RobotController) -> None:
    st.subheader("Navigation")
    nav = c.navigator
    if nav is None:
        st.markdown("**Mode:** manual - Start drives straight using the speed and turn sliders.")
        return
    status = {
        RouteStatus.ACTIVE: "▶ following route",
        RouteStatus.ARRIVED: "✔ destination reached",
        RouteStatus.BLOCKED: "✖ ROUTE BLOCKED - plan a new route",
    }[nav.status]
    gx, gy = nav.goal
    st.markdown(
        f"**Mode:** route to **{nav.label}** ({gx:.2f}, {gy:.2f}) m  \n"
        f"**Route status:** {status}  \n"
        f"Planned length {nav.plan.length_m:.2f} m · waypoints left {len(nav.remaining_waypoints)} · "
        f"grid {c.config.navigation.resolution_m:g} m cells"
    )
    if nav.blocked_reason:
        st.caption(nav.blocked_reason)
    st.caption("Navigation only requests motion. Obstacle stops, Emergency Stop and FAULT still apply. "
               "The turn-rate slider is ignored while following a route; the speed slider is the maximum.")


def render_summary(c: RobotController) -> None:
    s = c.summary()
    cols = st.columns(7)
    cols[0].metric("Sim time", f"{s['sim_time_s']:.1f} s")
    cols[1].metric("Steps", s["steps"])
    cols[2].metric("Distance", f"{s['distance_travelled_m']:.2f} m")
    cols[3].metric("Obstacle stops", s["obstacle_stops"])
    cols[4].metric("Faults", s["faults"])
    cols[5].metric("E-stops", s["emergency_stops"])
    cols[6].metric("Rejected cmds", s["rejected_commands"])


def render_event_log(c: RobotController) -> None:
    st.subheader("Event log")
    rows = [
        {
            "#": e.seq,
            "Time": e.wall_time.strftime("%H:%M:%S"),
            "Sim t (s)": f"{e.sim_time_s:.1f}",
            "Level": LEVEL_LABEL[e.level],
            "Category": e.category,
            "Message": e.message,
        }
        for e in c.log.latest(LOG_ROWS)
    ]
    st.dataframe(pd.DataFrame(rows), hide_index=True, height=320, width="stretch")
    st.download_button("⬇ Download full log (CSV)", data=c.log.to_csv(), file_name="mobilitybot_event_log.csv",
                       mime="text/csv", on_click="ignore")


def maybe_auto_tick(c: RobotController) -> None:
    if not ss.get("auto_run"):
        return
    now = time.monotonic()
    if now - ss.get("last_auto_tick", 0.0) >= AUTO_RUN_INTERVAL_S * 0.8:
        ss.last_auto_tick = now
        c.run_ticks(AUTO_RUN_TICKS)


def live_panel() -> None:
    c = ctrl()
    maybe_auto_tick(c)
    render_state_banner(c)
    render_controls(c)
    map_col, info_col = st.columns([3, 2], gap="large")
    with map_col:
        st.markdown(render_room_svg(c, show_grid=ss.get("show_grid", True)), unsafe_allow_html=True)
        st.caption(
            "Legend: hatched boxes = obstacles · grey band = no-go zone for the robot centre (planning grid) · "
            "purple line with S / G = planned route, start and goal (dashed grey if blocked) · "
            "yellow circle + arrow = robot and heading · shaded wedge = simulated ultrasonic beam "
            "(dashed grey when the reading is invalid) · red dashed arc = stop threshold · dotted line = path travelled."
        )
    with info_col:
        render_sensor_panel(c)
    st.subheader("Simulation status")
    render_summary(c)
    render_event_log(c)


def render_obstacle_editor() -> None:
    with st.expander("✏️ Edit obstacles (positions in metres, room origin at bottom-left)"):
        st.caption(
            "x, y = lower-left corner. Obstacles must stay inside the room and must not overlap the robot. "
            "Changes apply immediately, even while moving - try placing a box in front of the robot."
        )
        edited = st.data_editor(
            ss.obstacle_base,
            key=f"obstacle_editor_{ss.editor_version}",
            num_rows="dynamic",
            hide_index=True,
            column_config={
                "name": st.column_config.TextColumn("Name", required=True),
                "x": st.column_config.NumberColumn("x (m)", min_value=0.0, step=0.05, format="%.2f"),
                "y": st.column_config.NumberColumn("y (m)", min_value=0.0, step=0.05, format="%.2f"),
                "width": st.column_config.NumberColumn("Width (m)", min_value=0.01, step=0.05, format="%.2f"),
                "height": st.column_config.NumberColumn("Height (m)", min_value=0.01, step=0.05, format="%.2f"),
            },
        )
        try:
            ctrl().set_obstacles(df_to_obstacles(edited))
        except ValueError as exc:
            st.error(f"Obstacle change NOT applied: {exc}", icon="✖️")


def render_hardware_header() -> None:
    st.title("🤖 MobilityBot - Hardware Telemetry")
    st.warning(
        "**🔌 HARDWARE TELEMETRY MODE - READ-ONLY.** This page only shows validated messages received "
        "from the device over a serial port. It sends nothing to the device and has **no motor controls**. "
        "Simulated values are never shown here. A reading is only displayed when a fresh, valid message "
        "has actually been received.",
        icon="⚠️",
    )


def render_hardware_sidebar() -> None:
    sb = st.sidebar
    if "hw_baud" not in ss:
        ss.hw_baud = TelemetryConfig().baud_rate  # otherwise the selectbox would default to its first entry
    sb.header("Serial connection (read-only)")
    sb.text_input("Serial port", key="hw_port", placeholder="e.g. COM3 (Windows) or /dev/ttyACM0",
                  help="Find it in Windows Device Manager → Ports (COM & LPT), or press List serial ports.")
    sb.selectbox("Baud rate", BAUD_RATES, key="hw_baud",
                 help="Must match the device firmware. This is a software setting, not a wiring value.")
    sb.button("🔍 List serial ports", on_click=on_list_ports, width="stretch",
              help="Lists ports only; opens nothing. Needs the optional pyserial package.")
    connect_col, disconnect_col = sb.columns(2)
    connect_col.button("🔌 Connect", on_click=on_connect, width="stretch",
                       help="Opens the port for READING only. Opening an Arduino Uno's port normally resets the board.")
    disconnect_col.button("⏏ Disconnect", on_click=on_disconnect, width="stretch")
    sb.caption("Telemetry is read-only: no commands are ever sent to the device.")


def hardware_panel() -> None:
    provider: SerialTelemetryProvider | None = ss.get("hw_provider")
    flash = ss.get("hw_flash")
    if flash:
        {"info": st.info, "warning": st.warning}[flash[0]](flash[1])
    if provider is None:
        st.info("○ NOT CONNECTED. No hardware data. Enter the serial port in the sidebar and press Connect.",
                icon="ℹ️")
        return
    provider.poll()
    snap = provider.snapshot()

    label = CONNECTION_LABEL[snap.connection]
    detail = f"**Port:** {provider.port_label} · **Source:** {snap.source}"
    if snap.connection is ConnectionState.CONNECTED:
        st.success(f"### {label}\n{detail}", icon="✅")
    elif snap.connection in (ConnectionState.STALE, ConnectionState.WAITING_FOR_DATA):
        st.warning(f"### {label}\n{detail}", icon="⚠️")
    elif snap.connection in (ConnectionState.ERROR, ConnectionState.RECONNECTING):
        st.error(f"### {label}\n{detail}\n\n**Problem:** {snap.last_error or 'unknown'}", icon="✖️")
    else:
        st.info(f"### {label}\n{detail}", icon="ℹ️")
    if snap.connection is ConnectionState.STALE:
        st.warning(f"⚠ STALE DATA: no valid message for {snap.last_message_age_s:.1f} s. "
                   "The last reading is NOT shown and must not be trusted.")

    cols = st.columns(4)
    cols[0].metric("Distance (HARDWARE)", f"{snap.distance_m:.3f} m" if snap.distance_m is not None else "—",
                   help="Shown only for a fresh message that the device marks valid and the host range check accepts.")
    cols[1].metric("Sensor validity", VALIDITY_LABEL[snap.sensor_validity])
    cols[2].metric("Last valid message",
                   f"{snap.last_message_age_s:.1f} s ago" if snap.last_message_age_s is not None else "never")
    cols[3].metric("Sequence #", snap.seq if snap.seq is not None else "—")
    st.caption(f"Why: {snap.validity_reason}")
    if snap.last_message_wall is not None:
        st.caption(f"Last valid message received at {snap.last_message_wall.strftime('%H:%M:%S')} (host clock).")

    st.markdown(
        f"**Device-reported controller status:** `{snap.reported_status or '—'}` "
        "_(as reported by the device; not verified by this dashboard)_  \n"
        f"**Device-reported link health:** `{snap.device_link or '—'}`"
    )
    counts = st.columns(4)
    counts[0].metric("Messages accepted", snap.messages_ok)
    counts[1].metric("Messages rejected", snap.messages_rejected)
    counts[2].metric("Messages missed (seq gaps)", snap.messages_missed)
    counts[3].metric("Reconnects", snap.reconnects)
    if snap.last_error and snap.connection not in (ConnectionState.ERROR, ConnectionState.RECONNECTING):
        st.caption(f"Last problem: {snap.last_error}")

    st.subheader("Telemetry event log")
    rows = [{"#": e.seq, "Time": e.wall_time.strftime("%H:%M:%S"), "Level": LEVEL_LABEL[e.level],
             "Message": e.message} for e in provider.log.latest(LOG_ROWS)]
    st.dataframe(pd.DataFrame(rows, columns=["#", "Time", "Level", "Message"]), hide_index=True,
                 height=260, width="stretch")


def main() -> None:
    st.set_page_config(page_title="MobilityBot", page_icon="🤖", layout="wide")
    if "controller" not in ss:
        load_scenario(DEFAULT_SCENARIO)
    st.sidebar.radio("Dashboard mode", [MODE_SIMULATION, MODE_HARDWARE], key="mode", on_change=on_mode_change,
                     help="Simulation is the default. Hardware telemetry is read-only and opens nothing "
                          "until you press Connect.")
    if ss.mode == MODE_HARDWARE:
        render_hardware_header()
        render_hardware_sidebar()
        provider = ss.get("hw_provider")
        live = provider is not None and provider.is_active
        st.fragment(hardware_panel, run_every=HARDWARE_REFRESH_S if live else None)()
        st.caption("Unoccupied experimental prototype: never place a person on the robot.")
        return

    sync_simulation_widgets()
    render_header()
    render_sidebar()
    live_area = st.container()
    # The editor runs before the live panel is drawn, so obstacle edits show immediately.
    render_obstacle_editor()
    with live_area:
        st.fragment(live_panel, run_every=AUTO_RUN_INTERVAL_S if ss.auto_run else None)()
    st.caption(
        "MobilityBot Stage 1 - software-only simulation for a school expo. "
        "Unoccupied experimental prototype: never place a person on the robot."
    )


main()
