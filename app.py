"""MobilityBot Stage 1 dashboard (simulation only).

Run with:  streamlit run app.py

Safety rules for this file:
* Robot commands are sent ONLY from button callbacks. Streamlit reruns the
  script many times; re-rendering must never start motion or clear a latch.
* All robot state lives in one RobotController stored in st.session_state.
  This file only displays it and forwards user actions.
* Auto-run only calls tick(), which can never start movement.
"""

from __future__ import annotations

import time

import pandas as pd
import streamlit as st

from src.config import SimulationConfig
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
from src.scenarios import DEFAULT_SCENARIO, SCENARIOS, build_controller
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

ss = st.session_state


# --------------------------------------------------------------------- state

def ctrl() -> RobotController:
    return ss.controller


def load_scenario(key: str) -> None:
    """Build a fresh simulation and put every setting back to its default."""
    defaults = SimulationConfig()
    ss.controller = build_controller(key)
    ss.scenario_select = key
    ss.threshold = defaults.safety.stop_threshold_m
    ss.speed = defaults.motion.speed_mps
    ss.turn_rate = defaults.motion.turn_rate_dps
    ss.noise_cm = defaults.sensor.noise_std_m * 100
    ss.fault_mode = SensorFaultMode.NONE.value
    ss.auto_run = False
    ss.obstacle_base = obstacles_to_df(ss.controller.sim.world.obstacles)
    ss.editor_version = ss.get("editor_version", 0) + 1
    ss.flash = ("info", f"Loaded scenario: {SCENARIOS[key].title}. Robot is IDLE.")


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

    pose = c.sim.pose
    st.subheader("Robot pose")
    st.markdown(
        f"x = **{pose.x:.2f} m**, y = **{pose.y:.2f} m**, heading = **{pose.heading_deg:.0f}°**  \n"
        f"Speed setting {c.speed_mps:.2f} m/s, turn {c.turn_rate_dps:+.0f}°/s"
    )


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
        st.markdown(render_room_svg(c), unsafe_allow_html=True)
        st.caption(
            "Legend: hatched boxes = obstacles · yellow circle + arrow = robot and heading · "
            "shaded wedge = simulated ultrasonic beam (dashed grey when the reading is invalid) · "
            "red dashed arc = stop threshold · dotted line = path travelled."
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


def main() -> None:
    st.set_page_config(page_title="MobilityBot Simulator", page_icon="🤖", layout="wide")
    if "controller" not in ss:
        load_scenario(DEFAULT_SCENARIO)
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
