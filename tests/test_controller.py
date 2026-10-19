import itertools
import math

import pytest

from src.config import MotionConfig, SimulationConfig
from src.controller import ALLOWED_TRANSITIONS, InvalidTransitionError
from src.event_log import Level
from src.models import Command, FaultKind, Obstacle, Pose, RobotState, SensorFaultMode
from tests.helpers import make_geometric, make_scripted

S = RobotState
CLEAR = 2.0
NEAR = 0.3
BOX_AHEAD = Obstacle("Box", 2.6, 1.2, 0.4, 0.6)


def drive_until_not_moving(c, limit=1000):
    for _ in range(limit):
        if c.state is not S.MOVING:
            return
        c.tick()
    raise AssertionError("robot never stopped")


def put_in_state(target: RobotState):
    """Return (controller, sensor) placed in the target state by legitimate commands."""
    c, sensor = make_scripted([CLEAR])
    if target is S.IDLE:
        return c, sensor
    if target is S.EMERGENCY_STOP:
        c.emergency_stop()
        return c, sensor
    if target is S.FAULT:
        sensor.set(None)
        c.tick()
        return c, sensor
    assert c.start().accepted
    if target is S.PAUSED:
        c.pause()
    elif target is S.STOPPED:
        c.stop()
    elif target is S.OBSTACLE_DETECTED:
        sensor.set(NEAR)
        c.tick()
    assert c.state is target
    sensor.set(CLEAR)
    return c, sensor


# ------------------------------------------------------------ basics

def test_initial_state_is_idle_and_stationary():
    c, _ = make_scripted()
    assert c.state is S.IDLE
    c.run_ticks(50)
    assert c.state is S.IDLE
    assert c.sim.pose == Pose(0.6, 1.5, 0.0)


def test_start_moves_robot_deterministically():
    c, _ = make_scripted([CLEAR])
    assert c.start().accepted
    c.run_ticks(10)  # 10 x 0.1 s at 0.2 m/s
    assert c.sim.pose.x == pytest.approx(0.8)
    assert c.sim.pose.y == pytest.approx(1.5)
    assert c.sim.sim_time_s == pytest.approx(1.0)


def test_turning_updates_heading():
    c, _ = make_scripted([CLEAR])
    c.set_motion(0.0, 45.0)
    c.start()
    c.run_ticks(10)
    assert c.sim.pose.heading_deg == pytest.approx(45.0)
    assert (c.sim.pose.x, c.sim.pose.y) == pytest.approx((0.6, 1.5))


@pytest.mark.parametrize("state", [s for s in S if s is not S.IDLE])
def test_start_only_from_idle(state):
    c, _ = put_in_state(state)
    before = c.state
    assert not c.start().accepted
    assert c.state is before


# ---------------------------------------------------------- obstacle

def test_start_rejected_when_obstacle_inside_threshold():
    c, _ = make_scripted([NEAR])
    result = c.start()
    assert not result.accepted
    assert c.state is S.IDLE


def test_start_rejected_at_exact_threshold():
    c, _ = make_scripted([0.5])
    assert not c.start().accepted


def test_obstacle_at_threshold_stops_robot():
    c, sensor = make_scripted([CLEAR])
    c.start()
    sensor.set(0.5)
    x_before = c.sim.pose.x
    c.tick()
    assert c.state is S.OBSTACLE_DETECTED
    assert c.sim.pose.x == x_before  # no movement on the step the obstacle is seen


def test_obstacle_stop_is_latched_after_obstacle_clears():
    c, sensor = make_scripted([CLEAR])
    c.start()
    sensor.set(NEAR)
    c.tick()
    pose = c.sim.pose
    sensor.set(CLEAR)  # obstacle removed
    c.run_ticks(200)
    assert c.state is S.OBSTACLE_DETECTED
    assert c.sim.pose == pose
    assert any("stays latched" in e.message for e in c.log.entries)


def test_resume_after_obstacle_clears_requires_explicit_command():
    c, sensor = make_scripted([CLEAR])
    c.start()
    sensor.set(NEAR)
    c.tick()
    assert not c.resume().accepted  # still blocked
    assert c.state is S.OBSTACLE_DETECTED
    sensor.set(CLEAR)
    assert c.resume().accepted
    assert c.state is S.MOVING


def test_geometric_obstacle_stop_and_overshoot_bound():
    c = make_geometric((BOX_AHEAD,))
    assert c.start().accepted
    drive_until_not_moving(c)
    assert c.state is S.OBSTACLE_DETECTED
    step = c.speed_mps * c.config.motion.time_step_s
    true_distance = c.sensor.true_distance_m()
    # Simple stop rule: at most one step past the threshold, never touching the box.
    assert c.stop_threshold_m - step - 1e-9 <= true_distance <= c.stop_threshold_m
    assert c.sim.collision_at(c.sim.pose) is None


def test_moving_obstacle_into_path_stops_immediately():
    c = make_geometric()
    c.start()
    c.run_ticks(5)
    c.set_obstacles([Obstacle("Surprise", 1.0, 1.3, 0.2, 0.4)])
    assert c.state is S.OBSTACLE_DETECTED


def test_lowering_threshold_does_not_release_latch():
    c, sensor = make_scripted([CLEAR])
    c.start()
    sensor.set(0.45)
    c.tick()
    c.set_stop_threshold(0.2)
    c.run_ticks(20)
    assert c.state is S.OBSTACLE_DETECTED


def test_raising_threshold_while_moving_stops_robot():
    c, sensor = make_scripted([1.0])
    c.start()
    c.set_stop_threshold(1.2)
    assert c.state is S.OBSTACLE_DETECTED


# ------------------------------------------------------- pause/resume

def test_pause_halts_motion_and_resume_continues():
    c, _ = make_scripted([CLEAR])
    c.start()
    c.run_ticks(5)
    assert c.pause().accepted
    pose = c.sim.pose
    c.run_ticks(50)
    assert c.state is S.PAUSED
    assert c.sim.pose == pose
    assert c.resume().accepted
    c.tick()
    assert c.sim.pose.x > pose.x


def test_resume_from_pause_blocked_by_obstacle():
    c, sensor = make_scripted([CLEAR])
    c.start()
    c.pause()
    sensor.set(NEAR)
    assert not c.resume().accepted
    assert c.state is S.PAUSED


@pytest.mark.parametrize("state", [S.IDLE, S.MOVING, S.STOPPED, S.EMERGENCY_STOP, S.FAULT])
def test_resume_rejected_outside_paused_or_obstacle(state):
    c, _ = put_in_state(state)
    assert not c.resume().accepted
    assert c.state is state


# --------------------------------------------------------- emergency stop

@pytest.mark.parametrize("state", list(S))
def test_emergency_stop_accepted_in_every_state(state):
    c, _ = put_in_state(state)
    assert c.emergency_stop().accepted
    assert c.state is S.EMERGENCY_STOP


@pytest.mark.parametrize("command", [Command.START, Command.RESUME, Command.PAUSE, Command.STOP])
def test_emergency_stop_rejects_movement_commands(command):
    c, _ = make_scripted([CLEAR])
    c.start()
    c.emergency_stop()
    pose = c.sim.pose
    assert not c.handle(command).accepted
    c.run_ticks(50)
    assert c.state is S.EMERGENCY_STOP
    assert c.sim.pose == pose


def test_reset_after_emergency_stop_goes_idle_without_moving():
    c, _ = make_scripted([CLEAR])
    c.start()
    c.emergency_stop()
    assert c.reset().accepted
    assert c.state is S.IDLE
    pose = c.sim.pose
    c.run_ticks(100)
    assert c.state is S.IDLE
    assert c.sim.pose == pose


def test_emergency_stop_ignores_sensor_faults_until_reset():
    c, sensor = make_scripted([CLEAR])
    c.emergency_stop()
    sensor.set(math.nan)
    c.run_ticks(5)
    assert c.state is S.EMERGENCY_STOP
    c.reset()
    c.tick()
    assert c.state is S.FAULT


def test_emergency_stop_cannot_bypass_fault_recovery():
    c, sensor = make_scripted([None])
    c.tick()
    assert c.state is S.FAULT
    c.emergency_stop()
    sensor.set(CLEAR)
    assert c.reset().accepted
    assert c.state is S.FAULT  # back to fault, not idle
    assert c.recovery_valid_count == 0


# ----------------------------------------------------------------- reset

@pytest.mark.parametrize("state", [S.PAUSED, S.OBSTACLE_DETECTED, S.STOPPED])
def test_reset_returns_to_idle_without_movement(state):
    c, _ = put_in_state(state)
    assert c.reset().accepted
    assert c.state is S.IDLE
    pose = c.sim.pose
    c.run_ticks(100)
    assert c.sim.pose == pose


def test_reset_rejected_while_moving():
    c, _ = put_in_state(S.MOVING)
    assert not c.reset().accepted
    assert c.state is S.MOVING


def test_stop_then_start_requires_reset():
    c, _ = put_in_state(S.STOPPED)
    assert not c.start().accepted
    c.reset()
    assert c.start().accepted


# ----------------------------------------------------------------- faults

@pytest.mark.parametrize("bad", [None, math.nan, math.inf, -math.inf, -1.0, 0.0, 12.0, "x"])
@pytest.mark.parametrize("state", [S.IDLE, S.MOVING, S.PAUSED, S.OBSTACLE_DETECTED, S.STOPPED])
def test_invalid_reading_causes_fault(state, bad):
    c, sensor = put_in_state(state)
    sensor.set(bad)
    pose = c.sim.pose
    c.tick()
    assert c.state is S.FAULT
    assert c.active_fault.kind is FaultKind.SENSOR_INVALID
    assert c.sim.pose == pose


def test_start_with_invalid_reading_is_rejected_and_faults():
    c, _ = make_scripted([math.nan])
    assert not c.start().accepted
    assert c.state is S.FAULT


@pytest.mark.parametrize("command", [Command.START, Command.RESUME, Command.PAUSE, Command.STOP])
def test_fault_rejects_commands(command):
    c, _ = put_in_state(S.FAULT)
    assert not c.handle(command).accepted
    assert c.state is S.FAULT


def test_fault_recovery_sequence():
    c, sensor = make_scripted([CLEAR])
    c.start()
    sensor.set(None)
    c.tick()
    assert c.state is S.FAULT
    needed = c.config.safety.fault_recovery_readings

    assert not c.reset().accepted  # cause still present
    sensor.set(CLEAR)
    for i in range(needed - 1):
        c.tick()
        assert not c.reset().accepted, f"reset allowed after only {i + 1} readings"
    sensor.set(math.nan)  # relapse restarts the count
    c.tick()
    assert c.recovery_valid_count == 0
    sensor.set(CLEAR)
    c.run_ticks(needed)
    assert c.fault_recovery_ready
    pose = c.sim.pose
    assert c.reset().accepted
    assert c.state is S.IDLE
    assert c.active_fault is None
    c.run_ticks(50)
    assert c.sim.pose == pose  # recovery never restarts motion


def test_observe_does_not_count_towards_recovery():
    c, sensor = put_in_state(S.FAULT)
    sensor.set(CLEAR)
    for _ in range(10):
        c.observe()
    assert c.recovery_valid_count == 0


def test_blind_spot_collision_is_prevented_and_faults():
    post = Obstacle("Post", 2.2, 1.25, 0.08, 0.12)
    c = make_geometric((post,))
    c.start()
    drive_until_not_moving(c)
    assert c.state is S.FAULT
    assert c.active_fault.kind is FaultKind.COLLISION_PREVENTED
    assert c.sim.collision_at(c.sim.pose) is None


def test_start_rejected_if_first_step_collides():
    # A small post beside/behind the sensor: the beam reads the far wall (clear),
    # but the robot body would touch the post on its first step.
    post = Obstacle("Post", 1.08, 1.35, 0.02, 0.022)
    c = make_geometric((post,), pose=Pose(1.0, 1.5, 0.0))
    assert c.last_assessment.verdict.value == "CLEAR"
    result = c.start()
    assert not result.accepted
    assert "Post" in result.message
    assert c.state is S.IDLE


# ---------------------------------------------------- transition table

def test_transition_table_covers_every_state():
    assert set(ALLOWED_TRANSITIONS) == set(S)


@pytest.mark.parametrize("source, target", list(itertools.product(S, S)))
def test_transition_table_is_enforced(source, target):
    c, _ = make_scripted()
    c.state = source
    if target in ALLOWED_TRANSITIONS[source]:
        c._transition(target, "test")
        assert c.state is target
    else:
        with pytest.raises(InvalidTransitionError):
            c._transition(target, "test")
        assert c.state is source


def test_only_start_and_resume_lead_to_moving():
    assert {s for s, targets in ALLOWED_TRANSITIONS.items() if S.MOVING in targets} == {
        S.IDLE, S.PAUSED, S.OBSTACLE_DETECTED,
    }


@pytest.mark.parametrize("state", list(S))
def test_allowed_commands_match_behaviour(state):
    for command in Command:
        c, sensor = put_in_state(state)
        if state is S.FAULT:
            c.run_ticks(c.config.safety.fault_recovery_readings)
        allowed = command in c.allowed_commands()
        result = c.handle(command)
        if command is Command.EMERGENCY_STOP and state is S.EMERGENCY_STOP:
            continue  # harmless no-op, deliberately not advertised
        assert result.accepted == allowed, (state, command, result.message)


def test_no_automatic_restart_from_any_stopped_state():
    for state in (S.IDLE, S.PAUSED, S.OBSTACLE_DETECTED, S.STOPPED, S.EMERGENCY_STOP, S.FAULT):
        c, sensor = put_in_state(state)
        sensor.set(CLEAR)
        c.set_stop_threshold(0.3)
        c.set_motion(0.4, 10.0)
        c.run_ticks(200)
        assert c.state is not S.MOVING, state


# ------------------------------------------------------------ settings

def test_invalid_settings_rejected():
    c, _ = make_scripted()
    with pytest.raises(ValueError):
        c.set_stop_threshold(10.0)
    with pytest.raises(ValueError):
        c.set_motion(5.0, 0.0)
    with pytest.raises(ValueError):
        c.set_motion(0.2, math.nan)
    assert c.stop_threshold_m == 0.5


def test_obstacle_overlapping_robot_rejected():
    c = make_geometric()
    with pytest.raises(ValueError):
        c.set_obstacles([Obstacle("OnTop", 0.5, 1.4, 0.2, 0.2)])
    assert c.sim.world.obstacles == ()


def test_fault_injection_via_controller():
    c = make_geometric()
    c.inject_sensor_fault(SensorFaultMode.NO_ECHO)
    assert c.state is S.FAULT
    c.inject_sensor_fault(SensorFaultMode.NONE)
    c.run_ticks(3)
    assert c.reset().accepted


def test_events_are_logged():
    c, sensor = make_scripted([CLEAR])
    c.start()
    c.emergency_stop()
    c.start()
    messages = [e.message for e in c.log.entries]
    assert any("IDLE -> MOVING" in m for m in messages)
    assert any("MOVING -> EMERGENCY_STOP" in m for m in messages)
    assert any("START rejected" in m for m in messages)
    assert c.log.count(Level.CRITICAL) == 1
    assert c.rejected_commands == 1


def test_zero_speed_config_allowed():
    cfg = SimulationConfig(motion=MotionConfig(speed_mps=0.0))
    c, _ = make_scripted([CLEAR], config=cfg)
    c.start()
    c.run_ticks(10)
    assert c.sim.pose.x == pytest.approx(0.6)
