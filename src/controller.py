"""Robot state machine and safety interlocks.

The controller is the only place that changes the robot's state. Every
change goes through ``_transition``, which refuses anything not listed in
``ALLOWED_TRANSITIONS``. See docs/STATE_MACHINE.md for the full table.

Key rules:

* Movement only begins on an explicit START (from IDLE) or RESUME (from
  PAUSED / OBSTACLE_DETECTED), and only if a fresh sensor reading is CLEAR.
* Nothing else (a tick, a reset, a cleared obstacle, a settings change) can
  put the robot into MOVING.
* EMERGENCY_STOP is accepted in every state and rejects all movement until
  RESET.
* An invalid reading puts the robot into FAULT. Leaving FAULT requires a
  number of consecutive valid readings and then an explicit RESET.
* An optional Navigator only *requests* motion. Its request is clamped to the
  motion limits and goes through exactly the same checks as manual driving.
  Arrival or a blocked route ends the run with MOVING -> STOPPED.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

from src.config import SimulationConfig, validate_threshold
from src.event_log import EventLog, Level
from src.models import (
    Command,
    CommandResult,
    FaultInfo,
    FaultKind,
    Obstacle,
    RobotState,
    SafetyAssessment,
    SensorFaultMode,
    SensorReading,
    Verdict,
)
from src.navigation import (
    MotionRequest,
    Navigator,
    OccupancyGrid,
    PlanResult,
    RouteStatus,
    build_occupancy_grid,
    plan_path,
)
from src.safety import assess_reading
from src.sensors.base import DistanceSensor
from src.simulator import Simulator

S = RobotState

ALLOWED_TRANSITIONS: dict[RobotState, frozenset[RobotState]] = {
    S.IDLE: frozenset({S.MOVING, S.EMERGENCY_STOP, S.FAULT}),
    S.MOVING: frozenset({S.PAUSED, S.STOPPED, S.OBSTACLE_DETECTED, S.EMERGENCY_STOP, S.FAULT}),
    S.PAUSED: frozenset({S.MOVING, S.STOPPED, S.IDLE, S.EMERGENCY_STOP, S.FAULT}),
    S.OBSTACLE_DETECTED: frozenset({S.MOVING, S.STOPPED, S.IDLE, S.EMERGENCY_STOP, S.FAULT}),
    S.STOPPED: frozenset({S.IDLE, S.EMERGENCY_STOP, S.FAULT}),
    S.EMERGENCY_STOP: frozenset({S.IDLE, S.FAULT}),
    S.FAULT: frozenset({S.IDLE, S.EMERGENCY_STOP}),
}

STATE_LOG_LEVEL = {
    S.EMERGENCY_STOP: Level.CRITICAL,
    S.FAULT: Level.CRITICAL,
    S.OBSTACLE_DETECTED: Level.WARNING,
}


class InvalidTransitionError(RuntimeError):
    """Raised when code tries a state change that the state machine forbids."""


class RobotController:
    def __init__(
        self,
        simulator: Simulator,
        sensor: DistanceSensor,
        config: SimulationConfig,
        event_log: EventLog | None = None,
    ) -> None:
        self.sim = simulator
        self.sensor = sensor
        self.config = config
        self.log = event_log or EventLog()

        self.state = S.IDLE
        self.state_reason = "Simulation initialised"
        self.stop_threshold_m = config.safety.stop_threshold_m
        self.speed_mps = config.motion.speed_mps
        self.turn_rate_dps = config.motion.turn_rate_dps

        self.last_reading: SensorReading | None = None
        self.last_assessment: SafetyAssessment | None = None
        self.active_fault: FaultInfo | None = None
        self.navigator: Navigator | None = None
        self.recovery_valid_count = 0
        self._clear_while_latched_noted = False

        self.obstacle_stops = 0
        self.faults = 0
        self.emergency_stops = 0
        self.rejected_commands = 0

        self._log(Level.INFO, "SYSTEM", f"Simulation initialised in state {self.state}. Sensor source: {sensor.source}")

    # ------------------------------------------------------------------ commands

    def handle(self, command: Command) -> CommandResult:
        handlers = {
            Command.START: self.start,
            Command.PAUSE: self.pause,
            Command.RESUME: self.resume,
            Command.STOP: self.stop,
            Command.EMERGENCY_STOP: self.emergency_stop,
            Command.RESET: self.reset,
        }
        return handlers[Command(command)]()

    def start(self) -> CommandResult:
        if self.state is not S.IDLE:
            return self._reject(Command.START, self._start_hint())
        return self._begin_motion(Command.START)

    def resume(self) -> CommandResult:
        if self.state not in (S.PAUSED, S.OBSTACLE_DETECTED):
            return self._reject(Command.RESUME, f"Resume is only possible from PAUSED or OBSTACLE_DETECTED (now {self.state})")
        return self._begin_motion(Command.RESUME)

    def pause(self) -> CommandResult:
        if self.state is not S.MOVING:
            return self._reject(Command.PAUSE, f"Pause is only possible while MOVING (now {self.state})")
        self._transition(S.PAUSED, "Paused by user")
        return self._accept(Command.PAUSE, "Paused. Press Resume to continue.")

    def stop(self) -> CommandResult:
        if self.state not in (S.MOVING, S.PAUSED, S.OBSTACLE_DETECTED):
            return self._reject(Command.STOP, f"Stop is only possible while MOVING, PAUSED or OBSTACLE_DETECTED (now {self.state})")
        self._transition(S.STOPPED, "Stopped by user")
        return self._accept(Command.STOP, "Stopped. Press Reset to return to IDLE.")

    def emergency_stop(self) -> CommandResult:
        if self.state is S.EMERGENCY_STOP:
            return self._accept(Command.EMERGENCY_STOP, "Emergency stop is already active")
        self.emergency_stops += 1
        self._transition(S.EMERGENCY_STOP, "Emergency stop pressed")
        return self._accept(Command.EMERGENCY_STOP, "EMERGENCY STOP active. All movement commands are blocked until Reset.")

    def reset(self) -> CommandResult:
        if self.state is S.MOVING:
            return self._reject(Command.RESET, "Robot is moving: press Pause, Stop or Emergency Stop first")
        if self.state is S.IDLE:
            return self._reject(Command.RESET, "Nothing to reset: robot is already IDLE")

        if self.state is S.EMERGENCY_STOP:
            if self.active_fault is not None:
                # E-stop must not become a shortcut around fault recovery.
                self.recovery_valid_count = 0
                self._transition(S.FAULT, f"Emergency stop released; unresolved fault remains: {self.active_fault.detail}")
                return self._accept(Command.RESET, "Emergency stop released, but the earlier fault is still active. Complete fault recovery.")
            self._transition(S.IDLE, "Emergency stop reset by user")
            return self._accept(Command.RESET, "Emergency stop reset. Robot is IDLE and will not move until Start.")

        if self.state is S.FAULT:
            needed = self.config.safety.fault_recovery_readings
            if self.recovery_valid_count < needed:
                return self._reject(
                    Command.RESET,
                    f"Fault recovery incomplete: {self.recovery_valid_count}/{needed} consecutive valid readings. "
                    "Remove the cause and run the simulation for more steps.",
                )
            cleared = self.active_fault
            self.active_fault = None
            self.recovery_valid_count = 0
            self._transition(S.IDLE, f"Fault cleared by user after recovery ({cleared.kind if cleared else 'unknown'})")
            return self._accept(Command.RESET, "Fault cleared. Robot is IDLE and will not move until Start.")

        # PAUSED, OBSTACLE_DETECTED, STOPPED
        self._transition(S.IDLE, f"Reset by user from {self.state}")
        return self._accept(Command.RESET, "Reset. Robot is IDLE and will not move until Start.")

    # ----------------------------------------------------------- simulation loop

    def tick(self) -> None:
        """One simulation step: sense, decide, then (only if MOVING) act."""
        self._sense(count_for_recovery=True)
        if self.state is S.MOVING and self.last_assessment is not None and self.last_assessment.verdict is Verdict.CLEAR:
            request = self._motion_request()
            if request.arrived:
                self._transition(S.STOPPED, f"Destination reached: {self.navigator.label if self.navigator else ''}")
                self.sim.advance_time(self.config.motion.time_step_s)
                return
            if request.blocked_reason is not None:
                self._transition(S.STOPPED, f"Route blocked: {request.blocked_reason}")
                self.sim.advance_time(self.config.motion.time_step_s)
                return
            next_pose = self.sim.predict(request.linear_mps, request.angular_rps, self.config.motion.time_step_s)
            hit = self.sim.collision_at(next_pose)
            if hit is not None:
                self._enter_fault(
                    FaultKind.COLLISION_PREVENTED,
                    f"Next step would hit {hit}, which the sensor did not report (outside the beam). "
                    "Simulated motion was blocked.",
                )
            else:
                self.sim.move_to(next_pose)
        self.sim.advance_time(self.config.motion.time_step_s)

    def run_ticks(self, count: int) -> None:
        for _ in range(count):
            self.tick()

    def observe(self) -> SafetyAssessment:
        """Take a reading and apply safety reactions without moving or advancing time."""
        return self._sense(count_for_recovery=False)

    # ------------------------------------------------------------- configuration

    def set_stop_threshold(self, threshold_m: float) -> None:
        validate_threshold(threshold_m, self.config.sensor)
        if threshold_m == self.stop_threshold_m:
            return
        old = self.stop_threshold_m
        self.stop_threshold_m = float(threshold_m)
        self._log(Level.INFO, "CONFIG", f"Stop threshold changed {old:.2f} m -> {threshold_m:.2f} m")
        self.observe()

    def set_motion(self, speed_mps: float, turn_rate_dps: float) -> None:
        motion = self.config.motion
        if not (math.isfinite(speed_mps) and 0 <= speed_mps <= motion.max_speed_mps):
            raise ValueError(f"Speed must be between 0 and {motion.max_speed_mps} m/s")
        if not (math.isfinite(turn_rate_dps) and abs(turn_rate_dps) <= motion.max_turn_rate_dps):
            raise ValueError(f"Turn rate must be within +/-{motion.max_turn_rate_dps} deg/s")
        if (speed_mps, turn_rate_dps) == (self.speed_mps, self.turn_rate_dps):
            return
        self.speed_mps = float(speed_mps)
        self.turn_rate_dps = float(turn_rate_dps)
        self._log(Level.INFO, "CONFIG", f"Motion set to {speed_mps:.2f} m/s, turn {turn_rate_dps:+.0f} deg/s")

    def set_obstacles(self, obstacles: Iterable[Obstacle]) -> None:
        world = self.sim.world.with_obstacles(tuple(obstacles))  # validates
        if world == self.sim.world:
            return
        old_world = self.sim.world
        self.sim.set_world(world)
        hit = self.sim.collision_at(self.sim.pose)
        if hit is not None:
            self.sim.set_world(old_world)
            raise ValueError(f"{hit} would overlap the robot's current position")
        self._log(Level.INFO, "CONFIG", f"Obstacles updated ({len(world.obstacles)} in room)")
        self._replan_after_world_change()
        self.observe()

    def inject_sensor_fault(self, mode: SensorFaultMode) -> None:
        setter = getattr(self.sensor, "set_fault_mode", None)
        if setter is None:
            raise ValueError("This sensor does not support fault injection")
        mode = SensorFaultMode(mode)
        if getattr(self.sensor, "fault_mode", None) == mode:
            return
        setter(mode)
        level = Level.INFO if mode is SensorFaultMode.NONE else Level.WARNING
        self._log(level, "CONFIG", f"Sensor fault injection set to {mode}")
        self.observe()

    def set_sensor_noise(self, noise_std_m: float) -> None:
        setter = getattr(self.sensor, "set_noise_std", None)
        if setter is None:
            raise ValueError("This sensor does not support simulated noise")
        if getattr(getattr(self.sensor, "config", None), "noise_std_m", None) == noise_std_m:
            return
        setter(noise_std_m)
        self._log(Level.INFO, "CONFIG", f"Simulated sensor noise set to {noise_std_m * 100:.1f} cm std dev")

    # ---------------------------------------------------------------- navigation

    def plan_route(self, goal: tuple[float, float], label: str) -> PlanResult:
        """Plan from the robot's current position. Only allowed while IDLE.

        A successful plan does NOT start motion: the user still has to press Start.
        """
        if self.state is not S.IDLE:
            raise ValueError(f"Routes can only be planned while IDLE (now {self.state}). Press Reset first.")
        result = plan_path(self.occupancy_grid(), (self.sim.pose.x, self.sim.pose.y), goal)
        if result.ok:
            self.navigator = Navigator(result, label, self.config.navigation)
            self._log(Level.INFO, "NAV", f"Route to {label} ({goal[0]:.2f}, {goal[1]:.2f}) planned: "
                                         f"{result.length_m:.2f} m, {len(result.waypoints)} waypoints. Press Start to drive.")
        else:
            self._log(Level.WARNING, "NAV", f"No route to {label} ({goal[0]}, {goal[1]}): {result.status}: {result.reason}")
        return result

    def clear_route(self) -> None:
        if self.state is S.MOVING:
            raise ValueError("Pause or stop the robot before clearing the route")
        if self.navigator is not None:
            self.navigator = None
            self._log(Level.INFO, "NAV", "Route cleared: manual driving mode")

    def occupancy_grid(self) -> OccupancyGrid:
        return build_occupancy_grid(self.sim.world, self.config.robot.radius_m, self.config.navigation)

    # ------------------------------------------------------------------ queries

    def allowed_commands(self) -> list[Command]:
        """Commands the current state accepts (START/RESUME still need a clear path)."""
        table = {
            S.IDLE: [Command.START, Command.EMERGENCY_STOP],
            S.MOVING: [Command.PAUSE, Command.STOP, Command.EMERGENCY_STOP],
            S.PAUSED: [Command.RESUME, Command.STOP, Command.RESET, Command.EMERGENCY_STOP],
            S.OBSTACLE_DETECTED: [Command.RESUME, Command.STOP, Command.RESET, Command.EMERGENCY_STOP],
            S.STOPPED: [Command.RESET, Command.EMERGENCY_STOP],
            S.EMERGENCY_STOP: [Command.RESET],
            S.FAULT: [Command.RESET, Command.EMERGENCY_STOP] if self.fault_recovery_ready else [Command.EMERGENCY_STOP],
        }
        return table[self.state]

    @property
    def fault_recovery_ready(self) -> bool:
        return self.state is S.FAULT and self.recovery_valid_count >= self.config.safety.fault_recovery_readings

    def summary(self) -> dict[str, object]:
        return {
            "state": self.state.value,
            "sim_time_s": round(self.sim.sim_time_s, 2),
            "steps": self.sim.step_count,
            "distance_travelled_m": round(self.sim.distance_travelled_m, 3),
            "obstacle_stops": self.obstacle_stops,
            "faults": self.faults,
            "emergency_stops": self.emergency_stops,
            "rejected_commands": self.rejected_commands,
        }

    # ---------------------------------------------------------------- internals

    def _motion_request(self) -> MotionRequest:
        """Manual sliders, or the navigator's request, clamped to the motion limits."""
        max_angular = math.radians(self.config.motion.max_turn_rate_dps)
        if self.navigator is None:
            return MotionRequest(self.speed_mps, math.radians(self.turn_rate_dps))
        request = self.navigator.request(self.sim.pose, self.speed_mps, max_angular, self.config.motion.time_step_s)
        if request.arrived or request.blocked_reason is not None:
            return request
        linear = request.linear_mps if math.isfinite(request.linear_mps) else 0.0
        angular = request.angular_rps if math.isfinite(request.angular_rps) else 0.0
        return MotionRequest(min(max(linear, 0.0), self.speed_mps), min(max(angular, -max_angular), max_angular))

    def _replan_after_world_change(self) -> None:
        nav = self.navigator
        if nav is None or nav.status is not RouteStatus.ACTIVE:
            return
        result = plan_path(self.occupancy_grid(), (self.sim.pose.x, self.sim.pose.y), nav.goal)
        if result.ok:
            nav.replace_plan(result)
            self._log(Level.INFO, "NAV", f"Obstacles changed: route to {nav.label} re-planned ({result.length_m:.2f} m)")
        else:
            nav.block(f"obstacles changed and no route to {nav.label} remains ({result.reason})")
            self._log(Level.WARNING, "NAV", f"Route to {nav.label} blocked: {result.reason}")
            if self.state is S.MOVING:
                self._transition(S.STOPPED, f"Route blocked: {nav.blocked_reason}")

    def _begin_motion(self, command: Command) -> CommandResult:
        assessment = self._sense(count_for_recovery=False)
        if self.state is S.FAULT:
            return self._reject(command, f"Sensor fault: {assessment.reason}")
        if assessment.verdict is not Verdict.CLEAR:
            return self._reject(command, f"Path not clear: {assessment.reason}")
        request = self._motion_request()
        if request.arrived:
            return self._reject(command, "Already at the destination: plan a new route or clear navigation")
        if request.blocked_reason is not None:
            return self._reject(command, f"Route blocked: {request.blocked_reason}. Plan a new route or clear navigation")
        next_pose = self.sim.predict(request.linear_mps, request.angular_rps, self.config.motion.time_step_s)
        hit = self.sim.collision_at(next_pose)
        if hit is not None:
            return self._reject(command, f"First step would hit {hit}")
        self._clear_while_latched_noted = False
        self._transition(S.MOVING, f"{command.value.title()} by user ({assessment.reason})")
        return self._accept(command, "Moving.")

    def _sense(self, count_for_recovery: bool) -> SafetyAssessment:
        reading = self.sensor.read()
        assessment = assess_reading(reading, self.stop_threshold_m, self.config.sensor)
        previous = self.last_assessment
        self.last_reading, self.last_assessment = reading, assessment
        self._log_sensor_validity_change(previous, assessment)

        if self.state is S.EMERGENCY_STOP:
            return assessment  # e-stop has priority; nothing else changes state

        if assessment.verdict is Verdict.INVALID:
            if self.state is S.FAULT:
                if count_for_recovery and self.recovery_valid_count:
                    self._log(Level.WARNING, "SAFETY", "Invalid reading during fault recovery: recovery count restarted")
                self.recovery_valid_count = 0
            else:
                self._enter_fault(FaultKind.SENSOR_INVALID, assessment.reason)
            return assessment

        if self.state is S.FAULT:
            if count_for_recovery:
                self.recovery_valid_count += 1
                if self.recovery_valid_count == self.config.safety.fault_recovery_readings:
                    self._log(Level.INFO, "SAFETY", f"{self.recovery_valid_count} consecutive valid readings: Reset is now permitted")
        elif self.state is S.MOVING and assessment.verdict is Verdict.OBSTACLE:
            self.obstacle_stops += 1
            self._transition(S.OBSTACLE_DETECTED, assessment.reason)
        elif self.state is S.OBSTACLE_DETECTED and assessment.verdict is Verdict.CLEAR and not self._clear_while_latched_noted:
            self._clear_while_latched_noted = True
            self._log(Level.INFO, "SAFETY", f"Path now appears clear ({assessment.distance_m:.2f} m). Stop stays latched until Resume.")
        return assessment

    def _enter_fault(self, kind: FaultKind, detail: str) -> None:
        self.active_fault = FaultInfo(kind, detail, self.sim.sim_time_s)
        self.recovery_valid_count = 0
        self.faults += 1
        self._transition(S.FAULT, f"{kind}: {detail}")

    def _transition(self, new_state: RobotState, reason: str) -> None:
        if new_state not in ALLOWED_TRANSITIONS[self.state]:
            raise InvalidTransitionError(f"Transition {self.state} -> {new_state} is not permitted")
        old = self.state
        self.state = new_state
        self.state_reason = reason
        self._log(STATE_LOG_LEVEL.get(new_state, Level.INFO), "STATE", f"{old} -> {new_state}: {reason}")

    def _log_sensor_validity_change(self, previous: SafetyAssessment | None, current: SafetyAssessment) -> None:
        was_invalid = previous is not None and previous.verdict is Verdict.INVALID
        is_invalid = current.verdict is Verdict.INVALID
        if is_invalid and not was_invalid:
            self._log(Level.WARNING, "SENSOR", f"Invalid reading: {current.reason}")
        elif was_invalid and not is_invalid:
            self._log(Level.INFO, "SENSOR", f"Valid readings restored ({current.distance_m:.2f} m)")

    def _start_hint(self) -> str:
        hints = {
            S.MOVING: "Robot is already moving",
            S.PAUSED: "Robot is paused: use Resume",
            S.OBSTACLE_DETECTED: "Obstacle stop is latched: use Resume once the path is clear, or Reset",
            S.STOPPED: "Robot is stopped: press Reset first",
            S.EMERGENCY_STOP: "Emergency stop is active: press Reset first",
            S.FAULT: "Robot is in FAULT: complete fault recovery and press Reset first",
        }
        return hints.get(self.state, f"Start is only possible from IDLE (now {self.state})")

    def _accept(self, command: Command, message: str) -> CommandResult:
        self._log(Level.INFO, "COMMAND", f"{command} accepted: {message}")
        return CommandResult(command, True, message, self.state)

    def _reject(self, command: Command, message: str) -> CommandResult:
        self.rejected_commands += 1
        self._log(Level.WARNING, "COMMAND", f"{command} rejected: {message}")
        return CommandResult(command, False, message, self.state)

    def _log(self, level: Level, category: str, message: str) -> None:
        self.log.add(level, category, message, self.sim.sim_time_s)
