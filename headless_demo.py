"""Run every demo scenario without a browser and print what happened.

Usage:  python headless_demo.py
Useful to check the simulator works before opening the dashboard.
"""

from __future__ import annotations

from src.models import RobotState
from src.scenarios import DESTINATIONS, SCENARIOS, build_controller

MAX_STEPS = 1000


def main() -> None:
    print("MobilityBot headless demo - ALL READINGS ARE SIMULATED\n")
    for key, scenario in SCENARIOS.items():
        c = build_controller(key)
        result = c.start()
        print(f"== {scenario.title}")
        print(f"   START: {'accepted' if result.accepted else 'REJECTED'} - {result.message}")
        for _ in range(MAX_STEPS):
            if c.state is not RobotState.MOVING:
                break
            c.tick()
        pose = c.sim.pose
        print(f"   Final state: {c.state}  ({c.state_reason})")
        print(f"   Pose: x={pose.x:.2f} m, y={pose.y:.2f} m, heading={pose.heading_deg:.0f} deg; "
              f"sim time {c.sim.sim_time_s:.1f} s; travelled {c.sim.distance_travelled_m:.2f} m\n")

    print("== Navigation (scenario 1, A* route to each destination)")
    for name, goal in DESTINATIONS.items():
        c = build_controller("obstacle_ahead")
        plan = c.plan_route(goal, name)
        if not plan.ok:
            print(f"   {name}: NO ROUTE - {plan.reason}")
            continue
        c.start()
        for _ in range(MAX_STEPS * 5):
            if c.state is not RobotState.MOVING:
                break
            c.tick()
        print(f"   {name:12s} planned {plan.length_m:.2f} m -> {c.state} ({c.state_reason})")


if __name__ == "__main__":
    main()
