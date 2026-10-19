# CLAUDE.md - MobilityBot project rules

MobilityBot is an **unoccupied** experimental robot prototype for a school expo.
It must never carry a person. The current stage is **Stage 1: software-only simulation**.

## Rules
- Never activate or command real hardware during simulation development. Automated tests must never touch hardware.
- Keep hardware drivers separate from the simulation (future `src/hardware/` backends, disabled by default, chosen only by an explicit config change). `tests/test_misc.py` fails if `src/` or `app.py` imports GPIO/serial libraries.
- Do not invent electrical specifications (voltages, currents, pinouts, battery chemistry). Ask the user for labels, photos or multimeter measurements.
- Never claim untested functionality, and never fabricate test results or sensor data. Report real test output.
- Inspect existing files before editing. Preserve unrelated user work. Don't rewrite the repo unnecessarily.
- Keep dependencies minimal (currently only `streamlit` and `pytest`). No paid APIs or cloud services.
- Run `python -m pytest -q` after every change; fix failures before reporting completion.
- Document important architecture decisions in README.md, and keep `docs/STATE_MACHINE.md` in sync with `ALLOWED_TRANSITIONS`.
- Update the README "Project status" only after verifying the work.

## Safety invariants (must stay true; all are tested)
- Only START (from IDLE) and RESUME (from PAUSED/OBSTACLE_DETECTED) can lead to MOVING, and only with a fresh CLEAR reading.
- Invalid, missing, non-finite or out-of-range readings are never treated as a clear path. They lead to FAULT.
- Obstacle stops are latched. Clearing the obstacle never restarts motion.
- EMERGENCY_STOP is accepted in every state and blocks movement until RESET. RESET never starts motion.
- Fault recovery needs N consecutive valid readings and then RESET. E-stop + reset can't bypass it.
- Dashboard: robot commands only come from button callbacks. Reruns and auto-run (`tick()`) must never start motion or clear latches.

## Layout
- `src/` is pure Python with no Streamlit. `config.py`, `models.py`, `simulator.py`, `sensors/`, `safety.py`, `controller.py`, `event_log.py`, `scenarios.py`, `visualization.py`.
- `app.py` is the Streamlit dashboard (presentation only).
- `tests/` holds pytest tests. `tests/helpers.py` has the fake sensor and builders.

## Commands
- Tests: `python -m pytest -q`
- Dashboard: `python -m streamlit run app.py`
- Headless check: `python headless_demo.py`
