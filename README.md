# MobilityBot 🤖

A low-budget autonomous mobility **prototype** for a school exhibition. It is a small,
**unoccupied** wheeled robot that will eventually show obstacle stops, simple navigation,
a live dashboard, and some sensing experiments.

> ⚠️ **This is not a wheelchair or mobility aid, and it must never carry a person.**
> Everything in this repository is currently **simulation only**. Simulated results
> do not prove real-world safety.

## Current stage: Stage 1, working simulation ✅

A deterministic 2-D simulator, a simulated HC-SR04-style ultrasonic sensor, a safety
state machine, a Streamlit dashboard, and automated tests. **There is no hardware
integration of any kind** (a test fails if GPIO/serial libraries are imported).

## Setup (Windows, Python 3.12)

Run these in **PowerShell** in the folder where you want the project:

```powershell
git clone https://github.com/Abilash0805/Smart-Wheel-Chair.git
cd Smart-Wheel-Chair
git checkout claude/dazzling-clarke-7bvux9
python --version                 # must print Python 3.12.x
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

If `python --version` doesn't show 3.12, try `py -3.12 -m venv .venv` instead. If neither works,
install Python 3.12 from python.org with **"Add python.exe to PATH"** ticked, then reopen PowerShell.
(On macOS/Linux, use `python3.12 -m venv .venv` and `.venv/bin/python` instead.)

Dependencies are deliberately minimal: **streamlit** (dashboard) and **pytest** (tests).
Everything else is the Python standard library. Streamlit brings its own dependencies
(e.g. pandas). No API keys, accounts or internet services are used at runtime.

## Run

| What | Command (PowerShell, from the project folder) |
|---|---|
| Dashboard | `.\.venv\Scripts\python.exe -m streamlit run app.py` → opens http://localhost:8501 |
| All tests | `.\.venv\Scripts\python.exe -m pytest -q` |
| Headless check (no browser) | `.\.venv\Scripts\python.exe headless_demo.py` |

To stop the dashboard, press `Ctrl+C` in the terminal.

## Dashboard controls

| Control | What it does |
|---|---|
| **▶ Start** | IDLE → MOVING, only if a fresh simulated reading shows a clear path. |
| **⏸ Pause / ⏯ Resume** | Halt and continue. Resume re-checks the path. |
| **■ Stop** | End the run (STOPPED). Needs Reset before a new Start. |
| **↺ Reset** | Return to IDLE. Never starts movement. Clears a latched obstacle stop or e-stop, and a fault once recovery is complete. |
| **🛑 EMERGENCY STOP** | Works in every state. Blocks all movement until Reset. *A software control, not a substitute for a physical e-stop.* |
| **Step ×1 / ×10** | Advance the simulation 0.1 s / 1.0 s. |
| **Auto-run (live)** (sidebar) | Advances 0.5 s every 0.5 s. It only advances time and never starts movement. |
| Speed, turn rate (sidebar) | Simulated motion settings. |
| Stop threshold (sidebar) | Simulated stopping distance (default 0.50 m, not physically validated). |
| Sensor fault injection (sidebar) | No echo, NaN, infinity, negative, above max, below min. |
| Noise (sidebar) | Seeded Gaussian noise. 0 by default, so runs are repeatable. |
| Load / reset scenario (sidebar) | Restores room, robot and all settings. Refused while moving, in e-stop or in fault. |
| ✏️ Edit obstacles | Table of rectangles (metres). You can add or move boxes, even while moving. |
| Event log | Timestamped log (wall clock + sim time) with CSV download. |

### Suggested expo demo script
1. **Obstacle stop.** Scenario 1 → Start → Auto-run. The robot stops at the box (⚠ OBSTACLE STOP).
2. **Latch.** Move "Box A" away in the obstacle editor. The robot *still* doesn't move. Press Resume.
3. **Emergency stop.** While moving, press 🛑. Try Start: it's rejected (see "Last action REJECTED"). Reset → IDLE.
4. **Sensor fault.** Inject "NaN" → ✖ FAULT. Reset is refused. Set fault to "No fault", let it run 3 steps, then Reset.
5. **Honest limitation.** Scenario 3 (blind spot). One narrow beam misses the post, and the simulator blocks the collision with a FAULT.

## Architecture

```
app.py                      Streamlit dashboard (display + button callbacks only)
headless_demo.py            Runs all scenarios in the terminal
src/
  config.py                 All tunable values, validated (frozen dataclasses)
  models.py                 States, commands, Pose, Obstacle, SensorReading, ...
  simulator.py              Room, ray casting, collision checks, motion (no hardware!)
  sensors/base.py           DistanceSensor interface (future real sensors implement this)
  sensors/simulated_ultrasonic.py   Ray-cast HC-SR04-style sensor + fault injection
  safety.py                 CLEAR / OBSTACLE / INVALID decision for one reading
  controller.py             State machine + interlocks (the ONLY place state changes)
  event_log.py              Bounded, timestamped log with CSV export
  scenarios.py              Resettable demo scenarios
  visualization.py          Pure SVG renderer of the room
tests/                      pytest tests (fake sensors via dependency injection)
docs/STATE_MACHINE.md       Every permitted state transition
```

Key decisions:
- **Sense → decide → act.** Each `tick()` reads the sensor, applies safety rules, and only then
  moves the robot, and only in MOVING with a CLEAR reading.
- **The sensor reports raw values and the safety module judges them.** A sensor can't
  "vouch" for itself, so None/NaN/out-of-range values can't sneak through as "clear".
- **One controller owns state.** All transitions are checked against `ALLOWED_TRANSITIONS`.
- **Streamlit safety.** All state lives in one controller in `st.session_state`. Commands come
  only from button callbacks, so page reruns can't restart motion or clear latches.
  Auto-run uses Streamlit's built-in `st.fragment(run_every=...)`, so there's no extra dependency.
- **No navigation module yet.** Path planning is Stage 4, so there are no empty placeholder files.
  Wall and obstacle collision prevention is in the simulator.

## Known limitations (be honest at the expo!)
- All numbers are simulation defaults. The HC-SR04 range (2 cm–4 m) and 15° beam are *commonly
  published nominal values*, not measured on our module. The robot radius (0.15 m) is a placeholder.
- The sensor model is simplified: 5 straight rays, perfect reflections, no cross-talk, no soft or
  angled surfaces. Real ultrasonic sensors miss things this model sees, and vice versa. Very thin
  objects can fall between rays.
- One forward-facing sensor can't see obstacles to the side or behind (Scenario 3 shows this).
- Simple stop rule: the robot can end up to one step (speed × 0.1 s = 2 cm at default speed) inside the threshold.
  Real robots also have sensor latency, braking distance and wheel slip. None of these are modelled.
- Position is known perfectly in simulation. A real robot needs localisation (encoders, IMU, etc.).
- A sensor that is "stuck" at a plausible value would not be detected by range checks alone.
- The software e-stop isn't a physical safety device. Real tests need an independent hardware cut-off.

## Project status

**Completed (verified by tests, 246 passing at the time of writing):**
- [x] Project structure, config validation, deterministic simulator (motion, ray casting, collisions)
- [x] Simulated ultrasonic sensor with range limits, seeded noise, 6 fault-injection modes
- [x] Safety assessment: threshold boundary, every invalid-value class
- [x] State machine with 7 states, enforced transition table, latched obstacle stop, e-stop, fault recovery
- [x] Streamlit dashboard: room map, controls, threshold, obstacle editor, fault injection, log, summary, scenarios
- [x] Dashboard smoke tests (Streamlit AppTest), plus a check that no hardware imports exist
- [x] GitHub Actions workflow running tests on Ubuntu and Windows (Python 3.12)

**Not started (later stages):** hardware validation (Stage 2), hardware sensor/motor backends
(Stage 3), A* navigation (Stage 4), ESP32 CSI research (Stage 5), destination commands (Stage 6).

**Blocked:** Stage 2 needs real hardware information from you (see below).

## Next milestone: Stage 2, hardware inventory and verification checklist
No wiring and no powered motor tests yet. To prepare the checklist I will need:
1. Clear photos of **all labels** on the motors, the L298N board (both sides), and the battery pack.
2. Battery: chemistry (e.g. Li-ion, NiMH, alkaline), nominal voltage, capacity, and whether it has a protection circuit/BMS.
3. A multimeter reading of the battery voltage (DC volts), with polarity noted.
4. Motor details: any part number, and whether the motors have encoders (extra wires/discs).
5. Chassis measurements: overall length and width, wheel diameter, and where the HC-SR04 could be mounted.
6. The exact HC-SR04 module marking. Its Echo pin is usually 5 V and the Raspberry Pi GPIO is
   3.3 V, so we'll design and verify a level shifter before connecting anything.
