# MobilityBot 🤖

A low-budget autonomous mobility **prototype** for a school exhibition. It is a small,
**unoccupied** wheeled robot that will eventually show obstacle stops, simple navigation,
a live dashboard, and some sensing experiments.

> ⚠️ **This is not a wheelchair or mobility aid, and it must never carry a person.**
> Everything in this repository is currently **simulation only**. Simulated results
> do not prove real-world safety.

## Current stage: simulation ✅ + simulated navigation ✅ + read-only hardware telemetry (software side) ✅

A deterministic 2-D simulator, a simulated HC-SR04-style ultrasonic sensor, a safety
state machine, A* route planning on an occupancy grid, a Streamlit dashboard, and automated
tests.

The dashboard has two modes:
- **Simulation** (default).
- **Hardware telemetry (read-only)**: it can *display* validated sensor messages from a serial
  port.

There is **no motor control and no GPIO code**. Nothing is sent to any device. The serial library
is loaded only when you press Connect in hardware mode. **No physical hardware has been connected
or tested yet**: hardware mode has only been tested against fake streams and pyserial's virtual
`loop://` port.

Navigation was done in simulation before hardware validation (Stage 2) on purpose. It needs
no hardware and doesn't change the safety rules. Stage 2 is still blocked on hardware
information.

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

**Optional, only for reading a real serial port:** `pyserial`. The simulator and tests work without it.
```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-hardware.txt
```

## Run

| What | Command (PowerShell, from the project folder) |
|---|---|
| Dashboard | `.\.venv\Scripts\python.exe -m streamlit run app.py` → opens http://localhost:8501 |
| All tests | `.\.venv\Scripts\python.exe -m pytest -q` |
| Navigation tests only | `.\.venv\Scripts\python.exe -m pytest -q tests/test_navigation.py` |
| Headless check (no browser) | `.\.venv\Scripts\python.exe headless_demo.py` |
| Telemetry tests only | `.\.venv\Scripts\python.exe -m pytest -q tests/test_telemetry.py` |

To stop the dashboard, press `Ctrl+C` in the terminal.

## Dashboard modes
Pick the mode at the top of the sidebar:
- **Simulation** (default): everything below. Every value is labelled SIMULATED.
- **Hardware telemetry (read-only)**: a separate page. It has no room map, no simulated values,
  and no Start/Stop/motion buttons. See [Hardware telemetry](#hardware-telemetry-read-only).

## Dashboard controls (Simulation mode)

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
| Destination (sidebar) | Kitchen, Living Room, Bedroom, Expo Goal, or **Custom goal** (x, y in metres). |
| **🧭 Plan route** (sidebar) | A* route from the robot's position. Only while IDLE. **Does not start motion.** Press Start to drive it. |
| **✖ Clear route** (sidebar) | Back to manual straight driving. Not allowed while moving. |
| Show planning grid (sidebar) | Grey band = cells where the robot *centre* may not go (robot radius + 5 cm margin). |
| ✏️ Edit obstacles | Table of rectangles (metres). You can add or move boxes, even while moving. If a route is active it is re-planned, or the robot stops if no route remains. |
| Event log | Timestamped log (wall clock + sim time) with CSV download. |

### Suggested expo demo script
1. **Obstacle stop.** Scenario 1 → Start → Auto-run. The robot stops at the box (⚠ OBSTACLE STOP).
2. **Latch.** Move "Box A" away in the obstacle editor. The robot *still* doesn't move. Press Resume.
3. **Emergency stop.** While moving, press 🛑. Try Start: it's rejected (see "Last action REJECTED"). Reset → IDLE.
4. **Sensor fault.** Inject "NaN" → ✖ FAULT. Reset is refused. Set fault to "No fault", let it run 3 steps, then Reset.
5. **Honest limitation.** Scenario 3 (blind spot). One narrow beam misses the post, and the simulator blocks the collision with a FAULT.
6. **Navigation.** Load scenario 1, pick "Kitchen", Plan route (purple line with S and G), Start, Auto-run.
   Press 🛑 mid-route: the robot stops and stays stopped. Reset, then Start continues to the Kitchen.
7. **Re-planning.** While driving to the Kitchen, add a box on the purple line in the obstacle editor.
   The route is re-planned around it.

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
  telemetry.py              Telemetry schema + validation, simulation & serial providers (read-only)
  hardware/serial_port.py   Lazy pyserial adapter, opened only on Connect, no write method
  controller.py             State machine + interlocks (the ONLY place state changes)
  event_log.py              Bounded, timestamped log with CSV export
  navigation.py             Occupancy grid, A* planner, route follower (requests motion only)
  scenarios.py              Resettable demo scenarios + predefined destinations
  visualization.py          Pure SVG renderer of the room
tests/                      pytest tests (fake sensors via dependency injection)
docs/STATE_MACHINE.md       Every permitted state transition
docs/TELEMETRY_PROTOCOL.md  Telemetry message format and how bad/stale data is handled
requirements-hardware.txt   Optional pyserial (only for a real serial port)
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
- **Wall and obstacle collision prevention lives in the simulator**, and navigation reuses the same test.

## Navigation (simulation only)

**Occupancy grid** (`build_occupancy_grid`). The 4 m × 3 m room becomes 80 × 60 cells of 0.05 m
(`NavigationConfig.resolution_m`), using the same coordinates as the simulator: origin bottom-left,
x east, y north. A cell is **blocked** if a circle of radius *robot radius + safety margin*
(0.15 + 0.05 = 0.20 m) at the cell centre would overlap a wall or obstacle. The grid calls the
simulator's own collision function, so the two can't disagree.

**A\* planner** (`plan_path`):
- 8-connected moves and an octile heuristic.
- Diagonal moves can't cut the corner of a blocked cell.
- Ties are broken deterministically, so the same input always gives the same route.
- A clearance cost makes cells close to obstacles more expensive, so routes keep away from
  surfaces where there is room.
- Results are explicit: `OK`, `INVALID_START` (robot inside a no-go zone), `INVALID_GOAL`
  (outside the room, not a number, inside an obstacle, or too close to a wall or obstacle), or
  `UNREACHABLE` (cut off, or every gap is narrower than the inflated footprint).
- The waypoints are the cell centres where the route changes direction, ending exactly at the goal.

Why the margin must be at least half a cell diagonal (validated in config): every straight segment
between free cell centres then stays at least the robot radius away from every surface.

**Following a route.** The `Navigator` turns toward the next waypoint, on the spot if the heading
error is over 20°, then drives at up to the speed-slider value. It only **requests** motion:
- The controller clamps the request to the speed and turn limits.
- Every step still goes through the usual checks: sensor reading → obstacle latch → Emergency
  Stop / FAULT → collision check.
- Reaching the goal, or the route becoming blocked, ends the run with the existing
  `MOVING → STOPPED` transition. No new states or transitions were added.
- Planning never starts motion.
- If obstacles change, the route is re-planned from the current position. If no route remains,
  the robot stops.

## Hardware telemetry (read-only)

Message format: **[docs/TELEMETRY_PROTOCOL.md](docs/TELEMETRY_PROTOCOL.md)**. One JSON object per
line, versioned (`"v": 1`), with sequence number, distance in metres, validity, device-reported
status and link health. Every message is validated, and a distance is shown **only** when a fresh
(≤ 1 s), valid message passes the host's range check. Stale, missing, malformed or disconnected
data is never shown as a reading.

How it's built:
- `src/telemetry.py`: the schema and parser, plus two providers (`SimulationTelemetryProvider`,
  `SerialTelemetryProvider`) that produce the same `TelemetrySnapshot`.
  - The serial reader is polled (no background threads) and reads from an injected stream, so
    tests use fake streams.
  - It handles split and over-long lines, rejects bad messages, detects stale data, and
    reconnects with back-off after an unplug.
- `src/hardware/serial_port.py`: the **only** file allowed to import pyserial, and only inside a
  function that runs when you press Connect. Its stream wrapper has no write method.
- Tests enforce these rules:
  - No GPIO or serial imports anywhere else.
  - No module-level pyserial import.
  - No `.write()`/`.send()` calls in the telemetry code or the dashboard.
  - Starting the app, even in hardware mode, doesn't load `serial` (checked in a subprocess).

### Windows: development and testing
1. **No hardware needed for tests:** `.\.venv\Scripts\python.exe -m pytest -q`. Without pyserial,
   2 adapter tests are skipped. Install `requirements-hardware.txt` to run them too; they use
   pyserial's virtual `loop://` port, not a device.
2. **Try the hardware page without hardware:** start the dashboard, choose *Hardware telemetry*,
   enter `loop://` as the port and press Connect. You should see "PORT OPEN - WAITING FOR VALID
   DATA" and no distance (nothing sends data on a loopback). A made-up name like `COM99` should show
   "CONNECTION ERROR".
3. **Finding a real port later:** Device Manager → *Ports (COM & LPT)*, or the *List serial ports*
   button. Close the Arduino IDE's Serial Monitor first, because only one program can open a COM
   port at a time.
4. Opening an Arduino Uno's USB serial port normally **resets the board**. That's expected; the
   dashboard logs "sequence restarted".

Before plugging anything in, the electrical checks in the next milestone still apply. Reading
telemetry over USB doesn't need the motors, the L298N or the battery to be connected.

## Known limitations (be honest at the expo!)
- All numbers are simulation defaults. The HC-SR04 range (2 cm–4 m) and 15° beam are *commonly
  published nominal values*, not measured on our module. The robot radius (0.15 m) is a placeholder.
- The sensor model is simplified: 5 straight rays, perfect reflections, no cross-talk, no soft or
  angled surfaces. Real ultrasonic sensors miss things this model sees, and vice versa. Very thin
  objects can fall between rays.
- One forward-facing sensor can't see obstacles to the side or behind (Scenario 3 shows this).
- Simple stop rule: driving straight, the robot can end up to one step (speed × 0.1 s = 2 cm at default speed)
  inside the threshold. While turning, the beam sweeps onto new surfaces, so it can end slightly further
  inside (2.3 cm was observed in testing). Tests check that it never collides, not an exact margin.
  Real robots also have sensor latency, braking distance and wheel slip. None of these are modelled.
- Position is known perfectly in simulation. A real robot needs localisation (encoders, IMU, etc.).
- A sensor that is "stuck" at a plausible value would not be detected by range checks alone.
- **Navigation limits:**
  - Routes are planned on a perfectly known, static, simulated map with perfect position. A route
    being "collision-free on the grid" does **not** mean a real robot would drive it without hitting
    anything. That would need real localisation (wheel encoders, IMU and/or markers), a
    calibrated map, and far better obstacle sensing than one forward ultrasonic sensor.
  - The robot follows the route approximately (it curves slightly when correcting heading). The
    simulator's collision check still guards every step, and tests confirm the demo routes never collide.
  - Routes look stair-stepped (45° and 90° moves only), and the planner doesn't consider which way
    the robot ends up facing.
  - The forward-sensor stop rule still applies during navigation. A goal close to a wall can end in
    a (correct, safe) obstacle stop before arrival. Example: (3.5, 1.5) is a valid goal, but driving
    straight at it puts the wall within 0.5 m first. That stop is latched; if the robot is left facing
    the obstacle, Resume stays blocked until the scenario is reset.
  - The four destinations are named demo positions. They aren't text or voice commands yet (Stage 6).
- The software e-stop isn't a physical safety device. Real tests need an independent hardware cut-off.
- **Telemetry limits:**
  - No device firmware exists yet, so hardware mode has never received real data.
  - The device-reported `status` is only displayed, never verified or acted on.
  - Freshness uses the time each message *arrives* at the PC, so a device sending old buffered
    readings would look fresh. The sequence number only detects gaps and repeats.
  - There's no checksum: corrupted bytes that still form valid JSON would be accepted.
  - Hardware mode is display-only; it doesn't feed the safety state machine.

## Project status

**Completed (verified by tests, 423 passing at the time of writing with pyserial installed; 421 + 2 skipped without it):**
- [x] Project structure, config validation, deterministic simulator (motion, ray casting, collisions)
- [x] Simulated ultrasonic sensor with range limits, seeded noise, 6 fault-injection modes
- [x] Safety assessment: threshold boundary, every invalid-value class
- [x] State machine with 7 states, enforced transition table, latched obstacle stop, e-stop, fault recovery
- [x] Streamlit dashboard: room map, controls, threshold, obstacle editor, fault injection, log, summary, scenarios
- [x] Dashboard smoke tests (Streamlit AppTest), including auto-run never starting motion, plus a check that no hardware imports exist
- [x] GitHub Actions workflow running tests on Ubuntu and Windows (Python 3.12)
- [x] Simulated navigation: occupancy grid with inflation, A* with invalid/unreachable handling,
      route following through the controller, re-planning on obstacle changes, dashboard route display,
      predefined and custom goals (tests in `tests/test_navigation.py` and `tests/test_dashboard.py`)
- [x] Read-only telemetry: versioned schema, validation, staleness, disconnect/reconnect, Simulation vs
      Hardware dashboard modes (tests with fake streams in `tests/test_telemetry.py`, `tests/test_dashboard.py`).
      **Not yet tested with real hardware.**

**Not started (later stages):** hardware validation (Stage 2), device firmware that sends telemetry,
motor backends (Stage 3), real-world localisation for navigation (Stage 4, hardware part), ESP32 CSI research
(Stage 5), text/voice destination commands (Stage 6).

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
