# MobilityBot telemetry protocol, version 1 (read-only)

Direction: **device → dashboard only.** The dashboard never sends anything back, so there are
no commands and no motor control in this protocol. Implemented and validated in
`src/telemetry.py` (`parse_message`, `SerialTelemetryProvider`).

## Framing
- Transport: a serial port (for example the Arduino Uno's USB serial port). Baud rate is a
  software setting that must match the firmware; the default is **115200**.
- One message per line: a UTF-8 JSON object followed by `\n` (`\r\n` is accepted too).
- Maximum line length: **512 bytes**. Longer input is rejected and skipped up to the next newline.
- Blank lines are ignored.

## Message fields (all required)

| Field | Type | Meaning | Rules |
|---|---|---|---|
| `v` | integer | Protocol version | Must be `1`. Any other version is rejected. |
| `seq` | integer | Message counter | 0 … 4294967295. Should increase by 1 per message. |
| `t_ms` | integer | Device uptime in ms (e.g. Arduino `millis()`) | 0 … 4294967295. Informational only; the device clock is not trusted for freshness. |
| `dist_m` | number or `null` | Distance in metres | Finite. `null` when there is no reading. |
| `dist_valid` | boolean | Device's own view of the reading | `true` requires a non-null `dist_m`. |
| `status` | string | Controller status *reported by the device* | One of `IDLE`, `MOVING`, `PAUSED`, `OBSTACLE_DETECTED`, `STOPPED`, `EMERGENCY_STOP`, `FAULT`, `BOOTING`, `UNKNOWN`. |
| `link` | string | Device's own communication health | `OK`, `DEGRADED` or `ERROR`. |

Unknown extra fields are ignored, so later versions can add fields.

### Examples
```json
{"v":1,"seq":42,"t_ms":120345,"dist_m":0.873,"dist_valid":true,"status":"IDLE","link":"OK"}
{"v":1,"seq":43,"t_ms":120395,"dist_m":null,"dist_valid":false,"status":"IDLE","link":"OK"}
```
The second line is a *valid message* that reports an *invalid reading* (for example no echo).

## How the dashboard treats data

| Situation | What the dashboard does |
|---|---|
| Invalid JSON, wrong types, missing fields, `NaN`/`Infinity`/overflowing numbers, unknown `status`/`link`, wrong version, not UTF-8, over-long line | The whole message is **rejected** and counted. Nothing from it is used. |
| Duplicate `seq` | Rejected. |
| `seq` jumps forward | Accepted; the gap is counted as "missed". |
| `seq` goes backwards | Accepted and logged as "device probably reset". Opening the Uno's port usually resets it. |
| `dist_valid: false` | Message accepted, but sensor shown as **INVALID** with no distance. |
| `dist_valid: true` but distance outside the host's valid range (0.02–4.00 m) | Sensor shown as **INVALID** ("device claimed valid, host rejected it"). No distance. |
| No accepted message for more than **1.0 s** | **STALE**: no distance is shown and the old value must not be trusted. Rejected messages don't count as fresh. |
| Port read error or unplugged | **RECONNECTING**: old data is discarded, the port is closed, and it retries after 0.5 s, 1 s, 2 s, 4 s, then every 5 s. |
| First Connect fails (wrong port, port busy, pyserial missing) | **ERROR**: no automatic retries; the user must press Connect again. |
| User presses Disconnect, or switches back to Simulation | The port is closed and no more reconnects happen. |

A distance is displayed **only** when all of these are true: the port is open, the latest
accepted message is at most 1.0 s old, the device says the reading is valid, and the host's range
check passes. "✔ RECEIVING VALID TELEMETRY" is shown only when a valid protocol message has
actually arrived within that time.

The device-reported `status` is shown as reported, with a note that the dashboard has not
verified it. It is not used for any decision.
