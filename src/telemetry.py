"""Read-only telemetry: message schema, validation and data providers.

Two providers produce the same ``TelemetrySnapshot`` for the dashboard:

* ``SimulationTelemetryProvider``: wraps the simulated robot. Source is SIMULATED.
* ``SerialTelemetryProvider``: parses JSON-lines telemetry from a byte stream
  (a serial port in real use, a fake stream in tests). Source is HARDWARE.

Rules this module enforces:

* It never sends anything to a device. Telemetry is strictly read-only.
* It never imports serial or GPIO libraries. The real port is opened through
  an injected ``opener`` (see ``src/hardware/serial_port.py``) and only when
  the user explicitly connects.
* Missing, stale, malformed or disconnected data never produces a distance
  value. ``TelemetrySnapshot.distance_m`` is set only for a fresh message whose
  reading the device marks valid AND the host re-validates.
* Simulated values are never used as hardware values.

Protocol: see docs/TELEMETRY_PROTOCOL.md.
"""

from __future__ import annotations

import json
import math
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from enum import StrEnum
from typing import Protocol

from src.config import SensorConfig, TelemetryConfig
from src.event_log import EventLog, Level
from src.models import RobotState, SensorReading, Verdict
from src.safety import assess_reading

PROTOCOL_VERSION = 1
MAX_SEQ = 2**32 - 1
REQUIRED_FIELDS = ("v", "seq", "t_ms", "dist_m", "dist_valid", "status", "link")
REPORTED_STATUSES = frozenset({s.value for s in RobotState} | {"BOOTING", "UNKNOWN"})
DEVICE_LINK_VALUES = frozenset({"OK", "DEGRADED", "ERROR"})

SIMULATED_SOURCE = "SIMULATED"
HARDWARE_SOURCE = "HARDWARE (serial telemetry, read-only)"


# ------------------------------------------------------------------- schema

class TelemetryParseError(ValueError):
    """A line that is not a valid protocol message. It is rejected as a whole."""


@dataclass(frozen=True)
class TelemetryMessage:
    version: int
    seq: int
    device_time_ms: int
    distance_m: float | None
    distance_valid: bool  # what the DEVICE claims; the host re-checks it
    status: str           # controller status reported by the device (not verified by the host)
    link: str             # device's own view of its communication health
    received_at: float    # host monotonic clock (seconds)
    received_wall: datetime


def _reject_constant(name: str) -> float:
    raise TelemetryParseError(f"non-finite number {name} is not allowed")


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def parse_message(raw: bytes | str, received_at: float, received_wall: datetime,
                  max_line_bytes: int = 512) -> TelemetryMessage:
    """Parse and validate one line. Raises TelemetryParseError with a human-readable reason."""
    if isinstance(raw, str):
        raw = raw.encode("utf-8")
    raw = raw.strip()
    if not raw:
        raise TelemetryParseError("empty line")
    if len(raw) > max_line_bytes:
        raise TelemetryParseError(f"line longer than {max_line_bytes} bytes")
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        raise TelemetryParseError("not valid UTF-8") from None
    try:
        data = json.loads(text, parse_constant=_reject_constant)
    except TelemetryParseError:
        raise
    except ValueError as exc:
        raise TelemetryParseError(f"invalid JSON ({exc.msg})") from None
    if not isinstance(data, dict):
        raise TelemetryParseError("message must be a JSON object")

    missing = [f for f in REQUIRED_FIELDS if f not in data]
    if missing:
        raise TelemetryParseError(f"missing field(s): {', '.join(missing)}")

    version = data["v"]
    if not _is_int(version):
        raise TelemetryParseError("'v' must be an integer")
    if version != PROTOCOL_VERSION:
        raise TelemetryParseError(f"unsupported protocol version {version} (expected {PROTOCOL_VERSION})")

    seq, t_ms = data["seq"], data["t_ms"]
    if not _is_int(seq) or not 0 <= seq <= MAX_SEQ:
        raise TelemetryParseError(f"'seq' must be an integer 0..{MAX_SEQ}")
    if not _is_int(t_ms) or not 0 <= t_ms <= MAX_SEQ:
        raise TelemetryParseError(f"'t_ms' must be an integer 0..{MAX_SEQ}")

    dist_valid = data["dist_valid"]
    if not isinstance(dist_valid, bool):
        raise TelemetryParseError("'dist_valid' must be true or false")
    dist = data["dist_m"]
    if dist is not None:
        if isinstance(dist, bool) or not isinstance(dist, (int, float)):
            raise TelemetryParseError("'dist_m' must be a number or null")
        dist = float(dist)
        if not math.isfinite(dist):  # e.g. 1e400 parses as infinity
            raise TelemetryParseError("'dist_m' must be finite")
    if dist_valid and dist is None:
        raise TelemetryParseError("'dist_valid' is true but 'dist_m' is null")

    status, link = data["status"], data["link"]
    if not isinstance(status, str) or status not in REPORTED_STATUSES:
        raise TelemetryParseError(f"unknown 'status' {status!r}")
    if not isinstance(link, str) or link not in DEVICE_LINK_VALUES:
        raise TelemetryParseError(f"unknown 'link' {link!r}")

    return TelemetryMessage(version, seq, t_ms, dist, dist_valid, status, link, received_at, received_wall)


# ----------------------------------------------------------------- snapshot

class ConnectionState(StrEnum):
    SIMULATED = "SIMULATED"                # no hardware involved at all
    DISCONNECTED = "DISCONNECTED"          # not connected (user has not connected, or disconnected)
    ERROR = "ERROR"                        # could not open the port
    WAITING_FOR_DATA = "WAITING_FOR_DATA"  # port open, no valid message yet
    CONNECTED = "CONNECTED"                # port open and a valid message arrived recently
    STALE = "STALE"                        # port open but no valid message for too long
    RECONNECTING = "RECONNECTING"          # link dropped; retrying with back-off


class SensorValidity(StrEnum):
    VALID = "VALID"
    INVALID = "INVALID"
    STALE = "STALE"
    NO_DATA = "NO_DATA"


@dataclass(frozen=True)
class TelemetrySnapshot:
    source: str
    connection: ConnectionState
    sensor_validity: SensorValidity
    distance_m: float | None  # only ever set when sensor_validity is VALID
    validity_reason: str
    reported_status: str | None = None
    device_link: str | None = None
    last_message_wall: datetime | None = None
    last_message_age_s: float | None = None
    seq: int | None = None
    messages_ok: int = 0
    messages_rejected: int = 0
    messages_missed: int = 0
    reconnects: int = 0
    last_error: str | None = None

    @property
    def hardware_verified(self) -> bool:
        """True only for real hardware with a fresh, valid message. Never for simulation."""
        return self.source == HARDWARE_SOURCE and self.connection is ConnectionState.CONNECTED


class TelemetryProvider(ABC):
    @abstractmethod
    def poll(self) -> None:
        """Take in any new data. Must never block for long and never write to a device."""

    @abstractmethod
    def snapshot(self) -> TelemetrySnapshot:
        """Current view for the dashboard."""


# --------------------------------------------------------------- simulation

class SimulationTelemetryProvider(TelemetryProvider):
    """Presents the simulated robot through the same interface. Always labelled SIMULATED."""

    def __init__(self, controller_getter: Callable[[], object]) -> None:
        self._get = controller_getter

    def poll(self) -> None:
        pass  # the simulation advances through the controller, not here

    def snapshot(self) -> TelemetrySnapshot:
        c = self._get()
        a = c.last_assessment
        if a is None:
            return TelemetrySnapshot(SIMULATED_SOURCE, ConnectionState.SIMULATED, SensorValidity.NO_DATA, None,
                                     "No simulated reading yet", reported_status=c.state.value)
        valid = a.verdict is not Verdict.INVALID
        return TelemetrySnapshot(
            SIMULATED_SOURCE, ConnectionState.SIMULATED,
            SensorValidity.VALID if valid else SensorValidity.INVALID,
            a.distance_m if valid else None, a.reason, reported_status=c.state.value,
        )


# ------------------------------------------------------------------ hardware

class ByteStream(Protocol):
    """Minimal read-only stream. Deliberately has no write method."""

    def read_available(self) -> bytes: ...

    def close(self) -> None: ...


Opener = Callable[[], ByteStream]


class SerialTelemetryProvider(TelemetryProvider):
    """Turns a byte stream of JSON lines into validated, freshness-checked snapshots."""

    def __init__(
        self,
        opener: Opener,
        config: TelemetryConfig,
        sensor_config: SensorConfig,
        port_label: str,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], datetime] | None = None,
        log: EventLog | None = None,
    ) -> None:
        self._opener = opener
        self.config = config
        self.sensor_config = sensor_config
        self.port_label = port_label
        self._clock = clock
        self._wall = wall_clock or (lambda: datetime.now().astimezone())
        self.log = log or EventLog(clock=self._wall, max_entries=500)
        self._started = clock()

        self._stream: ByteStream | None = None
        self._wanted = False  # True only between an explicit connect() and disconnect()
        self._buffer = b""
        self._discarding = False  # inside an over-long line; drop bytes until the next newline
        self._next_retry_at = 0.0
        self._retry_delay = config.reconnect_initial_s
        self._state = ConnectionState.DISCONNECTED
        self._last: TelemetryMessage | None = None
        self.last_error: str | None = None
        self.messages_ok = 0
        self.messages_rejected = 0
        self.messages_missed = 0
        self.reconnects = 0

    # ---- user actions

    def connect(self) -> None:
        """Explicit, user-initiated connection attempt."""
        self._wanted = True
        self._retry_delay = self.config.reconnect_initial_s
        self._open(reconnect=False)

    def disconnect(self) -> None:
        self._wanted = False
        self._close()
        self._last = None
        self._state = ConnectionState.DISCONNECTED
        self._log(Level.INFO, f"Disconnected from {self.port_label} by user")

    # ---- polling

    def poll(self) -> None:
        if not self._wanted:
            return
        now = self._clock()
        if self._stream is None:
            if now >= self._next_retry_at:
                self._open(reconnect=True)
            return
        try:
            data = self._stream.read_available()
        except Exception as exc:  # serial unplugged, driver error, ...
            self._lost(f"read failed: {exc}")
            return
        if data:
            self._feed(data)
        self._refresh_state()

    def _feed(self, data: bytes) -> None:
        self._buffer += data
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                break
            line, self._buffer = self._buffer[:newline], self._buffer[newline + 1:]
            if self._discarding:
                self._discarding = False  # end of an over-long line that was already rejected
                continue
            self._process_line(line)
        if len(self._buffer) > self.config.max_line_bytes:
            if not self._discarding:
                self._reject(f"line longer than {self.config.max_line_bytes} bytes")
            self._buffer = b""
            self._discarding = True

    def _process_line(self, line: bytes) -> None:
        if not line.strip():
            return
        try:
            msg = parse_message(line, self._clock(), self._wall(), self.config.max_line_bytes)
        except TelemetryParseError as exc:
            self._reject(str(exc))
            return
        last = self._last
        if last is not None:
            if msg.seq == last.seq:
                self._reject(f"duplicate sequence number {msg.seq}")
                return
            if msg.seq < last.seq:
                self._log(Level.WARNING, f"Sequence restarted ({last.seq} -> {msg.seq}); device probably reset")
            elif msg.seq > last.seq + 1:
                self.messages_missed += msg.seq - last.seq - 1
        if msg.link != "OK" and (last is None or last.link != msg.link):
            self._log(Level.WARNING, f"Device reports link health {msg.link}")
        if last is None:
            self._log(Level.INFO, f"First valid telemetry message (seq {msg.seq})")
        self._last = msg
        self.messages_ok += 1

    def _refresh_state(self) -> None:
        if self._stream is None:
            return
        previous = self._state
        if self._last is None:
            self._state = ConnectionState.WAITING_FOR_DATA
        elif self._clock() - self._last.received_at > self.config.stale_after_s:
            self._state = ConnectionState.STALE
        else:
            self._state = ConnectionState.CONNECTED
        if self._state is not previous:
            level = Level.WARNING if self._state is ConnectionState.STALE else Level.INFO
            self._log(level, f"Link state {previous} -> {self._state}")

    # ---- open / close / reconnect

    def _open(self, reconnect: bool) -> None:
        self._close()
        try:
            stream = self._opener()
        except Exception as exc:
            self.last_error = str(exc) or exc.__class__.__name__
            self._state = ConnectionState.RECONNECTING if reconnect else ConnectionState.ERROR
            self._schedule_retry()
            self._log(Level.WARNING, f"Could not open {self.port_label}: {self.last_error}")
            if not reconnect:
                # A failed first attempt is not retried silently; the user must press Connect again.
                self._wanted = False
            return
        self._stream = stream
        self._buffer = b""
        self._discarding = False  # a partial first line (joined mid-stream) is simply rejected and counted
        self._last = None
        self._retry_delay = self.config.reconnect_initial_s
        if reconnect:
            self.reconnects += 1
        self._state = ConnectionState.WAITING_FOR_DATA
        self._log(Level.INFO, f"{'Reconnected to' if reconnect else 'Opened'} {self.port_label} (read-only)")

    def _lost(self, reason: str) -> None:
        self.last_error = reason
        self._close()
        self._last = None  # old data must not survive a disconnect
        self._state = ConnectionState.RECONNECTING
        self._schedule_retry()
        self._log(Level.CRITICAL, f"Telemetry link lost: {reason}. Retrying.")

    def _schedule_retry(self) -> None:
        self._next_retry_at = self._clock() + self._retry_delay
        self._retry_delay = min(self._retry_delay * 2, self.config.reconnect_max_s)

    def _close(self) -> None:
        if self._stream is not None:
            try:
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    def _reject(self, reason: str) -> None:
        self.messages_rejected += 1
        self.last_error = f"rejected message: {reason}"
        self._log(Level.WARNING, f"Rejected telemetry: {reason}")

    def _log(self, level: Level, message: str) -> None:
        self.log.add(level, "TELEMETRY", message, self._clock() - self._started)

    # ---- snapshot

    @property
    def state(self) -> ConnectionState:
        return self._state

    @property
    def is_active(self) -> bool:
        """True between an explicit connect() and disconnect() (or a failed first attempt)."""
        return self._wanted

    def snapshot(self) -> TelemetrySnapshot:
        self._refresh_state()
        msg = self._last
        common = dict(
            source=HARDWARE_SOURCE, connection=self._state, messages_ok=self.messages_ok,
            messages_rejected=self.messages_rejected, messages_missed=self.messages_missed,
            reconnects=self.reconnects, last_error=self.last_error,
        )
        if msg is None or self._stream is None:
            reason = {
                ConnectionState.DISCONNECTED: "Not connected",
                ConnectionState.ERROR: "Could not open the port",
                ConnectionState.RECONNECTING: "Link lost - no data",
                ConnectionState.WAITING_FOR_DATA: "Port open, waiting for the first valid message",
            }.get(self._state, "No data")
            return TelemetrySnapshot(sensor_validity=SensorValidity.NO_DATA, distance_m=None,
                                     validity_reason=reason, **common)

        age = self._clock() - msg.received_at
        details = dict(reported_status=msg.status, device_link=msg.link, last_message_wall=msg.received_wall,
                       last_message_age_s=age, seq=msg.seq)
        if self._state is ConnectionState.STALE:
            return TelemetrySnapshot(sensor_validity=SensorValidity.STALE, distance_m=None,
                                     validity_reason=f"No valid message for {age:.1f} s - last reading is not trusted",
                                     **common, **details)
        if not msg.distance_valid:
            return TelemetrySnapshot(sensor_validity=SensorValidity.INVALID, distance_m=None,
                                     validity_reason="Device reports the reading as invalid (e.g. no echo)",
                                     **common, **details)
        # The device says valid; the host applies the same range checks as the simulator.
        reading = SensorReading(msg.distance_m, "HARDWARE", "serial telemetry", 0.0)
        check = assess_reading(reading, threshold_m=self.sensor_config.max_range_m, sensor=self.sensor_config)
        if check.verdict is Verdict.INVALID:
            return TelemetrySnapshot(sensor_validity=SensorValidity.INVALID, distance_m=None,
                                     validity_reason=f"Device claimed valid, host rejected it: {check.reason}",
                                     **common, **details)
        return TelemetrySnapshot(sensor_validity=SensorValidity.VALID, distance_m=msg.distance_m,
                                 validity_reason="Fresh message, device and host range checks passed",
                                 **common, **details)
