"""Telemetry schema, serial reader and providers. Uses fake streams only: no hardware."""

import json
import sys

import pytest

from src.config import SensorConfig, TelemetryConfig
from src.hardware.serial_port import SerialUnavailableError, make_serial_opener
from src.models import SensorFaultMode
from src.scenarios import build_controller
from src.telemetry import (
    HARDWARE_SOURCE,
    MAX_SEQ,
    SIMULATED_SOURCE,
    ConnectionState,
    SensorValidity,
    SerialTelemetryProvider,
    SimulationTelemetryProvider,
    TelemetryParseError,
    parse_message,
)
from tests.helpers import FakeSerial, fixed_clock

CFG = TelemetryConfig()  # stale after 1.0 s, reconnect 0.5 s doubling to 5 s


def msg(seq=1, dist=0.85, valid=True, status="IDLE", link="OK", **extra) -> dict:
    data = {"v": 1, "seq": seq, "t_ms": 1000 + seq, "dist_m": dist, "dist_valid": valid,
            "status": status, "link": link}
    data.update(extra)
    return data


def line(**kwargs) -> bytes:
    return json.dumps(msg(**kwargs)).encode() + b"\n"


def parse(raw):
    return parse_message(raw, 0.0, fixed_clock())


# ---------------------------------------------------------------- fakes

class Clock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


class Opener:
    def __init__(self, *outcomes) -> None:
        self.outcomes = list(outcomes)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        outcome = self.outcomes.pop(0) if self.outcomes else OSError("no more fake ports")
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


def provider(*outcomes, config=CFG):
    clock = Clock()
    opener = Opener(*outcomes)
    p = SerialTelemetryProvider(opener, config, SensorConfig(), "FAKE1", clock=clock, wall_clock=fixed_clock)
    return p, opener, clock


def connected(*chunks):
    stream = FakeSerial(*chunks)
    p, opener, clock = provider(stream)
    p.connect()
    p.poll()
    return p, stream, clock, opener


# --------------------------------------------------------------- schema

def test_valid_message_parses():
    m = parse(json.dumps(msg(seq=7, dist=1.25, status="MOVING")))
    assert (m.version, m.seq, m.distance_m, m.distance_valid, m.status, m.link) == (1, 7, 1.25, True, "MOVING", "OK")
    assert m.device_time_ms == 1007


def test_extra_fields_are_ignored_for_forward_compatibility():
    assert parse(json.dumps(msg(battery_note="ignored"))).seq == 1


def test_invalid_reading_with_null_distance_is_a_valid_message():
    m = parse(json.dumps(msg(dist=None, valid=False)))
    assert m.distance_m is None and not m.distance_valid


@pytest.mark.parametrize("field", ["v", "seq", "t_ms", "dist_m", "dist_valid", "status", "link"])
def test_missing_field_rejected(field):
    data = msg()
    del data[field]
    with pytest.raises(TelemetryParseError, match=field):
        parse(json.dumps(data))


@pytest.mark.parametrize(
    "changes",
    [
        {"v": 2}, {"v": "1"}, {"v": True}, {"v": 1.0},
        {"seq": -1}, {"seq": MAX_SEQ + 1}, {"seq": 1.5}, {"seq": True}, {"seq": "1"},
        {"t_ms": -5}, {"t_ms": None},
        {"dist_m": "0.5"}, {"dist_m": True}, {"dist_m": [0.5]},
        {"dist_valid": "true"}, {"dist_valid": 1}, {"dist_valid": None},
        {"dist_m": None, "dist_valid": True},
        {"status": "DANCING"}, {"status": "idle"}, {"status": 3},
        {"link": "GOOD"}, {"link": None},
    ],
)
def test_bad_field_values_rejected(changes):
    data = msg()
    data.update(changes)
    with pytest.raises(TelemetryParseError):
        parse(json.dumps(data))


@pytest.mark.parametrize(
    "raw",
    [b"", b"   ", b"not json", b"{", b"[1, 2]", b"42", b'"text"', b"\xff\xfe\x00",
     b'{"v":1,"seq":1,"t_ms":1,"dist_m":NaN,"dist_valid":true,"status":"IDLE","link":"OK"}',
     b'{"v":1,"seq":1,"t_ms":1,"dist_m":Infinity,"dist_valid":true,"status":"IDLE","link":"OK"}',
     b'{"v":1,"seq":1,"t_ms":1,"dist_m":1e400,"dist_valid":true,"status":"IDLE","link":"OK"}',
     b"x" * 600],
)
def test_malformed_lines_rejected(raw):
    with pytest.raises(TelemetryParseError):
        parse(raw)


# --------------------------------------------------------- serial reader

def test_nothing_is_opened_until_connect():
    p, opener, clock = provider(FakeSerial())
    for _ in range(5):
        p.poll()
        clock.t += 10
    assert opener.calls == 0
    snap = p.snapshot()
    assert snap.connection is ConnectionState.DISCONNECTED
    assert snap.sensor_validity is SensorValidity.NO_DATA and snap.distance_m is None
    assert not snap.hardware_verified


def test_connected_but_no_data_shows_no_reading():
    p, _, _, _ = connected()
    snap = p.snapshot()
    assert snap.connection is ConnectionState.WAITING_FOR_DATA
    assert snap.distance_m is None and not snap.hardware_verified


def test_valid_message_gives_verified_reading():
    p, _, _, _ = connected(line(seq=1, dist=0.85))
    snap = p.snapshot()
    assert snap.source == HARDWARE_SOURCE
    assert snap.connection is ConnectionState.CONNECTED and snap.hardware_verified
    assert snap.sensor_validity is SensorValidity.VALID and snap.distance_m == 0.85
    assert snap.reported_status == "IDLE" and snap.device_link == "OK" and snap.seq == 1
    assert snap.last_message_wall == fixed_clock()


def test_messages_split_across_chunks_and_batched():
    raw = line(seq=1) + line(seq=2, dist=1.5)
    p, stream, _, _ = connected(raw[:10], raw[10:40])
    assert p.messages_ok == 0
    stream.push(raw[40:])
    p.poll()
    p.poll()
    assert p.messages_ok == 2 and p.snapshot().distance_m == 1.5


def test_windows_line_endings_accepted():
    p, _, _, _ = connected(json.dumps(msg()).encode() + b"\r\n")
    assert p.snapshot().sensor_validity is SensorValidity.VALID


def test_invalid_json_is_rejected_and_counted():
    p, _, _, _ = connected(b"garbage\n", b'{"v":1}\n')
    p.poll()
    snap = p.snapshot()
    assert snap.messages_rejected == 2 and snap.messages_ok == 0
    assert snap.distance_m is None and snap.connection is ConnectionState.WAITING_FOR_DATA
    assert "rejected" in snap.last_error


def test_device_reported_invalid_reading_shows_no_distance():
    p, _, _, _ = connected(line(dist=None, valid=False))
    snap = p.snapshot()
    assert snap.connection is ConnectionState.CONNECTED  # the LINK is fine...
    assert snap.sensor_validity is SensorValidity.INVALID and snap.distance_m is None  # ...the reading is not


@pytest.mark.parametrize("dist", [9.0, 4.01, 0.01, 0.0, -0.5])
def test_out_of_range_distance_rejected_by_host(dist):
    p, _, _, _ = connected(line(dist=dist, valid=True))
    snap = p.snapshot()
    assert snap.sensor_validity is SensorValidity.INVALID and snap.distance_m is None
    assert "host rejected" in snap.validity_reason


def test_range_boundaries_accepted():
    for dist in (0.02, 4.0):
        p, _, _, _ = connected(line(dist=dist))
        assert p.snapshot().distance_m == dist


def test_stale_data_is_not_shown():
    p, stream, clock, _ = connected(line(seq=1, dist=0.85))
    clock.t += 1.0
    assert p.snapshot().connection is ConnectionState.CONNECTED  # exactly at the limit: still fresh
    clock.t += 0.01
    snap = p.snapshot()
    assert snap.connection is ConnectionState.STALE
    assert snap.sensor_validity is SensorValidity.STALE and snap.distance_m is None
    assert not snap.hardware_verified
    assert snap.last_message_age_s == pytest.approx(1.01)
    stream.push(line(seq=2, dist=0.9))
    p.poll()
    assert p.snapshot().distance_m == 0.9


def test_rejected_messages_do_not_keep_data_fresh():
    p, stream, clock, _ = connected(line(seq=1))
    for _ in range(5):
        clock.t += 0.5
        stream.push(b"{bad json}\n")
        p.poll()
    assert p.snapshot().connection is ConnectionState.STALE


def test_duplicate_gap_and_restart_sequences():
    p, stream, _, _ = connected(line(seq=5), line(seq=5), line(seq=9), line(seq=1))
    for _ in range(4):
        p.poll()
    assert p.messages_ok == 3 and p.messages_rejected == 1  # duplicate 5 rejected
    assert p.messages_missed == 3  # 6, 7, 8
    assert p.snapshot().seq == 1
    assert any("Sequence restarted" in e.message for e in p.log.entries)


def test_over_long_line_is_discarded_then_reading_resumes():
    p, stream, _, _ = connected(b"x" * 400, b"y" * 400)  # no newline yet, exceeds 512
    p.poll()
    assert p.messages_rejected == 1
    stream.push(b"z" * 100 + b"\n" + line(seq=3))  # rest of the junk line, then a good one
    p.poll()
    assert p.messages_rejected == 1 and p.messages_ok == 1


def test_disconnect_drops_data_and_reconnects_with_backoff():
    first, second = FakeSerial(line(seq=1)), FakeSerial()
    p, opener, clock = provider(first, OSError("port busy"), second)
    p.connect()
    p.poll()
    assert p.snapshot().distance_m == 0.85
    first.fail_with = OSError("device unplugged")
    p.poll()
    snap = p.snapshot()
    assert snap.connection is ConnectionState.RECONNECTING
    assert snap.distance_m is None and snap.sensor_validity is SensorValidity.NO_DATA
    assert first.closed and "unplugged" in snap.last_error
    p.poll()
    assert opener.calls == 1  # waits for the back-off delay (0.5 s)
    clock.t += 0.5
    p.poll()  # second attempt fails -> next delay 1.0 s
    assert opener.calls == 2 and p.state is ConnectionState.RECONNECTING
    clock.t += 0.9
    p.poll()
    assert opener.calls == 2
    clock.t += 0.1
    p.poll()
    assert opener.calls == 3 and p.state is ConnectionState.WAITING_FOR_DATA and p.reconnects == 1
    second.push(line(seq=1))  # sequence starts again after a reset; that is fine
    p.poll()
    assert p.snapshot().hardware_verified


def test_backoff_is_capped():
    p, opener, clock = provider(FakeSerial(), *[OSError("gone")] * 10)
    p.connect()
    p._stream.fail_with = OSError("unplugged")
    p.poll()
    delays = []
    for attempt in range(6):
        delays.append(round(p._next_retry_at - clock.t, 6))
        clock.t = p._next_retry_at - 0.001
        p.poll()
        assert opener.calls == attempt + 1  # not yet
        clock.t = p._next_retry_at
        p.poll()
        assert opener.calls == attempt + 2  # retried exactly on time (and failed)
    assert delays == [0.5, 1.0, 2.0, 4.0, 5.0, 5.0]


def test_failed_first_connect_is_not_retried_silently():
    p, opener, clock = provider(OSError("could not open port COM9"))
    p.connect()
    for _ in range(10):
        clock.t += 10
        p.poll()
    assert opener.calls == 1
    snap = p.snapshot()
    assert snap.connection is ConnectionState.ERROR and "COM9" in snap.last_error


def test_user_disconnect_closes_and_stops_reconnecting():
    p, stream, clock, opener = connected(line())
    p.disconnect()
    assert stream.closed
    clock.t += 60
    p.poll()
    assert opener.calls == 1
    snap = p.snapshot()
    assert snap.connection is ConnectionState.DISCONNECTED and snap.distance_m is None


def test_reader_never_writes():
    # FakeSerial.write raises; a full session including errors and reconnects must not call it.
    p, stream, clock, _ = connected(line(seq=1), b"junk\n", line(seq=2, valid=False, dist=None))
    for _ in range(3):
        p.poll()
    stream.fail_with = OSError("unplugged")
    p.poll()
    p.disconnect()


# ----------------------------------------------------------- simulation

def test_simulation_provider_is_always_labelled_simulated():
    c = build_controller(clock=fixed_clock)
    snap = SimulationTelemetryProvider(lambda: c).snapshot()
    assert snap.source == SIMULATED_SOURCE and snap.connection is ConnectionState.SIMULATED
    assert not snap.hardware_verified
    assert snap.sensor_validity is SensorValidity.VALID and snap.distance_m == pytest.approx(1.85)
    c.inject_sensor_fault(SensorFaultMode.NOT_A_NUMBER)
    snap = SimulationTelemetryProvider(lambda: c).snapshot()
    assert snap.sensor_validity is SensorValidity.INVALID and snap.distance_m is None


# ------------------------------------------------------- pyserial adapter

def test_opener_requires_a_port_name_and_opens_nothing_when_created():
    with pytest.raises(ValueError):
        make_serial_opener("  ", 115200)
    make_serial_opener("COM3", 115200)  # creating the opener must not open anything


def test_missing_pyserial_gives_install_hint(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", None)  # makes "import serial" fail
    with pytest.raises(SerialUnavailableError, match="requirements-hardware.txt"):
        make_serial_opener("COM3", 115200)()


def test_nonexistent_port_reports_error():
    pytest.importorskip("serial")
    with pytest.raises(SerialUnavailableError):
        make_serial_opener("/definitely/not/a/serial/port", 115200)()


def test_pyserial_adapter_with_virtual_loopback_port():
    pytest.importorskip("serial")
    stream = make_serial_opener("loop://", 115200)()  # pyserial's in-memory loopback, no hardware
    assert not hasattr(stream, "write")
    stream._port.write(line(seq=1))  # the TEST plays the device by writing into the loopback
    p = SerialTelemetryProvider(lambda: stream, CFG, SensorConfig(), "loop://", wall_clock=fixed_clock)
    p.connect()
    for _ in range(50):
        p.poll()
        if p.messages_ok:
            break
    assert p.snapshot().distance_m == 0.85
    p.disconnect()
