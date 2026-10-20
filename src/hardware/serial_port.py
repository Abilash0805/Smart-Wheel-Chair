"""The ONLY place that touches a real serial port, and only when the user connects.

* ``pyserial`` is imported inside the functions below, never at module level,
  so importing this module (or starting the dashboard) does not load it.
* The returned stream wrapper exposes ``read_available`` and ``close`` only.
  There is no write method: telemetry is read-only, and nothing here can send
  motor commands.
* Opening an Arduino Uno's USB serial port normally resets the board (the DTR
  line toggles). That is expected behaviour, not a fault.

pyserial is optional: ``pip install -r requirements-hardware.txt``.
"""

from __future__ import annotations

from collections.abc import Callable

INSTALL_HINT = "pyserial is not installed. Run: .\\.venv\\Scripts\\python.exe -m pip install -r requirements-hardware.txt"


class SerialUnavailableError(RuntimeError):
    """pyserial is missing or the port could not be opened."""


class _ReadOnlySerialStream:
    def __init__(self, port) -> None:
        self._port = port

    def read_available(self) -> bytes:
        waiting = self._port.in_waiting
        return self._port.read(waiting) if waiting else b""

    def close(self) -> None:
        self._port.close()


def make_serial_opener(port: str, baud_rate: int) -> Callable[[], _ReadOnlySerialStream]:
    """Return a function that opens the port when called. Nothing is opened here."""
    port = port.strip()
    if not port:
        raise ValueError("Enter a serial port name first (for example COM3 on Windows)")

    def open_port() -> _ReadOnlySerialStream:
        try:
            import serial  # noqa: PLC0415 - deliberately lazy; see module docstring
        except ImportError:
            raise SerialUnavailableError(INSTALL_HINT) from None
        try:
            # serial_for_url accepts normal names (COM3, /dev/ttyACM0) and test URLs (loop://).
            handle = serial.serial_for_url(port, baudrate=baud_rate, timeout=0)
        except (serial.SerialException, ValueError, OSError) as exc:
            raise SerialUnavailableError(f"{port}: {exc}") from None
        return _ReadOnlySerialStream(handle)

    return open_port


def list_serial_ports() -> list[tuple[str, str]]:
    """(device, description) pairs. Only lists ports; opens nothing."""
    try:
        from serial.tools import list_ports  # noqa: PLC0415 - deliberately lazy
    except ImportError:
        raise SerialUnavailableError(INSTALL_HINT) from None
    return sorted((p.device, p.description or "") for p in list_ports.comports())
