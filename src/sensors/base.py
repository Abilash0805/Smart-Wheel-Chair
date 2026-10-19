"""Sensor interface shared by simulated (and, later, real) distance sensors."""

from __future__ import annotations

from abc import ABC, abstractmethod

from src.models import SensorReading


class DistanceSensor(ABC):
    """Anything that can produce a forward distance reading.

    Implementations must report raw values honestly, including failures
    (None, NaN, out-of-range numbers). They must never substitute a made-up
    "clear" value. Validation happens in ``src.safety``.
    """

    #: Where readings come from, shown next to every value ("SIMULATED", ...).
    source: str = "UNKNOWN"
    name: str = "distance sensor"

    @abstractmethod
    def read(self) -> SensorReading:
        """Take one measurement."""
