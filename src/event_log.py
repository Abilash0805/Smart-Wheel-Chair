"""Timestamped event log for commands, state changes and diagnostics."""

from __future__ import annotations

import csv
import io
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from enum import StrEnum


class Level(StrEnum):
    INFO = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


@dataclass(frozen=True)
class LogEntry:
    seq: int
    wall_time: datetime
    sim_time_s: float
    level: Level
    category: str
    message: str


class EventLog:
    def __init__(self, clock: Callable[[], datetime] | None = None, max_entries: int = 1000) -> None:
        self._clock = clock or (lambda: datetime.now().astimezone())
        self._entries: deque[LogEntry] = deque(maxlen=max_entries)
        self._next_seq = 1

    def add(self, level: Level, category: str, message: str, sim_time_s: float) -> LogEntry:
        entry = LogEntry(self._next_seq, self._clock(), sim_time_s, Level(level), category, message)
        self._next_seq += 1
        self._entries.append(entry)
        return entry

    @property
    def entries(self) -> list[LogEntry]:
        return list(self._entries)

    def latest(self, n: int) -> list[LogEntry]:
        """Most recent entries first."""
        return list(reversed(self._entries))[:n]

    def count(self, level: Level | None = None) -> int:
        if level is None:
            return len(self._entries)
        return sum(1 for e in self._entries if e.level == level)

    def to_csv(self) -> str:
        buffer = io.StringIO()
        writer = csv.writer(buffer)
        writer.writerow(["seq", "wall_time", "sim_time_s", "level", "category", "message"])
        for e in self._entries:
            writer.writerow([e.seq, e.wall_time.isoformat(timespec="seconds"), f"{e.sim_time_s:.2f}",
                             e.level.value, e.category, e.message])
        return buffer.getvalue()
