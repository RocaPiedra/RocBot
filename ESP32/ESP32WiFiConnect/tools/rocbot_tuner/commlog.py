"""Communication event log for RocBot tuner.

Records *all* micro-ROS / serial traffic plus the micro-ROS agent
(Docker container) status transitions, so every behaviour can be
tracked from a single place.

Event model
-----------
Each event has a timestamp, a direction and a transport tag:

- ``TX``    — app  -> ESP32 (commands such as ``p``, ``kp1.0``, ``d100``)
- ``RX``    — ESP32 -> app  (parsed state frames, raw debug lines, topics)
- ``AGENT`` — micro-ROS agent / Docker status change or probe result
- ``SYS``   — app-internal events (connect, disconnect, errors, ...)

Thread-safety: NiceGUI callbacks, the ROS2 spin loop and the serial
reader run on different asyncio tasks/threads, so every mutation is
guarded by a lock.
"""

from __future__ import annotations

import os
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Optional

DIRECTIONS = ("TX", "RX", "AGENT", "SYS")
LEVELS = ("DEBUG", "INFO", "WARN", "ERROR")

MAX_EVENTS_DEFAULT = 5000


@dataclass
class CommEvent:
    """One communication / status event."""

    seq: int = 0
    wall_time: float = field(default_factory=time.time)
    direction: str = "SYS"      # TX | RX | AGENT | SYS
    transport: str = "-"        # serial | ros2 | agent | sys
    level: str = "INFO"         # DEBUG | INFO | WARN | ERROR
    message: str = ""
    agent_running: Optional[bool] = None  # agent snapshot at event time

    @property
    def timestamp_str(self) -> str:
        return datetime.fromtimestamp(self.wall_time).strftime("%H:%M:%S.%f")[:-3]

    def format_line(self) -> str:
        agent = ""
        if self.agent_running is not None:
            agent = " [agent:UP]" if self.agent_running else " [agent:DOWN]"
        return (
            f"{self.timestamp_str} #{self.seq:05d} "
            f"[{self.direction:5s}] [{self.transport:6s}]{agent} {self.message}"
        )


@dataclass
class AgentStatus:
    """Point-in-time snapshot of the micro-ROS server side."""

    docker_running: bool = False
    transport_connected: bool = False
    transport_type: str = "-"
    rclpy_ok: bool = False
    last_rx_age_s: Optional[float] = None
    tx_count: int = 0
    rx_count: int = 0
    checked_at: float = field(default_factory=time.time)

    @property
    def link(self) -> str:
        """High-level link state derived from the raw flags."""
        if self.transport_connected and self.docker_running:
            return "LINKED"
        if self.transport_connected:
            return "TRANSPORT-ONLY"
        if self.docker_running:
            return "AGENT-ONLY"
        return "DOWN"

    def summary(self) -> str:
        age = "never" if self.last_rx_age_s is None else f"{self.last_rx_age_s:.1f}s ago"
        return (
            f"link={self.link} docker={'UP' if self.docker_running else 'DOWN'} "
            f"transport={self.transport_type}:"
            f"{'UP' if self.transport_connected else 'DOWN'} "
            f"rclpy={'ok' if self.rclpy_ok else 'n/a'} "
            f"lastRX={age} TX={self.tx_count} RX={self.rx_count}"
        )


def is_docker_agent_running(container: str = "rocbot_microros_agent") -> bool:
    """Check whether the micro-ROS agent Docker container is running."""
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", f"name={container}", "--format", "{{.Names}}"],
            capture_output=True, text=True, check=False, timeout=5,
        )
        return container in result.stdout
    except Exception:
        return False


class CommLogger:
    """Ring-buffer + file-backed log of all comm events."""

    def __init__(
        self,
        maxlen: int = MAX_EVENTS_DEFAULT,
        log_dir: Optional[str] = None,
        file_enabled: bool = True,
    ):
        self._lock = threading.Lock()
        self._events: deque[CommEvent] = deque(maxlen=maxlen)
        self._seq = 0
        self._listeners: list[Callable[[CommEvent], None]] = []
        self.tx_count = 0
        self.rx_count = 0
        self.log_dir = log_dir
        self._file = None
        self._file_path: Optional[str] = None

        if file_enabled:
            if log_dir is None:
                log_dir = os.path.join(os.path.dirname(__file__), "logs")
            os.makedirs(log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._file_path = os.path.join(log_dir, f"commlog_{stamp}.log")
            try:
                self._file = open(self._file_path, "w", encoding="utf-8")
                self._file.write(f"# RocBot comm log started {stamp}\n")
                self._file.flush()
            except OSError:
                self._file = None

    # ── core ──────────────────────────────────────────────────────────
    def log(
        self,
        direction: str,
        message: str,
        transport: str = "-",
        level: str = "INFO",
        agent_running: Optional[bool] = None,
    ) -> CommEvent:
        with self._lock:
            self._seq += 1
            ev = CommEvent(
                seq=self._seq,
                direction=direction,
                transport=transport,
                level=level,
                message=message,
                agent_running=agent_running,
            )
            self._events.append(ev)
            if direction == "TX":
                self.tx_count += 1
            elif direction == "RX":
                self.rx_count += 1
            if self._file:
                try:
                    self._file.write(ev.format_line() + "\n")
                    # Flush RX at a reduced rate to avoid an I/O bottleneck
                    # at 20 Hz; everything else flushes immediately.
                    if direction != "RX" or ev.seq % 10 == 0:
                        self._file.flush()
                except OSError:
                    pass
            listeners = list(self._listeners)
        for cb in listeners:
            try:
                cb(ev)
            except Exception:
                pass
        return ev

    # ── convenience ───────────────────────────────────────────────────
    def tx(self, cmd: str, transport: str = "-", agent_running: Optional[bool] = None) -> CommEvent:
        return self.log("TX", f">> {cmd}", transport=transport, agent_running=agent_running)

    def rx(self, message: str, transport: str = "-", agent_running: Optional[bool] = None) -> CommEvent:
        return self.log("RX", message, transport=transport, level="DEBUG", agent_running=agent_running)

    def agent(self, message: str, level: str = "INFO") -> CommEvent:
        return self.log("AGENT", message, transport="agent", level=level)

    def sys(self, message: str, level: str = "INFO") -> CommEvent:
        return self.log("SYS", message, transport="sys", level=level)

    def error(self, message: str, transport: str = "sys") -> CommEvent:
        return self.log("SYS", message, transport=transport, level="ERROR")

    # ── access ────────────────────────────────────────────────────────
    def add_listener(self, cb: Callable[[CommEvent], None]):
        with self._lock:
            self._listeners.append(cb)

    def snapshot(
        self,
        directions: Optional[set[str]] = None,
        levels: Optional[set[str]] = None,
        search: str = "",
        limit: int = 2000,
    ) -> list[CommEvent]:
        with self._lock:
            events = list(self._events)
        if directions:
            events = [e for e in events if e.direction in directions]
        if levels:
            events = [e for e in events if e.level in levels]
        if search:
            needle = search.lower()
            events = [e for e in events if needle in e.message.lower() or needle in e.transport.lower()]
        if limit and len(events) > limit:
            events = events[-limit:]
        return events

    def clear(self):
        with self._lock:
            self._events.clear()

    def export(self, path: str) -> str:
        with self._lock:
            events = list(self._events)
        with open(path, "w", encoding="utf-8") as f:
            for ev in events:
                f.write(ev.format_line() + "\n")
        return path

    def close(self):
        with self._lock:
            if self._file:
                try:
                    self._file.flush()
                    self._file.close()
                except OSError:
                    pass
                self._file = None

    @property
    def file_path(self) -> Optional[str]:
        return self._file_path

    @property
    def last_seq(self) -> int:
        with self._lock:
            return self._seq

    def __len__(self) -> int:
        with self._lock:
            return len(self._events)
