"""Abstract transport interface for RocBot motor controller."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class MotorState:
    """Parsed motor state from one motor."""
    motor_id: str = ""
    target_rpm: float = 0.0
    rpm: float = 0.0
    rpm_filt: float = 0.0
    pwr: float = 0.0
    pwr_filt: float = 0.0
    direction: str = "STP"  # FWD, REV, STP
    pulses: int = 0
    timestamp: float = 0.0  # seconds since start

    @property
    def error(self) -> float:
        return self.target_rpm - self.rpm


@dataclass
class ControllerState:
    """Overall controller state from debug output."""
    elapsed_ms: float = 0.0
    mode: str = "STOP"  # PID, DIRECT, STEP_TEST, STOP
    target_value: float = 0.0
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    motors: dict[str, MotorState] = field(default_factory=dict)


class Transport(ABC):
    """Abstract transport for RocBot communication."""

    @abstractmethod
    async def connect(self) -> bool:
        """Connect to the controller. Returns True on success."""
        ...

    @abstractmethod
    async def disconnect(self):
        """Disconnect from the controller."""
        ...

    @abstractmethod
    async def send_command(self, cmd: str):
        """Send a command string to the controller."""
        ...

    @abstractmethod
    async def read_loop(self, callback):
        """
        Async generator that yields parsed ControllerState objects.
        The callback is called with each new ControllerState.
        Runs until disconnect() is called.
        """
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the transport is currently connected."""
        ...
