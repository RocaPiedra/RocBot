"""Abstract transport interface for RocBot motor controller."""

from abc import ABC, abstractmethod

from rocbot_tuner.models import MotorState, ControllerState


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
