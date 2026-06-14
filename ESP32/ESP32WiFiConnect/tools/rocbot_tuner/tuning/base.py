"""Abstract base class for pluggable PID tuning strategies."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Optional


@dataclass
class TuningPhase:
    """A single phase in a tuning procedure (for UI progress tracking)."""
    name: str
    status: str = "pending"  # pending, running, complete, error
    message: str = ""


def _phase_icon(status: str) -> str:
    return {
        "pending": "O",
        "running": ">",
        "complete": "OK",
        "error": "XX",
    }.get(status, "?")


@dataclass
class TuningResult:
    """Result of a tuning procedure with recommended gains."""
    kp: float
    ki: float
    kd: float
    output_scale: float = 10.0
    method: str = ""
    method_name: str = ""
    phases: list[TuningPhase] = field(default_factory=list)
    data: dict[str, Any] = field(default_factory=dict)

    def summary(self) -> str:
        phase_line = " → ".join(
            f"[{_phase_icon(p.status)}] {p.name}" for p in self.phases
        )
        return (
            f"**Method:** {self.method_name}\n\n"
            f"| Gain | Value |\n"
            f"|------|-------|\n"
            f"| Kp | `{self.kp:.4f}` |\n"
            f"| Ki | `{self.ki:.6f}` |\n"
            f"| Kd | `{self.kd:.6f}` |\n"
            f"| Out Scale | `{self.output_scale:.1f}` |\n\n"
            f"**Phases:** {phase_line}"
        )


class TuningMethod(ABC):
    """Abstract base for a PID tuning strategy.

    Subclasses implement `run()` which executes the full tuning
    procedure via the transport and returns recommended gains.
    The strategy pattern makes it trivial to swap tuning algorithms.
    """

    def __init__(self):
        self._phases: list[TuningPhase] = []
        self._on_phase_change: Optional[Callable] = None

    @abstractmethod
    async def run(
        self,
        transport: Any,
        motor_id: str,
        target_rpm: float,
        **kwargs,
    ) -> TuningResult:
        """Execute the full tuning procedure.

        Args:
            transport: Connected Transport object
            motor_id: Motor to tune (FL, FR, BL, BR)
            target_rpm: Target RPM for the test step
            **kwargs: Method-specific parameters

        Returns:
            TuningResult with recommended PID gains
        """
        ...

    @classmethod
    @abstractmethod
    def parameters(cls) -> list[dict]:
        """Return configurable parameter specs for the UI.

        Each entry::
            {
                "id": str,
                "label": str,
                "type": "float" | "int" | "bool" | "select",
                "default": Any,
                "min": float,
                "max": float,
                "step": float,
                "options": list,
                "description": str,
            }
        """
        ...

    @classmethod
    @abstractmethod
    def method_id(cls) -> str:
        """Unique identifier for this tuning method."""
        ...

    @classmethod
    @abstractmethod
    def method_name(cls) -> str:
        """Human-readable name."""
        ...

    # --- Phase tracking helpers ---

    def on_phase_change(self, callback: Callable):
        """Register callback for phase transitions (for UI progress)."""
        self._on_phase_change = callback

    def _start_phase(self, name: str, message: str = ""):
        """Mark the start of a new phase."""
        phase = TuningPhase(name=name, status="running", message=message)
        self._phases.append(phase)
        if self._on_phase_change:
            self._on_phase_change(phase)

    def _complete_phase(self, message: str = ""):
        """Mark the current phase as complete."""
        if self._phases:
            self._phases[-1].status = "complete"
            self._phases[-1].message = message
            if self._on_phase_change:
                self._on_phase_change(self._phases[-1])

    def _fail_phase(self, message: str = ""):
        """Mark the current phase as failed."""
        if self._phases:
            self._phases[-1].status = "error"
            self._phases[-1].message = message
            if self._on_phase_change:
                self._on_phase_change(self._phases[-1])
