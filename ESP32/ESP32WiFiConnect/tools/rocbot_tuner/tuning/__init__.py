"""PID Tuning method registry — pluggable tuning strategies."""

from .base import TuningMethod, TuningResult, TuningPhase
from .ziegler_nichols import ZieglerNicholsStepResponse
from .relay import RelayTuning
from .cohen_coon import CohenCoonTuning

# Registry: all available tuning methods
METHODS: dict[str, type[TuningMethod]] = {
    "ziegler_nichols": ZieglerNicholsStepResponse,
    "relay": RelayTuning,
    "cohen_coon": CohenCoonTuning,
}

def list_methods() -> list[dict]:
    """Return metadata for all registered tuning methods."""
    return [
        {
            "id": mid,
            "name": cls.__name__,
            "description": cls.__doc__.strip().split("\n")[0] if cls.__doc__ else "",
            "parameters": cls.parameters(),
        }
        for mid, cls in METHODS.items()
    ]

def get_method(method_id: str) -> type[TuningMethod]:
    """Get a tuning method class by ID."""
    if method_id not in METHODS:
        raise KeyError(f"Unknown tuning method: {method_id}. Available: {list(METHODS.keys())}")
    return METHODS[method_id]

__all__ = [
    "TuningMethod", "TuningResult", "TuningPhase",
    "ZieglerNicholsStepResponse", "RelayTuning", "CohenCoonTuning",
    "METHODS", "list_methods", "get_method",
]
