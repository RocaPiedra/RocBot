"""Step response metrics calculation."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class StepResponseMetrics:
    """Metrics from a step response test."""
    target_rpm: float = 0.0
    peak_rpm: float = 0.0
    peak_time: float = 0.0  # seconds
    overshoot_pct: float = 0.0
    rise_time: float = 0.0  # 10% to 90%
    settling_time: float = 0.0  # within 2% of target
    steady_state_error: float = 0.0
    final_rpm: float = 0.0
    test_duration: float = 0.0

    def summary(self) -> str:
        return (
            f"Target: {self.target_rpm:.0f} RPM\n"
            f"Peak: {self.peak_rpm:.1f} RPM ({self.peak_time:.2f}s)\n"
            f"Overshoot: {self.overshoot_pct:.1f}%\n"
            f"Rise time: {self.rise_time:.2f}s\n"
            f"Settling: {self.settling_time:.2f}s\n"
            f"SS error: {self.steady_state_error:.1f} RPM\n"
            f"Final: {self.final_rpm:.1f} RPM"
        )


def calculate_step_response(
    timestamps: list[float],
    rpm_values: list[float],
    target_rpm: float,
    settling_pct: float = 0.02
) -> StepResponseMetrics:
    """
    Calculate step response metrics from time-series data.

    Args:
        timestamps: List of elapsed times in seconds
        rpm_values: List of filtered RPM values
        target_rpm: The step target RPM
        settling_pct: Fraction of target for settling band (default 2%)
    """
    metrics = StepResponseMetrics(target_rpm=target_rpm)

    if len(timestamps) < 2 or len(rpm_values) < 2:
        return metrics

    import numpy as np

    rpm_arr = np.array(rpm_values)
    time_arr = np.array(timestamps)
    test_duration = time_arr[-1] - time_arr[0]
    metrics.test_duration = test_duration

    # Peak
    peak_idx = np.argmax(rpm_arr)
    metrics.peak_rpm = float(rpm_arr[peak_idx])
    metrics.peak_time = float(time_arr[peak_idx])

    # Overshoot
    if target_rpm > 0:
        metrics.overshoot_pct = max(0.0, (metrics.peak_rpm - target_rpm) / target_rpm * 100)
    elif target_rpm < 0:
        metrics.overshoot_pct = max(0.0, (target_rpm - metrics.peak_rpm) / abs(target_rpm) * 100)

    # Rise time (10% to 90% of target)
    try:
        if target_rpm > 0:
            idx_10 = next(i for i, v in enumerate(rpm_arr) if v >= target_rpm * 0.1)
            idx_90 = next(i for i, v in enumerate(rpm_arr) if v >= target_rpm * 0.9)
        else:
            idx_10 = next(i for i, v in enumerate(rpm_arr) if v <= target_rpm * 0.1)
            idx_90 = next(i for i, v in enumerate(rpm_arr) if v <= target_rpm * 0.9)
        metrics.rise_time = float(time_arr[idx_90] - time_arr[idx_10])
    except StopIteration:
        metrics.rise_time = 0.0

    # Settling time (last point outside settling band)
    try:
        band = abs(target_rpm) * settling_pct
        # Find last index outside band
        outside_indices = [i for i in range(len(rpm_arr)) if abs(rpm_arr[i] - target_rpm) > band]
        if outside_indices:
            last_outside = outside_indices[-1]
            metrics.settling_time = float(time_arr[last_outside])
        else:
            metrics.settling_time = 0.0
    except Exception:
        metrics.settling_time = 0.0

    # Steady-state error (last 20% of data)
    last_20 = max(1, int(len(rpm_arr) * 0.8))
    metrics.steady_state_error = float(target_rpm - np.mean(rpm_arr[last_20:]))
    metrics.final_rpm = float(rpm_arr[-1])

    return metrics
