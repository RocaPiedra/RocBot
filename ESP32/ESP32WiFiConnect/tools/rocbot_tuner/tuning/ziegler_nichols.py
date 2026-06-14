"""Ziegler-Nichols open-loop step response tuning (Process Reaction Curve).

Applies a step input to the motor in open loop, measures the process
reaction curve, and computes PID gains using the tangent-line method.
"""

import asyncio
import time
import numpy as np
from typing import Any

from .base import TuningMethod, TuningResult


class ZieglerNicholsStepResponse(TuningMethod):
    """Ziegler-Nichols step response (open-loop process reaction curve).

    Measures the S-shaped process reaction curve from an open-loop step
    input and fits a tangent line at the inflection point to extract
    the dead time (L) and time constant (T) parameters.
    """

    @classmethod
    def method_id(cls) -> str:
        return "ziegler_nichols"

    @classmethod
    def method_name(cls) -> str:
        return "Ziegler-Nichols Step Response"

    @classmethod
    def parameters(cls) -> list[dict]:
        return [
            {
                "id": "step_pwm",
                "label": "Step PWM",
                "type": "int",
                "default": 80,
                "min": 30,
                "max": 200,
                "step": 10,
                "description": "Open-loop PWM amplitude for the step input",
            },
            {
                "id": "collect_seconds",
                "label": "Collection Time (s)",
                "type": "float",
                "default": 4.0,
                "min": 2.0,
                "max": 10.0,
                "step": 0.5,
                "description": "How long to record the reaction curve",
            },
            {
                "id": "pid_type",
                "label": "Controller Type",
                "type": "select",
                "default": "pid",
                "options": ["p", "pi", "pid"],
                "description": "Which controller type to tune for",
            },
        ]

    async def run(
        self,
        transport: Any,
        motor_id: str,
        target_rpm: float = 60,
        **kwargs,
    ) -> TuningResult:
        step_pwm = int(kwargs.get("step_pwm", 80))
        collect_seconds = float(kwargs.get("collect_seconds", 4.0))
        pid_type = kwargs.get("pid_type", "pid")

        data: list[dict] = []
        start_time = time.time()
        poll_interval = 0.02  # 50 Hz polling

        # Phase 1: Stop
        self._start_phase("Stopping motors")
        await transport.send_command("s")
        await asyncio.sleep(0.3)
        self._complete_phase("Motors stopped")

        # Phase 2: Capture baseline (before step)
        self._start_phase("Capturing baseline", "0.5s pre-step recording")
        for _ in range(int(0.5 / poll_interval)):
            await transport.send_command("g")
            data.append({"t": time.time() - start_time, "rpm": 0.0, "pwm": 0.0})
            await asyncio.sleep(poll_interval)
        self._complete_phase("Baseline captured")

        # Phase 3: Apply step input
        self._start_phase("Applying step", f"Sending PWM d{step_pwm}")
        await transport.send_command(f"d{step_pwm}")
        self._complete_phase(f"Step d{step_pwm} applied")

        # Phase 4: Collect reaction curve
        self._start_phase("Collecting reaction curve", f"{collect_seconds}s recording")
        polls = int(collect_seconds / poll_interval)
        for _ in range(polls):
            await transport.send_command("g")
            data.append({"t": time.time() - start_time, "pwm": float(step_pwm)})
            await asyncio.sleep(poll_interval)
        self._complete_phase(f"{polls} samples collected ({collect_seconds:.1f}s)")

        # Phase 5: Stop motors
        self._start_phase("Stopping motors")
        await transport.send_command("s")
        self._complete_phase("Motors stopped")

        # Phase 6: Compute gains
        self._start_phase("Computing gains", "Tangent-line fitting")
        result = self._compute_gains(data, step_pwm, pid_type)
        self._complete_phase(
            f"Kp={result.kp:.4f} Ki={result.ki:.6f} Kd={result.kd:.6f}"
        )
        return result

    def _compute_gains(
        self, data: list[dict], step_pwm: int, pid_type: str,
    ) -> TuningResult:
        """Compute PID gains using Ziegler-Nichols tangent-line method."""
        result = TuningResult(
            kp=1.0, ki=0.0, kd=0.0,
            method=self.method_id(), method_name=self.method_name(),
            phases=self._phases[:],
        )

        if len(data) < 10:
            result.data = {"error": f"Not enough data: {len(data)} samples"}
            return result

        rpm_arr = np.array([d.get("rpm", 0.0) for d in data])
        time_arr = np.array([d["t"] for d in data])

        # Baseline from first 10 samples
        baseline = float(np.mean(rpm_arr[:10]))
        # Final steady-state from last 20%
        last_20 = max(10, len(rpm_arr) // 5)
        final_val = float(np.mean(rpm_arr[-last_20:]))
        delta_rpm = final_val - baseline

        if abs(delta_rpm) < 1.0 or step_pwm == 0:
            result.data = {"error": f"Response too small: delta_rpm={delta_rpm:.1f}"}
            return result

        # Process gain: K = delta_rpm / delta_input
        K = delta_rpm / step_pwm

        # Smooth with moving average before computing slope
        window = max(3, len(rpm_arr) // 50)
        kernel = np.ones(window) / window
        rpm_smooth = np.convolve(rpm_arr, kernel, mode="same")

        # Find inflection point (maximum slope)
        slopes = np.diff(rpm_smooth) / np.diff(time_arr)
        # Skip initial noise (first 5 samples), skip near the end (last 10%)
        start_idx = 5
        end_idx = len(slopes) - len(slopes) // 10
        search_slopes = np.abs(slopes[start_idx:end_idx])
        if len(search_slopes) == 0:
            result.data = {"error": "No valid inflection point found"}
            return result

        inflect_offset = int(np.argmax(search_slopes))
        inflect_idx = start_idx + inflect_offset
        inflect_slope = slopes[inflect_idx]

        if abs(inflect_slope) < 1e-6:
            result.data = {"error": "Slope too flat — no response detected"}
            return result

        # Tangent line at inflection point
        t_inflect = time_arr[inflect_idx]
        rpm_inflect = rpm_smooth[inflect_idx]

        # Dead time L: where tangent line crosses baseline
        L = t_inflect - (rpm_inflect - baseline) / inflect_slope
        L = max(L, 0.0)

        # Time constant T: time for tangent to go from baseline to final value
        T = (final_val - baseline) / inflect_slope

        L = max(L, 0.001)
        T = max(T, 0.01)
        a = K * L / T  # Normalized gain for ZN table

        # Ziegler-Nichols rules
        if pid_type == "p":
            result.kp = 1.0 / a
            result.ki = 0.0
            result.kd = 0.0
        elif pid_type == "pi":
            result.kp = 0.9 / a
            result.ki = result.kp / (3.33 * L)
            result.kd = 0.0
        else:  # pid
            result.kp = 1.2 / a
            result.ki = result.kp / (2.0 * L)
            result.kd = result.kp * 0.5 * L

        result.data = {
            "K": float(K),
            "L (dead time)": float(L),
            "T (time constant)": float(T),
            "a (norm. gain)": float(a),
            "baseline_rpm": float(baseline),
            "final_rpm": float(final_val),
            "step_pwm": step_pwm,
            "inflection_slope_rpm/s": float(inflect_slope),
            "pid_type": pid_type,
        }

        return result
