"""Cohen-Coon tuning (open-loop step response).

Similar to Ziegler-Nichols but uses different formulas optimized for
systems with significant dead time and disturbance rejection. Produces
more aggressive gains that often work better for motor speed control.
"""

import asyncio
import time
import numpy as np
from typing import Any

from .base import TuningMethod, TuningResult


class CohenCoonTuning(TuningMethod):
    """Cohen-Coon open-loop step response tuning.

    Uses the same process reaction curve as Ziegler-Nichols but applies
    different formulas optimized for disturbance rejection and systems
    with significant dead time (L/T > 0.1). Often yields better
    performance for motor speed control than standard ZN.
    """

    @classmethod
    def method_id(cls) -> str:
        return "cohen_coon"

    @classmethod
    def method_name(cls) -> str:
        return "Cohen-Coon Step Response"

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
        state_source=lambda: None,
        **kwargs,
    ) -> TuningResult:
        step_pwm = int(kwargs.get("step_pwm", 80))
        collect_seconds = float(kwargs.get("collect_seconds", 4.0))
        pid_type = kwargs.get("pid_type", "pid")

        data: list[dict] = []
        start_time = time.time()
        poll_interval = 0.02
        self._running = True

        def _poll_rpm() -> float:
            state = state_source()
            if state and motor_id in state.motors:
                return state.motors[motor_id].rpm_filt
            return 0.0

        # Phase 1: Stop
        self._start_phase("Stopping motors")
        await transport.send_command("s")
        await asyncio.sleep(0.3)
        self._complete_phase("Motors stopped")

        # Phase 2: Baseline
        self._start_phase("Capturing baseline", "0.5s pre-step")
        for _ in range(int(0.5 / poll_interval)):
            if not self._running:
                break
            data.append({"t": time.time() - start_time, "rpm": _poll_rpm(), "pwm": 0.0})
            await asyncio.sleep(poll_interval)
        self._complete_phase("Baseline captured")

        # Phase 3: Step
        self._start_phase("Applying step", f"d{step_pwm}")
        await transport.send_command(f"d{step_pwm}")
        self._complete_phase(f"Step d{step_pwm} applied")

        # Phase 4: Collect
        self._start_phase("Collecting reaction curve", f"{collect_seconds}s")
        polls = int(collect_seconds / poll_interval)
        for _ in range(polls):
            if not self._running:
                break
            data.append({"t": time.time() - start_time, "rpm": _poll_rpm(), "pwm": float(step_pwm)})
            await asyncio.sleep(poll_interval)
        self._complete_phase(f"{len(data)} samples")

        # Phase 5: Stop
        self._start_phase("Stopping motors")
        await transport.send_command("s")
        self._complete_phase("Motors stopped")

        # Phase 6: Compute
        self._start_phase("Computing gains", "Cohen-Coon formulas")
        result = self._compute_gains(data, step_pwm, pid_type)
        self._complete_phase(
            f"Kp={result.kp:.4f} Ki={result.ki:.6f} Kd={result.kd:.6f}"
        )
        return result

    def _compute_gains(
        self, data: list[dict], step_pwm: int, pid_type: str,
    ) -> TuningResult:
        """Cohen-Coon tuning from process reaction curve parameters."""
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

        baseline = float(np.mean(rpm_arr[:10]))
        last_20 = max(10, len(rpm_arr) // 5)
        final_val = float(np.mean(rpm_arr[-last_20:]))
        delta_rpm = final_val - baseline

        if abs(delta_rpm) < 1.0 or step_pwm == 0:
            result.data = {"error": f"Response too small: delta_rpm={delta_rpm:.1f}"}
            return result

        K = delta_rpm / step_pwm

        # Smooth for slope computation
        window = max(3, len(rpm_arr) // 50)
        kernel = np.ones(window) / window
        rpm_smooth = np.convolve(rpm_arr, kernel, mode="same")

        # Find inflection
        slopes = np.diff(rpm_smooth) / np.diff(time_arr)
        start_idx = 5
        end_idx = len(slopes) - len(slopes) // 10
        search_slopes = np.abs(slopes[start_idx:end_idx])
        if len(search_slopes) == 0:
            result.data = {"error": "No inflection point"}
            return result

        inflect_idx = start_idx + int(np.argmax(search_slopes))
        inflect_slope = slopes[inflect_idx]

        if abs(inflect_slope) < 1e-6:
            result.data = {"error": "Slope too flat"}
            return result

        t_inflect = time_arr[inflect_idx]
        rpm_inflect = rpm_smooth[inflect_idx]

        # Dead time L and time constant T (same as ZN)
        L = t_inflect - (rpm_inflect - baseline) / inflect_slope
        L = max(L, 0.0)
        T = (final_val - baseline) / inflect_slope
        L = max(L, 0.001)
        T = max(T, 0.01)

        # Cohen-Coon formulas (use tau = L/T ratio)
        tau = L / T

        if pid_type == "p":
            result.kp = (1.0 / K) * (1.0 + tau / 3.0) / (1.0 + tau)
            result.ki = 0.0
            result.kd = 0.0
        elif pid_type == "pi":
            result.kp = (0.9 / K) * (1.0 + 0.92 * tau) / (1.0 + tau)
            result.ki = result.kp / ((3.33 * L) * (1.0 + 0.3 * tau) / (1.0 + 2.2 * tau))
            result.kd = 0.0
        else:  # pid
            result.kp = (1.35 / K) * (1.0 + 0.18 * tau) / (1.0 + 0.61 * tau)
            result.ki = result.kp / ((2.5 * L) * (1.0 + 0.3 * tau) / (1.0 + 1.3 * tau))
            result.kd = result.kp * (0.37 * L) * (1.0 - 0.19 * tau) / (1.0 - 0.41 * tau)

        # Clamp Kd to non-negative
        result.kd = max(0.0, result.kd)

        result.data = {
            "K (process gain)": float(K),
            "L (dead time, s)": float(L),
            "T (time constant, s)": float(T),
            "tau = L/T": float(tau),
            "baseline_rpm": float(baseline),
            "final_rpm": float(final_val),
            "step_pwm": step_pwm,
            "pid_type": pid_type,
        }

        return result
