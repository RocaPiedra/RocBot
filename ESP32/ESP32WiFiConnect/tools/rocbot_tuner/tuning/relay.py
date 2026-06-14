"""Åström-Hägglund relay feedback tuning.

Replaces the PID controller with a relay (ON-OFF) that switches when
the error crosses zero, causing the system to oscillate at its ultimate
gain. The oscillation amplitude and period are used to compute PID gains.
Robust to noise, works well for systems with significant dead time.
"""

import asyncio
import time
import numpy as np
from typing import Any

from .base import TuningMethod, TuningResult


class RelayTuning(TuningMethod):
    """Relay feedback auto-tuning (Åström-Hägglund).

    Injects a relay (bang-bang) control signal that causes sustained
    oscillations. Measures ultimate gain Ku and ultimate period Tu,
    then applies tuning rules to compute PID gains.
    """

    @classmethod
    def method_id(cls) -> str:
        return "relay"

    @classmethod
    def method_name(cls) -> str:
        return "Relay Feedback (Astrom-Hagglund)"

    @classmethod
    def parameters(cls) -> list[dict]:
        return [
            {
                "id": "relay_amplitude",
                "label": "Relay Amplitude (PWM)",
                "type": "int",
                "default": 80,
                "min": 30,
                "max": 200,
                "step": 10,
                "description": "PWM amplitude of the relay ON signal",
            },
            {
                "id": "hysteresis",
                "label": "Hysteresis (RPM)",
                "type": "float",
                "default": 5.0,
                "min": 1.0,
                "max": 30.0,
                "step": 1.0,
                "description": "Deadband around zero error to prevent chatter",
            },
            {
                "id": "max_cycles",
                "label": "Max Cycles",
                "type": "int",
                "default": 6,
                "min": 3,
                "max": 20,
                "step": 1,
                "description": "Number of relay cycles to observe before stopping",
            },
            {
                "id": "target_rpm",
                "label": "Target RPM",
                "type": "int",
                "default": 60,
                "min": 20,
                "max": 200,
                "step": 10,
                "description": "Target RPM around which the relay oscillates",
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
        relay_amp = int(kwargs.get("relay_amplitude", 80))
        hysteresis = float(kwargs.get("hysteresis", 5.0))
        max_cycles = int(kwargs.get("max_cycles", 6))
        pid_type = kwargs.get("pid_type", "pid")
        setpoint = float(kwargs.get("target_rpm", target_rpm))

        # Phase 1: Stop & prepare
        self._start_phase("Stopping motors")
        await transport.send_command("s")
        await asyncio.sleep(0.3)
        self._complete_phase("Motors stopped")

        # Phase 2: Enable PID mode with zero gains (we'll use relay instead)
        self._start_phase("Setting up relay", f"Amplitude={relay_amp} Hysteresis={hysteresis}")
        # Set PID to minimal gains so the motor can respond to direct commands
        await transport.send_command("kp0")
        await transport.send_command("ki0")
        await transport.send_command("kd0")
        await asyncio.sleep(0.1)
        self._complete_phase("Relay configured")

        # Phase 3: Run relay cycles
        self._start_phase(
            "Running relay cycles",
            f"Observing up to {max_cycles} oscillation cycles",
        )

        # Relay state
        relay_on = True
        relay_output = relay_amp
        prev_error = 0.0
        zero_crossings = 0
        crossing_times: list[float] = []
        oscillation_amplitudes: list[float] = []
        current_peak = 0.0
        start_time = time.time()

        # We track oscillations by monitoring the process variable via 'g' commands
        poll_interval = 0.02
        time_limit = 30.0  # Safety timeout

        last_reported_rpm = 0.0

        while zero_crossings < max_cycles * 2 and (time.time() - start_time) < time_limit:
            # Poll current state
            await transport.send_command("g")
            # Apply relay control
            if relay_on:
                await transport.send_command(f"d{relay_output}")
            else:
                await transport.send_command("s")

            await asyncio.sleep(poll_interval)

            # The actual RPM feedback comes from the ongoing serial_reader
            # that populates shared state. For standalone use we rely on the
            # caller to provide feedback via shared state or kwargs.
            # We use a simplified approach: send 'g' and read the response
            # via a callback we've registered.
            current_rpm = getattr(self, "_last_rpm", 0.0)

            error = setpoint - current_rpm

            # Relay switching logic with hysteresis
            if error > hysteresis:
                relay_on = True
                relay_output = relay_amp
            elif error < -hysteresis:
                relay_on = False
                relay_output = 0
            # else stay in current state (hysteresis band)

            # Detect zero crossings
            if prev_error != 0 and error * prev_error < 0:
                zero_crossings += 1
                crossing_times.append(time.time() - start_time)
                oscillation_amplitudes.append(current_peak)
                current_peak = 0.0

            # Track peak amplitude within each half-cycle
            if abs(error) > current_peak:
                current_peak = abs(error)

            prev_error = error

        # Phase 4: Stop
        self._start_phase("Stopping motors")
        await transport.send_command("s")
        await asyncio.sleep(0.2)
        self._complete_phase("Motors stopped")

        # Phase 5: Compute gains from oscillation data
        self._start_phase("Computing gains", "Relay analysis")
        result = self._compute_gains(
            crossing_times, oscillation_amplitudes, relay_amp, setpoint, pid_type,
        )
        self._complete_phase(
            f"Kp={result.kp:.4f} Ki={result.ki:.6f} Kd={result.kd:.6f}"
        )

        return result

    def _compute_gains(
        self,
        crossing_times: list[float],
        amplitudes: list[float],
        relay_amp: int,
        setpoint: float,
        pid_type: str,
    ) -> TuningResult:
        result = TuningResult(
            kp=1.0, ki=0.0, kd=0.0,
            method=self.method_id(), method_name=self.method_name(),
            phases=self._phases[:],
        )

        if len(crossing_times) < 4 or len(amplitudes) < 2:
            result.data = {"error": f"Not enough oscillation data: {len(crossing_times)} crossings, {len(amplitudes)} amplitudes"}
            return result

        # Ultimate period Tu: average time between zero crossings
        intervals = np.diff(crossing_times)
        Tu = float(2.0 * np.mean(intervals))  # Full period = 2 half-cycles

        # Ultimate amplitude: average of oscillation amplitudes
        a = float(np.mean(amplitudes[-max(3, len(amplitudes)//2):]))

        if a < 0.1:
            result.data = {"error": f"Oscillation amplitude too small: {a:.2f} RPM"}
            return result

        # Relay amplitude (d): the output swing
        d = float(relay_amp)

        # Ultimate gain: Ku = 4*d / (pi * a)
        Ku = 4.0 * d / (np.pi * a)

        # Apply Ziegler-Nichols from ultimate gain/period
        if pid_type == "p":
            result.kp = 0.5 * Ku
            result.ki = 0.0
            result.kd = 0.0
        elif pid_type == "pi":
            result.kp = 0.45 * Ku
            result.ki = result.kp / (0.83 * Tu)
            result.kd = 0.0
        else:  # pid
            result.kp = 0.60 * Ku
            result.ki = result.kp / (0.5 * Tu)
            result.kd = result.kp * 0.125 * Tu

        result.data = {
            "Ku (ultimate gain)": float(Ku),
            "Tu (ultimate period, s)": float(Tu),
            "oscillation_amplitude (a, RPM)": float(a),
            "relay_amplitude (d, PWM)": d,
            "cycles_observed": len(amplitudes),
            "pid_type": pid_type,
        }

        return result
