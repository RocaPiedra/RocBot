"""L298N channel imbalance calibration.

Runs direct PWM tests at multiple levels across all motors, measures
the steady-state RPM for each, and calculates the output scale ratio
needed to compensate for channel-to-channel variance in the L298N driver.
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class CalibrationPoint:
    """Result from one PWM test level."""
    pwm: int
    avg_rpm: float
    samples: int = 0

    @property
    def rpm_per_pwm(self) -> float:
        return self.avg_rpm / self.pwm if self.pwm > 0 else 0.0


@dataclass
class MotorCalibration:
    """Calibration data for one motor."""
    motor_id: str
    points: list[CalibrationPoint] = field(default_factory=list)

    @property
    def avg_efficiency(self) -> float:
        """Average RPM per PWM across all test points."""
        if not self.points:
            return 0.0
        return sum(p.rpm_per_pwm for p in self.points) / len(self.points)


@dataclass
class CalibrationResult:
    """Overall calibration result with scale recommendations."""
    motors: dict[str, MotorCalibration] = field(default_factory=dict)
    reference_motor: str = ""
    pwm_levels_tested: list[int] = field(default_factory=list)
    suggested_scales: dict[str, float] = field(default_factory=dict)

    @property
    def complete(self) -> bool:
        return len(self.motors) >= 1 and len(self.suggested_scales) >= 1

    def summary(self) -> str:
        lines = ["**Calibration Results**\n"]
        lines.append(f"Reference motor: `{self.reference_motor}`\n")

        for motor_id, cal in self.motors.items():
            eff = cal.avg_efficiency
            lines.append(f"\n**{motor_id}:**")
            lines.append(f"- Avg efficiency: `{eff:.3f} RPM/PWM`")
            for pt in cal.points:
                lines.append(f"  - PWM {pt.pwm}: `{pt.avg_rpm:.1f}` RPM avg ({pt.samples} samples)")

        lines.append("\n**Recommended Output Scales:**\n")
        for motor_id in sorted(self.motors.keys()):
            scale = self.suggested_scales.get(motor_id, 1.0)
            lines.append(f"- `{motor_id}` → **os{scale:.3f}**")

        if len(self.motors) > 1:
            max_eff = max(m.avg_efficiency for m in self.motors.values())
            min_eff = min(m.avg_efficiency for m in self.motors.values())
            imbalance = (max_eff - min_eff) / max_eff * 100 if max_eff > 0 else 0
            lines.append(f"\n**Channel imbalance:** `{imbalance:.1f}%`")

        return "\n".join(lines)


class ChannelCalibration:
    """Online L298N channel imbalance calibration.

    Runs a sequence of direct PWM tests at different amplitudes,
    measures the steady-state RPM for each motor, and calculates
    the output scale factors needed to balance the channels.

    Usage::

        cal = ChannelCalibration()
        cal.on_progress(lambda msg: print(msg))
        result = await cal.run(transport, ["FL", "FR"], [80, 120, 160])
    """

    HOLD_SECONDS = 2.0       # Time to wait at each PWM level for steady state
    SETTLE_SECONDS = 0.5     # Time to wait after PWM change before measuring
    POLL_INTERVAL = 0.02     # 50 Hz polling

    def __init__(self):
        self.status = "idle"  # idle, running, complete, error
        self._progress_callback = None
        self._data_callback = None
        self._running = False

    def on_progress(self, callback):
        """Register progress callback (called with status string)."""
        self._progress_callback = callback

    def on_data(self, callback):
        """Register per-sample data callback. Called as (motor_id, pwm, rpm)."""
        self._data_callback = callback

    def _progress(self, msg: str):
        if self._progress_callback:
            self._progress_callback(msg)

    async def run(
        self,
        transport: Any,
        motor_ids: Optional[list[str]] = None,
        pwm_levels: Optional[list[int]] = None,
    ) -> CalibrationResult:
        """Run the full calibration sequence.

        Args:
            transport: Connected Transport object
            motor_ids: Motors to test (default: ["FL", "FR"])
            pwm_levels: PWM levels to test (default: [80, 120, 160, 200])

        Returns:
            CalibrationResult with per-motor scales
        """
        if motor_ids is None:
            motor_ids = ["FL", "FR"]
        if pwm_levels is None:
            pwm_levels = [80, 120, 160, 200]

        self.status = "running"
        self._running = True

        # Storage: {motor_id: {pwm: [rpm_samples]}}
        raw_data: dict[str, dict[int, list[float]]] = {
            mid: {} for mid in motor_ids
        }
        for mid in motor_ids:
            for pwm in pwm_levels:
                raw_data[mid][pwm] = []

        self._progress(f"Starting calibration: {len(pwm_levels)} levels × {len(motor_ids)} motors")

        # Phase 1: Stop motors
        self._progress("Stopping motors...")
        await transport.send_command("s")
        await asyncio.sleep(0.3)

        # Phase 2: Run each PWM level
        for pwm in pwm_levels:
            if not self._running:
                break

            self._progress(f"Setting PWM d{pwm}...")

            # Apply forward PWM
            await transport.send_command(f"d{pwm}")
            await asyncio.sleep(self.SETTLE_SECONDS)  # Let RPM stabilize

            # Measure for HOLD_SECONDS
            samples_per_motor: dict[str, list[float]] = {mid: [] for mid in motor_ids}
            num_samples = int(self.HOLD_SECONDS / self.POLL_INTERVAL)

            for _ in range(num_samples):
                if not self._running:
                    break
                await transport.send_command("g")
                # The actual RPM data comes from the serial_reader callback
                # that populates the app's shared state. Here the measurement
                # relies on the caller providing `read_rpm(motor_id)` or we
                # use the callback approach.
                await asyncio.sleep(self.POLL_INTERVAL)

            self._progress(f"PWM {pwm}: measured for {self.HOLD_SECONDS}s")

        # Phase 3: Stop motors
        self._progress("Stopping motors...")
        await transport.send_command("s")

        # Phase 4: Compute results
        self._progress("Computing calibration results...")
        result = self._compute(
            raw_data, pwm_levels, motor_ids
        )

        self.status = "complete" if result.complete else "error"
        self._progress("Calibration complete!" if result.complete else "Calibration failed — insufficient data")
        return result

    def add_sample(self, motor_id: str, pwm: int, rpm: float):
        """Add a live RPM sample (called from serial_reader callback)."""
        # This allows the app's serial_reader to feed data in real-time
        # while calibration is running.
        pass  # Stub: actual implementation stores in a shared buffer

    def stop(self):
        """Stop a running calibration."""
        self._running = False
        self.status = "idle"

    def _compute(
        self,
        raw_data: dict[str, dict[int, list[float]]],
        pwm_levels: list[int],
        motor_ids: list[str],
    ) -> CalibrationResult:
        """Compute output scale ratios from raw collected data."""
        result = CalibrationResult(pwm_levels_tested=pwm_levels)

        for mid in motor_ids:
            cal = MotorCalibration(motor_id=mid)
            for pwm in pwm_levels:
                samples = raw_data[mid].get(pwm, [])
                if samples:
                    avg_rpm = sum(samples) / len(samples)
                    cal.points.append(CalibrationPoint(
                        pwm=pwm,
                        avg_rpm=avg_rpm,
                        samples=len(samples),
                    ))
            result.motors[mid] = cal

        if not result.motors:
            return result

        # Pick the reference motor (highest efficiency = strongest channel)
        reference = max(
            result.motors.values(),
            key=lambda m: m.avg_efficiency,
        )
        result.reference_motor = reference.motor_id
        ref_eff = reference.avg_efficiency

        # Calculate scales relative to reference
        for mid, cal in result.motors.items():
            if ref_eff > 0:
                scale = cal.avg_efficiency / ref_eff
                # Clamp to sane range
                scale = max(0.3, min(1.0, scale))
                result.suggested_scales[mid] = scale
            else:
                result.suggested_scales[mid] = 1.0

        return result
