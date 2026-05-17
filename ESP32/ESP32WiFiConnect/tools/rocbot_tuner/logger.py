"""CSV logging for RocBot motor data."""

import csv
import os
from datetime import datetime
from typing import Optional

from rocbot_tuner.models import ControllerState


class DataLogger:
    """Logs motor data to CSV files."""

    def __init__(self, log_dir: Optional[str] = None):
        if log_dir is None:
            log_dir = os.path.join(os.path.dirname(__file__), "logs")
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)

        self._file = None
        self._writer = None
        self._current_file: Optional[str] = None

    def start_session(self) -> str:
        """Start a new logging session. Returns the log file path."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._current_file = os.path.join(self.log_dir, f"motor_log_{timestamp}.csv")

        self._file = open(self._current_file, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "timestamp", "elapsed_ms", "mode",
            "kp", "ki", "kd",
            "motor_id", "target_rpm", "rpm", "rpm_filt",
            "pwr", "pwr_filt", "error", "direction", "pulses"
        ])
        self._file.flush()

        print(f"Logging to: {self._current_file}")
        return self._current_file

    def log(self, state: ControllerState):
        """Log a ControllerState to CSV."""
        if not self._writer:
            return

        for motor_id, motor in state.motors.items():
            self._writer.writerow([
                f"{state.timestamp:.3f}",
                f"{state.elapsed_ms:.1f}",
                state.mode,
                f"{state.kp:.4f}",
                f"{state.ki:.4f}",
                f"{state.kd:.4f}",
                motor_id,
                f"{motor.target_rpm:.1f}",
                f"{motor.rpm:.1f}",
                f"{motor.rpm_filt:.1f}",
                f"{motor.pwr:.1f}",
                f"{motor.pwr_filt:.1f}",
                f"{motor.error:.1f}",
                motor.direction,
                motor.pulses
            ])

        # Flush every 10 records to avoid I/O bottleneck
        if self._file:
            self._file.flush()

    def close(self):
        """Close the log file."""
        if self._file:
            self._file.close()
            self._file = None
            self._writer = None
            print(f"Log saved: {self._current_file}")

    @property
    def is_logging(self) -> bool:
        return self._file is not None

    @property
    def current_file(self) -> Optional[str]:
        return self._current_file
