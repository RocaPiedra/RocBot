"""Parser for ESP32 debug firmware serial output."""

import re
import time
from typing import Optional

from .transport.base import MotorState, ControllerState


def parse_debug_line(line: str, start_time: float) -> Optional[ControllerState]:
    """
    Parse debug line format:
    --- DEBUG T:12345 --- PID Tgt:30 Kp:1.00 Ki:0.00 Kd:0.00 | FL RPM:28.5 F:28.1 PWM:120.5 Out:12.0 Dir:FWD Pulses:42 | FR RPM:27.3 F:27.0 PWM:118.2 Out:11.8 Dir:FWD Pulses:40 |
    """
    state = ControllerState()

    # Extract elapsed time
    t_match = re.search(r"T:(\d+)", line)
    if t_match:
        state.elapsed_ms = float(t_match.group(1))
    state.timestamp = time.time() - start_time

    # Extract mode
    if " STEP_TEST->" in line:
        state.mode = "STEP_TEST"
        step_match = re.search(r"STEP_TEST->(-?\d+)", line)
        if step_match:
            state.target_value = float(step_match.group(1))
    elif " DIRECT:" in line:
        state.mode = "DIRECT"
        dir_match = re.search(r"DIRECT:(FWD|REV)\s+(-?\d+)", line)
        if dir_match:
            state.target_value = float(dir_match.group(2))
    elif " PID" in line:
        state.mode = "PID"
    elif " STOP" in line:
        state.mode = "STOP"

    # Extract target (Tgt:)
    tgt_match = re.search(r"Tgt:(-?\d+)", line)
    if tgt_match:
        state.target_value = float(tgt_match.group(1))

    # Extract PID params
    kp_match = re.search(r"Kp:([\d.]+)", line)
    ki_match = re.search(r"Ki:([\d.]+)", line)
    kd_match = re.search(r"Kd:([\d.]+)", line)
    if kp_match:
        state.kp = float(kp_match.group(1))
    if ki_match:
        state.ki = float(ki_match.group(1))
    if kd_match:
        state.kd = float(kd_match.group(1))

    # Extract motor states (split by |)
    parts = line.split("|")
    for part in parts:
        part = part.strip()
        if not part or part.startswith("---"):
            continue

        motor = parse_motor_debug_block(part)
        if motor:
            state.motors[motor.motor_id] = motor

    return state if state.motors else None


def parse_motor_debug_block(block: str) -> Optional[MotorState]:
    """
    Parse a motor debug block:
    FL RPM:28.5 F:28.1 PWM:120.5 Out:12.0 Dir:FWD Pulses:42
    """
    # Must start with motor ID
    id_match = re.match(r"^(FL|FR|BL|BR)\s+", block)
    if not id_match:
        return None

    motor = MotorState()
    motor.motor_id = id_match.group(1)

    # Parse key:value pairs
    rpm_match = re.search(r"RPM:([\d.-]+)", block)
    filt_match = re.search(r"F:([\d.-]+)", block)
    pwm_match = re.search(r"PWM:([\d.-]+)", block)
    out_match = re.search(r"Out:([\d.-]+)", block)
    dir_match = re.search(r"Dir:(FWD|REV|STP)", block)
    pulse_match = re.search(r"Pulses:(-?\d+)", block)

    if rpm_match:
        motor.rpm = float(rpm_match.group(1))
    if filt_match:
        motor.rpm_filt = float(filt_match.group(1))
    if pwm_match:
        motor.pwr_filt = float(pwm_match.group(1))
    if out_match:
        motor.pwr = float(out_match.group(1))
    if dir_match:
        motor.direction = dir_match.group(1)
    if pulse_match:
        motor.pulses = int(pulse_match.group(1))

    return motor


def parse_motor_state_block(block: str) -> Optional[MotorState]:
    """
    Parse motor state block from 'g' command:
    FL;target:30;rpm:28.5;rpm_filt:28.1;pwr:120.5;pwr_filt:118.2
    """
    parts = block.split(";")
    if not parts:
        return None

    motor_id = parts[0].strip()
    if motor_id not in ("FL", "FR", "BL", "BR"):
        return None

    motor = MotorState()
    motor.motor_id = motor_id

    for part in parts[1:]:
        if ":" in part:
            key, value = part.split(":", 1)
            key = key.strip()
            try:
                val = float(value.strip())
                if key == "target":
                    motor.target_rpm = val
                elif key == "rpm":
                    motor.rpm = val
                elif key == "rpm_filt":
                    motor.rpm_filt = val
                elif key == "pwr":
                    motor.pwr = val
                elif key == "pwr_filt":
                    motor.pwr_filt = val
            except ValueError:
                pass

    return motor
