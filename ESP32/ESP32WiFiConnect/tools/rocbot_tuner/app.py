#!/usr/bin/env python3
"""
RocBot PID Tuner - Integrated Motor Control Dashboard

Real-time visualization, PID tuning, channel calibration, and
plugable PID auto-tuning via the strategy pattern.

Supports Serial (USB) and ROS2 (WiFi/UDP micro-ROS) transports.

Usage:
    python app.py                     # Default web UI on :8080
    python app.py --port /dev/ttyUSB0  # Default serial device
"""

import asyncio
import sys
import os
import time
import subprocess
import signal
from collections import deque
from datetime import datetime
from typing import Optional, Any

# Add parent dir (tools/) to path so rocbot_tuner is importable as a package
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nicegui import ui, app
import numpy as np

from rocbot_tuner.transport.serial import SerialTransport
from rocbot_tuner.transport.ros2 import Ros2Transport
from rocbot_tuner.models import ControllerState, MotorState
from rocbot_tuner.parser import parse_debug_line
from rocbot_tuner.logger import DataLogger
from rocbot_tuner.metrics import StepResponseMetrics, calculate_step_response
from rocbot_tuner.calibration import ChannelCalibration, CalibrationResult
from rocbot_tuner.tuning import (
    TuningMethod, TuningResult, TuningPhase,
    list_methods, get_method,
)

# ─── Configuration ───────────────────────────────────────────────────────

MAX_PLOT_POINTS = 500
PLOT_UPDATE_MS = 50

MOTOR_COLORS = {
    "FL": {"target": "#00f0ff", "rpm": "#00f0ff", "rpm_filt": "#00f0ff", "pwr": "#00f0ff"},
    "FR": {"target": "#ff00ff", "rpm": "#ff00ff", "rpm_filt": "#ff00ff", "pwr": "#ff00ff"},
    "BL": {"target": "#39ff14", "rpm": "#39ff14", "rpm_filt": "#39ff14", "pwr": "#39ff14"},
    "BR": {"target": "#ff9f1c", "rpm": "#ff9f1c", "rpm_filt": "#ff9f1c", "pwr": "#ff9f1c"},
}

# ─── Data Buffers ────────────────────────────────────────────────────────


class MotorBuffer:
    """Sliding window buffer for one motor's data."""

    def __init__(self, maxlen: int = MAX_PLOT_POINTS):
        self.maxlen = maxlen
        self.timestamps: deque[float] = deque(maxlen=maxlen)
        self.target_rpm: deque[float] = deque(maxlen=maxlen)
        self.rpm: deque[float] = deque(maxlen=maxlen)
        self.rpm_filt: deque[float] = deque(maxlen=maxlen)
        self.pwr: deque[float] = deque(maxlen=maxlen)
        self.pwr_filt: deque[float] = deque(maxlen=maxlen)
        self.error: deque[float] = deque(maxlen=maxlen)

    def update(self, motor: MotorState, timestamp: float):
        self.timestamps.append(timestamp)
        self.target_rpm.append(motor.target_rpm)
        self.rpm.append(motor.rpm)
        self.rpm_filt.append(motor.rpm_filt)
        self.pwr.append(motor.pwr)
        self.pwr_filt.append(motor.pwr_filt)
        self.error.append(motor.target_rpm - motor.rpm)

    def clear(self):
        self.timestamps.clear()
        self.target_rpm.clear()
        self.rpm.clear()
        self.rpm_filt.clear()
        self.pwr.clear()
        self.pwr_filt.clear()
        self.error.clear()


# ─── Application State ───────────────────────────────────────────────────


class AppState:
    """Global application state."""

    def __init__(self):
        self.transport: Optional[Any] = None
        self.logger = DataLogger()
        self.buffers: dict[str, MotorBuffer] = {}
        self.connected = False
        self.logging = False
        self.log_file: Optional[str] = None

        # PID params
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.output_scale = 10.0

        # Control
        self.target_rpm = 30
        self.direct_pwm = 100
        self.mode = "STOP"

        # Step test
        self.step_test_active = False
        self.step_test_start = 0.0
        self.step_test_target = 30
        self.step_test_duration = 5.0
        self.step_test_metrics: Optional[StepResponseMetrics] = None
        self.step_test_buffers: dict[str, MotorBuffer] = {}

        # Latest controller state
        self.latest_state: Optional[ControllerState] = None
        self.start_time = 0.0

        # Agent management
        self.agent_process: Optional[subprocess.Popen] = None
        self.agent_running = False
        self.serial_port = "/dev/ttyUSB0"

        # Calibration state
        self.calibration = ChannelCalibration()

        # Tuning state
        self.tuning_method_id = "ziegler_nichols"
        self.tuning_target_rpm = 60
        self.tuning_result: Optional[TuningResult] = None
        self.tuning_running = False

    def get_buffer(self, motor_id: str) -> MotorBuffer:
        if motor_id not in self.buffers:
            self.buffers[motor_id] = MotorBuffer()
        return self.buffers[motor_id]


state = AppState()

# ─── ECharts Configuration ──────────────────────────────────────────────


def _chart_theme() -> dict:
    """Common dark/neon theme for ECharts."""
    return {
        "backgroundColor": "transparent",
        "textStyle": {"fontFamily": "monospace", "color": "#e0e0e0"},
        "tooltip": {
            "trigger": "axis",
            "backgroundColor": "#0f0f14",
            "borderColor": "#1a1a2e",
            "textStyle": {"color": "#e0e0e0"},
        },
        "legend": {"data": [], "top": 5, "textStyle": {"color": "#8888a0"}},
    }


def build_rpm_chart() -> dict:
    theme = _chart_theme()
    theme.update({
        "grid": {"left": 50, "right": 20, "top": 35, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "nameTextStyle": {"color": "#8888a0"},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#8888a0"},
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "RPM",
            "nameTextStyle": {"color": "#8888a0"},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#8888a0"},
            "splitLine": {"lineStyle": {"type": "dashed", "color": "#1a1a2e"}},
        },
        "series": [],
    })
    return theme


def build_pwr_chart() -> dict:
    theme = _chart_theme()
    theme.update({
        "grid": {"left": 50, "right": 20, "top": 35, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "nameTextStyle": {"color": "#8888a0"},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#8888a0"},
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "PWM",
            "min": 0,
            "max": 280,
            "nameTextStyle": {"color": "#8888a0"},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#8888a0"},
            "splitLine": {"lineStyle": {"type": "dashed", "color": "#1a1a2e"}},
        },
        "series": [],
    })
    return theme


def build_error_chart() -> dict:
    theme = _chart_theme()
    theme.update({
        "grid": {"left": 50, "right": 20, "top": 35, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "nameTextStyle": {"color": "#8888a0"},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#8888a0"},
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "Error (RPM)",
            "nameTextStyle": {"color": "#8888a0"},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#8888a0"},
            "splitLine": {"lineStyle": {"type": "dashed", "color": "#1a1a2e"}},
        },
        "series": [],
    })
    return theme


# ─── Cyberpunk Theme ─────────────────────────────────────────────────────

ui.add_css("""
/* Global cyberpunk theme overrides */
body { background-color: #050508 !important; font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace !important; }
.q-dark { background-color: #0f0f14 !important; }
.q-drawer { background-color: #0f0f14 !important; border-right: 1px solid #1a1a2e !important; }
.q-header { background-color: #0f0f14 !important; border-bottom: 1px solid #1a1a2e !important; box-shadow: 0 0 15px rgba(0, 240, 255, 0.08) !important; }
.q-card { background-color: #0f0f14 !important; border: 1px solid #1a1a2e !important; }
.q-field__native { color: #e0e0e0 !important; }
.q-field__label { color: #8888a0 !important; }
.q-field--outlined .q-field__control { border-color: #1a1a2e !important; }
.q-field--outlined.q-field--focused .q-field__control { border-color: #00f0ff !important; box-shadow: 0 0 6px rgba(0, 240, 255, 0.25) !important; }
.q-separator { background-color: #1a1a2e !important; }
.q-btn { text-transform: uppercase; letter-spacing: 0.08em; font-family: monospace; font-weight: 600; }
.q-menu { background-color: #0f0f14 !important; border: 1px solid #1a1a2e !important; }
/* Scrollbar */
::-webkit-scrollbar { width: 5px; }
::-webkit-scrollbar-track { background: #050508; }
::-webkit-scrollbar-thumb { background: #1a1a2e; border-radius: 2px; }
::-webkit-scrollbar-thumb:hover { background: #00f0ff; }
/* Tab styling */
.q-tab { text-transform: uppercase; letter-spacing: 0.1em; font-weight: 600; }
.q-tab--active { color: #00f0ff !important; }
.q-tabs__indicator { background: #00f0ff !important; }
""")

# ─── UI Components ───────────────────────────────────────────────────────

# ECharts instances
rpm_chart: Optional[ui.echart] = None
pwr_chart: Optional[ui.echart] = None
error_chart: Optional[ui.echart] = None

# Status indicators
status_label: Optional[ui.label] = None
mode_label: Optional[ui.label] = None
log_label: Optional[ui.label] = None
agent_status: Optional[ui.label] = None

# PID sliders
kp_slider: Optional[ui.number] = None
ki_slider: Optional[ui.number] = None
kd_slider: Optional[ui.number] = None
os_slider: Optional[ui.number] = None

# Motor state cards
motor_cards: dict[str, dict] = {}

# Metrics display
metrics_label: Optional[ui.markdown] = None

# Calibration UI refs
cal_status: Optional[ui.label] = None
cal_results: Optional[ui.markdown] = None
cal_progress: Optional[ui.linear_progress] = None

# Tuning UI refs
tuning_status: Optional[ui.label] = None
tuning_result_label: Optional[ui.markdown] = None
tuning_phases: Optional[ui.markdown] = None
tuning_params_container: Any = None

# ─── Chart Update ────────────────────────────────────────────────────────


def update_charts():
    """Update all three ECharts with current buffer data."""
    if not rpm_chart:
        return

    rpm_option = build_rpm_chart()
    pwr_option = build_pwr_chart()
    error_option = build_error_chart()

    for motor_id, buf in state.buffers.items():
        if not buf.timestamps:
            continue
        colors = MOTOR_COLORS.get(motor_id, MOTOR_COLORS["FL"])
        time_labels = [f"{t:.1f}" for t in buf.timestamps]

        rpm_option["legend"]["data"].append(f"{motor_id} Target")
        rpm_option["legend"]["data"].append(f"{motor_id} RPM")
        rpm_option["series"].append({
            "name": f"{motor_id} Target",
            "type": "line",
            "data": list(buf.target_rpm),
            "lineStyle": {"type": "dashed", "width": 1},
            "itemStyle": {"color": colors["target"]},
            "symbol": "none",
        })
        rpm_option["series"].append({
            "name": f"{motor_id} RPM",
            "type": "line",
            "data": [round(v, 1) for v in buf.rpm_filt],
            "lineStyle": {"width": 2},
            "itemStyle": {"color": colors["rpm_filt"]},
            "symbol": "none",
        })

        pwr_option["legend"]["data"].append(f"{motor_id} PWM")
        pwr_option["series"].append({
            "name": f"{motor_id} PWM",
            "type": "line",
            "data": [round(v, 1) for v in buf.pwr_filt],
            "lineStyle": {"width": 2},
            "itemStyle": {"color": colors["pwr"]},
            "symbol": "none",
        })

        error_option["legend"]["data"].append(f"{motor_id} Error")
        error_option["series"].append({
            "name": f"{motor_id} Error",
            "type": "line",
            "data": [round(v, 1) for v in buf.error],
            "lineStyle": {"width": 2},
            "itemStyle": {"color": colors["rpm"]},
            "symbol": "none",
        })

        rpm_option["xAxis"]["data"] = time_labels
        pwr_option["xAxis"]["data"] = time_labels
        error_option["xAxis"]["data"] = time_labels

    rpm_chart.options.clear()
    rpm_chart.options.update(rpm_option)
    rpm_chart.update()

    pwr_chart.options.clear()
    pwr_chart.options.update(pwr_option)
    pwr_chart.update()

    error_chart.options.clear()
    error_chart.options.update(error_option)
    error_chart.update()


def update_motor_cards():
    """Update motor state display cards."""
    if not state.latest_state:
        return

    for motor_id, motor in state.latest_state.motors.items():
        if motor_id in motor_cards:
            cards = motor_cards[motor_id]
            cards["rpm"].set_text(f"RPM: {motor.rpm:.1f}")
            cards["filt"].set_text(f"Filtered RPM: {motor.rpm_filt:.1f}")
            cards["pwm"].set_text(f"PWM: {motor.pwr_filt:.1f}")
            cards["dir"].set_text(f"Dir: {motor.direction}")
            cards["err"].set_text(f"Error: {motor.error:.1f}")


# ─── Serial Reader Task ──────────────────────────────────────────────────


async def serial_reader():
    """Background task that reads from serial and updates UI."""
    def on_state(new_state: ControllerState):
        state.latest_state = new_state

        # Update live buffers (dashboard)
        for motor_id, motor in new_state.motors.items():
            buf = state.get_buffer(motor_id)
            buf.update(motor, new_state.timestamp)

            # Step test buffer
            if state.step_test_active:
                if motor_id not in state.step_test_buffers:
                    state.step_test_buffers[motor_id] = MotorBuffer(maxlen=5000)
                state.step_test_buffers[motor_id].update(motor, new_state.timestamp)

            # Calibration data feed
            if state.calibration.status == "running":
                _feed_calibration(motor_id, motor)

        # Log if enabled
        if state.logging:
            state.logger.log(new_state)

    try:
        await state.transport.read_loop(on_state)
    except Exception as e:
        print(f"Reader error: {e}")
        state.connected = False
        if status_label:
            status_label.set_text("● Disconnected")
            status_label.style("color: red")


# ─── Calibration Data Feed ───────────────────────────────────────────────

_cal_data: dict[str, dict[int, list[float]]] = {}
_cal_current_pwm: int = 0

def _feed_calibration(motor_id: str, motor: MotorState):
    """Feed live RPM data into the calibration buffer."""
    global _cal_current_pwm
    if motor_id not in _cal_data:
        _cal_data[motor_id] = {}
    if _cal_current_pwm not in _cal_data[motor_id]:
        _cal_data[motor_id][_cal_current_pwm] = []
    _cal_data[motor_id][_cal_current_pwm].append(motor.rpm_filt)


# ─── UI Event Handlers ───────────────────────────────────────────────────


async def connect_serial(transport: str = "serial"):
    """Connect to ESP32 via Serial or ROS2 (WiFi/UDP)."""
    if transport == "ros2":
        if not _is_agent_running():
            ui.notify("Auto-launching micro-ROS agent...", type="info")
            await launch_agent()
            await asyncio.sleep(1.5)

        state.transport = Ros2Transport()
        success = await state.transport.connect()
        if success:
            state.connected = True
            state.start_time = time.time()
            status_label.set_text("● ROS2 (WiFi)")
            status_label.style("color: cyan")
            ui.notify("Connected via ROS2 (WiFi/UDP)")
            asyncio.create_task(serial_reader())
        else:
            status_label.set_text("● ROS2 Failed")
            status_label.style("color: red")
            ui.notify("ROS2 connection failed. Is micro-ROS agent running?", type="negative")
    else:
        port = port_input.value or "/dev/ttyUSB0"
        baud = int(baud_input.value or 115200)
        state.serial_port = port
        state.transport = SerialTransport(port=port, baud=baud)
        success = await state.transport.connect()
        if success:
            state.connected = True
            state.start_time = time.time()
            status_label.set_text("● Connected")
            status_label.style("color: green")
            ui.notify(f"Connected to {port}")
            asyncio.create_task(serial_reader())
        else:
            status_label.set_text("● Failed")
            status_label.style("color: red")
            ui.notify("Connection failed", type="negative")


def _is_agent_running() -> bool:
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", "name=rocbot_microros_agent", "--format", "{{.Names}}"],
            capture_output=True, text=True, check=False,
        )
        return "rocbot_microros_agent" in result.stdout
    except Exception:
        return False


async def launch_agent():
    if _is_agent_running():
        state.agent_running = True
        agent_status.set_text("Agent: ● Running").style("color: cyan")
        ui.notify("Agent already running (detected)", type="warning")
        return

    cmd = [
        "docker", "run", "-d", "--rm",
        "--net=host",
        "--name", "rocbot_microros_agent",
        "microros/micro-ros-agent:humble",
        "udp4", "--port", "8888", "-v6",
    ]

    try:
        state.agent_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        state.agent_running = True
        agent_status.set_text("Agent: ● Running (UDP 8888)").style("color: cyan")
        ui.notify("micro-ROS agent launched (UDP 8888)")
    except Exception as e:
        ui.notify(f"Failed to launch agent: {e}", type="negative")


async def stop_agent():
    if not state.agent_running and not _is_agent_running():
        ui.notify("Agent not running", type="warning")
        return

    try:
        subprocess.run(["docker", "stop", "rocbot_microros_agent"], check=False, capture_output=True)
        state.agent_running = False
        state.agent_process = None
        agent_status.set_text("Agent: ● Stopped").style("color: gray")
        ui.notify("micro-ROS agent stopped")
    except Exception as e:
        ui.notify(f"Failed to stop agent: {e}", type="negative")


async def disconnect_serial():
    if state.transport:
        await state.transport.disconnect()
        state.connected = False
        status_label.set_text("● Disconnected")
        status_label.style("color: red")
        ui.notify("Disconnected")


async def send_pid_params():
    if not state.connected:
        ui.notify("Not connected", type="warning")
        return

    await state.transport.send_command(f"kp{state.kp}")
    await state.transport.send_command(f"ki{state.ki}")
    await state.transport.send_command(f"kd{state.kd}")
    await state.transport.send_command(f"os{state.output_scale}")
    ui.notify(f"PID: Kp={state.kp} Ki={state.ki} Kd={state.kd} OS={state.output_scale}")


async def set_mode_pid():
    if not state.connected:
        return
    await state.transport.send_command("p")
    state.mode = "PID"
    mode_label.set_text("Mode: PID")
    ui.notify("PID mode enabled")


async def set_mode_direct():
    if not state.connected:
        return
    await state.transport.send_command(f"d{state.direct_pwm}")
    state.mode = "DIRECT"
    mode_label.set_text(f"Mode: DIRECT {state.direct_pwm}")
    ui.notify(f"Direct PWM: {state.direct_pwm}")


async def stop_motors():
    if not state.connected:
        return
    await state.transport.send_command("s")
    state.mode = "STOP"
    mode_label.set_text("Mode: STOP")
    ui.notify("Motors stopped")


async def set_target():
    if not state.connected:
        return
    await state.transport.send_command(str(state.target_rpm))
    ui.notify(f"Target: {state.target_rpm} RPM")


async def start_step_test():
    if not state.connected:
        ui.notify("Not connected", type="warning")
        return

    state.step_test_buffers.clear()
    state.step_test_active = True
    state.step_test_start = time.time()
    state.step_test_target = state.target_rpm

    await state.transport.send_command("p")
    await asyncio.sleep(0.1)
    await state.transport.send_command(str(state.step_test_target))

    mode_label.set_text(f"Mode: STEP TEST → {state.step_test_target}")
    ui.notify(f"Step test started: {state.step_test_target} RPM")

    await asyncio.sleep(state.step_test_duration)

    await state.transport.send_command("s")
    state.step_test_active = False
    mode_label.set_text("Mode: STOP (test complete)")

    for motor_id, buf in state.step_test_buffers.items():
        if len(buf.timestamps) > 10:
            timestamps = [t - buf.timestamps[0] for t in buf.timestamps]
            metrics = calculate_step_response(
                timestamps, list(buf.rpm_filt), state.step_test_target
            )
            if motor_id == list(state.step_test_buffers.keys())[0]:
                state.step_test_metrics = metrics

    if state.step_test_metrics:
        metrics_label.set_text(f"### Step Response ({state.step_test_target} RPM)\n\n{state.step_test_metrics.summary()}")
        ui.notify("Step test complete!")
    else:
        metrics_label.set_text("### Step Response\n\nNo data collected")


async def toggle_logging():
    if not state.logging:
        state.log_file = state.logger.start_session()
        state.logging = True
        log_label.set_text(f"Logging: {os.path.basename(state.log_file)}")
        log_label.style("color: green")
        ui.notify("Logging started")
    else:
        state.logger.close()
        state.logging = False
        state.log_file = None
        log_label.set_text("Logging: OFF")
        log_label.style("color: gray")
        ui.notify("Logging stopped")


async def clear_buffers():
    for buf in state.buffers.values():
        buf.clear()
    ui.notify("Buffers cleared")


# ─── Calibration Handlers ────────────────────────────────────────────────


async def run_calibration():
    """Run the channel imbalance calibration."""
    global _cal_data, _cal_current_pwm

    if not state.connected:
        ui.notify("Not connected", type="warning")
        return

    # Read calibration params from UI
    motor_ids = ["FL", "FR"]
    pwm_levels_text = cal_pwm_input.value or "80,120,160,200"
    try:
        pwm_levels = [int(x.strip()) for x in pwm_levels_text.split(",")]
    except ValueError:
        ui.notify("Invalid PWM levels format", type="negative")
        return

    direction = cal_dir_toggle.value  # "forward" or "reverse"

    # Reset data
    _cal_data = {mid: {} for mid in motor_ids}
    _cal_current_pwm = 0

    cal_status.set_text("Status: Running...")
    cal_status.style("color: #ff9f1c")
    cal_results.set_text("Collecting data...")

    total_steps = len(pwm_levels) + 2  # stop + levels + stop + compute
    step = 0

    # Step 1: Stop
    step += 1
    cal_status.set_text(f"[{step}/{total_steps}] Stopping motors...")
    await state.transport.send_command("s")
    await asyncio.sleep(0.3)

    # Step 2-N: Run each PWM level
    for i, pwm in enumerate(pwm_levels):
        step += 1
        _cal_current_pwm = pwm
        cal_status.set_text(f"[{step}/{total_steps}] Testing PWM {pwm} ({direction})...")
        cal_progress.set_value(step / total_steps)

        if direction == "reverse":
            await state.transport.send_command(f"D{pwm}")
        else:
            await state.transport.send_command(f"d{pwm}")

        await asyncio.sleep(0.5)  # settle
        await asyncio.sleep(2.0)  # hold / collect

    # Step N+1: Stop
    step += 1
    cal_status.set_text(f"[{step}/{total_steps}] Stopping...")
    await state.transport.send_command("s")

    # Step N+2: Compute
    step += 1
    cal_status.set_text(f"[{step}/{total_steps}] Computing results...")
    cal_progress.set_value(1.0)

    # Build result from collected data
    from rocbot_tuner.calibration import CalibrationResult, MotorCalibration, CalibrationPoint
    result = CalibrationResult(pwm_levels_tested=pwm_levels)
    for mid in motor_ids:
        cal = MotorCalibration(motor_id=mid)
        data = _cal_data.get(mid, {})
        for pwm in pwm_levels:
            samples = data.get(pwm, [])
            if samples:
                avg_rpm = sum(samples) / len(samples)
                cal.points.append(CalibrationPoint(pwm=pwm, avg_rpm=avg_rpm, samples=len(samples)))
        result.motors[mid] = cal

    # Compute scales
    if result.motors:
        ref = max(result.motors.values(), key=lambda m: m.avg_efficiency)
        result.reference_motor = ref.motor_id
        ref_eff = ref.avg_efficiency
        for mid, cal in result.motors.items():
            if ref_eff > 0:
                scale = cal.avg_efficiency / ref_eff
                scale = max(0.3, min(1.0, scale))
                result.suggested_scales[mid] = scale
            else:
                result.suggested_scales[mid] = 1.0

    cal_results.set_text(result.summary())
    cal_status.set_text("Status: Complete").style("color: #39ff14")
    state.calibration.status = "complete"
    ui.notify("Calibration complete!")

    # Store result for apply button
    state.calibration_result = result


def stop_calibration():
    state.calibration.stop()
    cal_status.set_text("Status: Stopped").style("color: #ff3333")
    ui.notify("Calibration stopped")


async def apply_calibration():
    """Apply the calibration scales to the ESP32."""
    if not state.connected:
        ui.notify("Not connected", type="warning")
        return
    result = getattr(state, "calibration_result", None)
    if not result or not result.complete:
        ui.notify("No calibration results to apply", type="warning")
        return

    for motor_id, scale in result.suggested_scales.items():
        await state.transport.send_command(f"os{scale:.3f}")
        ui.notify(f"{motor_id}: os{scale:.3f}")
    ui.notify("Calibration scales applied!")


# ─── Auto-Tuning Handlers ────────────────────────────────────────────────

_tuning_instance: Optional[TuningMethod] = None

def on_tuning_method_change(method_id: str):
    """Called when tuning method selector changes. Rebuilds params UI."""
    state.tuning_method_id = method_id
    _rebuild_tuning_params()


def _rebuild_tuning_params():
    """Rebuild the method-specific parameters in the tuning UI."""
    global tuning_params_container
    if tuning_params_container is None:
        return

    tuning_params_container.clear()
    with tuning_params_container:
        try:
            method_cls = get_method(state.tuning_method_id)
        except KeyError:
            ui.label("Unknown method").classes("text-[#ff3333] text-xs")
            return

        for param in method_cls.parameters():
            ptype = param.get("type", "float")
            pid = param["id"]
            default = param.get("default", 0)
            label = param.get("label", pid)
            desc = param.get("description", "")

            if ptype == "select":
                opts = param.get("options", [])
                ui.select(
                    label=label, options=opts, value=default,
                    on_change=lambda v, k=pid: setattr(state, f"tuning_{k}", v),
                ).props("dense outlined dark color=cyan").classes("w-full mt-1").tooltip(desc)
            elif ptype == "bool":
                ui.checkbox(label, value=default).bind_value(state, f"tuning_{pid}").classes("text-xs text-[#8888a0] mt-1")
            else:
                min_v = param.get("min", 0)
                max_v = param.get("max", 100)
                step = param.get("step", 1)
                ui.number(
                    value=default, min=min_v, max=max_v, step=step,
                    format="%.2f" if ptype == "float" else "%.0f",
                ).bind_value(state, f"tuning_{pid}").props(
                    f"dense outlined dark label='{label}' color=cyan"
                ).classes("w-full mt-1").tooltip(desc)

    tuning_params_container.update()


async def run_auto_tuning():
    """Run the selected PID auto-tuning method."""
    global _tuning_instance

    if not state.connected:
        ui.notify("Not connected", type="warning")
        return

    motor_id = tuning_motor_selector.value or "FL"
    target_rpm = state.tuning_target_rpm or 60

    # Get the method class and instantiate
    try:
        method_cls = get_method(state.tuning_method_id)
    except KeyError as e:
        ui.notify(str(e), type="negative")
        return

    _tuning_instance = method_cls()
    state.tuning_running = True
    tuning_status.set_text("Status: Running...").style("color: #ff9f1c")
    tuning_result_label.set_text("Running...")
    tuning_phases.set_text("")

    # Build kwargs from UI params
    kwargs = {}
    for param in method_cls.parameters():
        pid = param["id"]
        val = getattr(state, f"tuning_{pid}", param.get("default"))
        kwargs[pid] = val

    # Phase tracking callback
    def on_phase(phase: TuningPhase):
        phases_text = "\n".join(
            f"- {'[OK]' if p.status=='complete' else '[>]' if p.status=='running' else '[XX]' if p.status=='error' else '[ ]'} {p.name}"
            for p in _tuning_instance._phases
        )
        try:
            tuning_phases.set_text(phases_text)
        except Exception:
            pass

    _tuning_instance.on_phase_change(on_phase)

    try:
        result = await _tuning_instance.run(
            transport=state.transport,
            motor_id=motor_id,
            target_rpm=target_rpm,
            **kwargs,
        )
        state.tuning_result = result
        tuning_result_label.set_text(result.summary())
        tuning_status.set_text("Status: Complete").style("color: #39ff14")
        ui.notify(f"Tuning complete! Kp={result.kp:.3f}")
    except Exception as e:
        tuning_status.set_text(f"Status: Error — {e}").style("color: #ff3333")
        tuning_result_label.set_text(f"**Error:** {e}")
        ui.notify(f"Tuning failed: {e}", type="negative")
    finally:
        state.tuning_running = False
        # Stop motors after tuning
        try:
            await state.transport.send_command("s")
        except Exception:
            pass


async def apply_tuning():
    """Apply the auto-tuned PID gains to the ESP32."""
    if not state.connected:
        ui.notify("Not connected", type="warning")
        return
    result = state.tuning_result
    if not result:
        ui.notify("No tuning results to apply", type="warning")
        return

    await state.transport.send_command(f"kp{result.kp}")
    await state.transport.send_command(f"ki{result.ki}")
    await state.transport.send_command(f"kd{result.kd}")
    await state.transport.send_command(f"os{result.output_scale}")
    state.kp = result.kp
    state.ki = result.ki
    state.kd = result.kd
    state.output_scale = result.output_scale
    ui.notify(f"Applied: Kp={result.kp:.3f} Ki={result.ki:.5f} Kd={result.kd:.5f}")


async def stop_tuning():
    """Stop the running tuning procedure."""
    global _tuning_instance
    state.tuning_running = False
    if state.connected:
        await state.transport.send_command("s")
    tuning_status.set_text("Status: Stopped").style("color: #ff3333")
    ui.notify("Tuning stopped")


# ─── Auto-update Timer ───────────────────────────────────────────────────


def auto_update():
    """Called periodically to update UI."""
    update_charts()
    update_motor_cards()


# ─── Main UI Layout ──────────────────────────────────────────────────────

sidebar_open = True


def _btn(color: str, label: str, onclick, cls: str = ""):
    """Create a cyberpunk neon outline button."""
    return ui.button(label, on_click=onclick).props("dense flat size=sm").classes(
        f"border border-[{color}] bg-[{color}]/10 text-[{color}] hover:bg-[{color}]/20 uppercase tracking-wider text-[10px] font-bold {cls}"
    )


def _small_btn(color: str, label: str, onclick, cls: str = ""):
    """Create an extra-small cyberpunk button."""
    return ui.button(label, on_click=onclick).props("dense flat size=xs").classes(
        f"border border-[{color}] bg-[{color}]/10 text-[{color}] hover:bg-[{color}]/20 uppercase tracking-wider text-[9px] font-bold {cls}"
    )


# ─── Header ──────────────────────────────────────────────────────────────

with ui.header().classes("items-center justify-between bg-[#0f0f14] text-[#e0e0e0] px-4 border-b border-[#1a1a2e]"):
    with ui.row().classes("items-center gap-3"):
        ui.button("≡", on_click=lambda: drawer.toggle()).props("flat dense size=sm").classes("text-[#00f0ff]")
        ui.label("◈ ROCBOT PID TUNER").classes(
            "text-lg font-bold tracking-[0.2em] text-[#00f0ff] drop-shadow-[0_0_8px_rgba(0,240,255,0.4)]"
        )
    with ui.row().classes("items-center gap-4"):
        status_label = ui.label("● DISCONNECTED").classes("text-[10px] font-mono tracking-wider")
        mode_label = ui.label("MODE: STOP").classes("text-[10px] font-mono tracking-wider text-[#8888a0]")
        log_label = ui.label("LOG: OFF").classes("text-[10px] font-mono tracking-wider text-[#8888a0]")

# ─── Left Drawer ─────────────────────────────────────────────────────────

with ui.left_drawer(fixed=True).props("bordered").classes("bg-[#0f0f14] w-64 border-r border-[#1a1a2e]") as drawer:
    with ui.scroll_area().classes("fit p-3"):
        # Connection
        ui.label("◄ CONNECTION ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        transport_type = ui.toggle({"serial": "Serial (USB)", "ros2": "ROS2 (WiFi)"}, value="ros2").props("dense color=cyan")
        transport_type.classes("text-[10px]")
        port_input = ui.input(value="/dev/ttyUSB0").classes("w-full mt-1").props("dense outlined dark label='Serial Port' color=cyan")
        baud_input = ui.input(value="115200").classes("w-full mt-1").props("dense outlined dark label='Serial Baud' color=cyan")
        with ui.row().classes("gap-1 mt-1 w-full"):
            _btn("#39ff14", "Connect", lambda: connect_serial(transport_type.value), "flex-1")
            _btn("#ff3333", "Disconnect", disconnect_serial, "flex-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # micro-ROS Agent
        ui.label("◄ AGENT (UDP) ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        agent_status = ui.label("Agent: ○ STOPPED").classes("text-[10px] font-mono text-[#8888a0]")
        with ui.row().classes("gap-1 w-full"):
            _btn("#00f0ff", "Launch", launch_agent, "flex-1")
            _btn("#ff3333", "Stop", stop_agent, "flex-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Mode
        ui.label("◄ MODE ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        with ui.row().classes("gap-1"):
            _btn("#00f0ff", "PID", set_mode_pid, "flex-1")
            _btn("#ff9f1c", "Direct", set_mode_direct, "flex-1")
            _btn("#ff3333", "Stop", stop_motors, "flex-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Target
        ui.label("◄ TARGET ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=30, min=-330, max=330, step=1, format="%.0f").bind_value(state, "target_rpm").props("dense outlined dark label=RPM color=cyan").classes("w-full")
        _btn("#00f0ff", "Set Target", set_target, "w-full mt-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Direct PWM
        ui.label("◄ DIRECT PWM ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=100, min=0, max=255, step=1, format="%.0f").bind_value(state, "direct_pwm").props("dense outlined dark label=PWM color=orange").classes("w-full")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # PID Parameters
        ui.label("◄ PID GAINS ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=1.0, min=0, max=100, step=0.1, format="%.2f").bind_value(state, "kp").props("dense outlined dark label=Kp color=cyan").classes("w-full")
        ui.number(value=0.0, min=0, max=10, step=0.01, format="%.3f").bind_value(state, "ki").props("dense outlined dark label=Ki color=cyan").classes("w-full mt-1")
        ui.number(value=0.0, min=0, max=10, step=0.01, format="%.3f").bind_value(state, "kd").props("dense outlined dark label=Kd color=cyan").classes("w-full mt-1")
        ui.number(value=10.0, min=1, max=100, step=1, format="%.0f").bind_value(state, "output_scale").props("dense outlined dark label='Out Scale' color=cyan").classes("w-full mt-1")
        _btn("#00f0ff", "Apply PID", send_pid_params, "w-full mt-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Step Test
        ui.label("◄ STEP TEST ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=5.0, min=1, max=30, step=0.5, format="%.1f").bind_value(state, "step_test_duration").props("dense outlined dark label=Duration color=orange").classes("w-full")
        _btn("#ff9f1c", "Run Test", start_step_test, "w-full mt-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Utilities
        ui.label("◄ UTILITIES ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        with ui.row().classes("gap-1"):
            _btn("#8888a0", "Log", toggle_logging, "flex-1")
            _btn("#ff3333", "Clear", clear_buffers, "flex-1")

# ─── Main Content with Tabs ──────────────────────────────────────────────

with ui.column().classes("w-full flex-1 p-3 gap-0"):
    # Tabs
    tabs = ui.tabs().classes("w-full border-b border-[#1a1a2e]")
    ui.tab("Dashboard", icon="dashboard")
    ui.tab("Calibration", icon="tune")
    ui.tab("Auto-Tuning", icon="auto_fix_high")

    # Tab panels
    with ui.tab_panels(tabs, value="Dashboard").classes("w-full flex-1"):
        # ── Dashboard Tab ──────────────────────────────────────────────────
        with ui.tab_panel("Dashboard").classes("p-2 gap-3"):
            # Motor status bar
            with ui.row().classes("w-full gap-3"):
                for motor_id in ["FL", "FR"]:
                    accent = "#00f0ff" if motor_id == "FL" else "#ff00ff"
                    with ui.card().classes(
                        f"flex-1 p-3 bg-[#0f0f14] border border-[{accent}]/30 shadow-[0_0_10px_rgba({'0,240,255' if motor_id == 'FL' else '255,0,255'},0.05)]"
                    ):
                        with ui.row().classes("w-full items-center"):
                            ui.label(f"[{motor_id}]").classes(f"text-sm font-bold mr-4 w-10 font-mono text-[{accent}]")
                            motor_cards[motor_id] = {
                                "rpm": ui.label("RPM: 0.0").classes("text-[10px] w-24 font-mono text-[#e0e0e0]"),
                                "filt": ui.label("F: 0.0").classes("text-[10px] w-24 font-mono text-[#8888a0]"),
                                "pwm": ui.label("PWM: 0.0").classes("text-[10px] w-24 font-mono text-[#e0e0e0]"),
                                "dir": ui.label("DIR: STP").classes("text-[10px] w-20 font-mono text-[#8888a0]"),
                                "err": ui.label("ERR: 0.0").classes("text-[10px] font-mono text-[#8888a0]"),
                            }

            # Charts
            with ui.column().classes("w-full gap-2"):
                with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e]"):
                    rpm_chart = ui.echart(build_rpm_chart()).classes("w-full h-80")
                with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e]"):
                    pwr_chart = ui.echart(build_pwr_chart()).classes("w-full h-64")
                with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e]"):
                    error_chart = ui.echart(build_error_chart()).classes("w-full h-64")

            # Metrics
            with ui.card().classes("w-full p-2 bg-[#0f0f14] border border-[#1a1a2e]"):
                with ui.row().classes("w-full items-center"):
                    ui.label("◄ STEP RESPONSE ►").classes("text-[10px] font-bold mr-4 text-[#8888a0] tracking-[0.15em]")
                    metrics_label = ui.markdown("Run a step test to see metrics.").classes("text-[10px] flex-1 font-mono text-[#8888a0]")

        # ── Calibration Tab ────────────────────────────────────────────────
        with ui.tab_panel("Calibration").classes("p-2 gap-3"):
            with ui.row().classes("w-full gap-3"):
                # Left: controls
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ CALIBRATION SETTINGS ►").classes(
                        "text-[10px] font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )

                    ui.label("This test measures L298N channel imbalance by running each motor at multiple PWM levels and computing output scale ratios.").classes(
                        "text-[10px] text-[#8888a0] mb-2"
                    )

                    ui.select(label="Motors", options=["FL,FR", "FL,FR,BL,BR", "FL", "FR"], value="FL,FR").props(
                        "dense outlined dark color=cyan"
                    ).classes("w-full mt-1").bind_value(state, "cal_motors")

                    cal_pwm_input = ui.input(
                        value="80,120,160,200",
                    ).classes("w-full mt-1").props("dense outlined dark label='PWM Levels' color=cyan")

                    cal_dir_toggle = ui.toggle(
                        {"forward": "Forward (d)", "reverse": "Reverse (D)"},
                        value="forward",
                    ).props("dense color=cyan").classes("text-[10px] mt-1")

                    with ui.row().classes("gap-1 mt-2 w-full"):
                        _btn("#ff9f1c", "Run Calibration", run_calibration, "flex-1")
                        _btn("#ff3333", "Stop", stop_calibration, "flex-1")

                    cal_status = ui.label("Status: Idle").classes("text-[10px] font-mono mt-1 text-[#8888a0]")
                    cal_progress = ui.linear_progress(value=0).classes("mt-1").props("color=cyan")

                # Right: results
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ RESULTS ►").classes(
                        "text-[10px] font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )
                    cal_results = ui.markdown("Run a calibration to see results.").classes(
                        "text-[10px] font-mono text-[#8888a0]"
                    )
                    _btn("#39ff14", "Apply Scales to ESP32", apply_calibration, "w-full mt-2")

            with ui.card().classes("w-full p-2 bg-[#0f0f14] border border-[#1a1a2e]"):
                ui.label("◄ ABOUT ►").classes("text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
                ui.markdown(
                    "L298N dual-H-bridge channels have slightly different output characteristics due to "
                    "transistor mismatch and PCB layout. This calibration measures the RPM/PWM efficiency "
                    "of each motor and computes `os<scale>` values so both channels produce equal speed at "
                    "the same command. The reference motor (highest efficiency) stays at os=1.0 and others "
                    "are scaled down to match."
                ).classes("text-[10px] text-[#8888a0]")

        # ── Auto-Tuning Tab ────────────────────────────────────────────────
        with ui.tab_panel("Auto-Tuning").classes("p-2 gap-3"):
            with ui.row().classes("w-full gap-3"):
                # Left: controls
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ TUNING CONTROLS ►").classes(
                        "text-[10px] font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )

                    ui.markdown(
                        "Select a tuning method and its parameters below. "
                        "The strategy pattern makes it trivial to add new algorithms."
                    ).classes("text-[10px] text-[#8888a0] mb-1")

                    # Method selector
                    method_options = {m["id"]: f"{m['name']} — {m['description']}" for m in list_methods()}
                    tuning_method_selector = ui.select(
                        label="Tuning Method",
                        options=method_options,
                        value=state.tuning_method_id,
                        on_change=lambda e: on_tuning_method_change(e.value),
                    ).props("dense outlined dark color=cyan").classes("w-full")

                    make_tuning_params_container()

                    # Target RPM
                    ui.number(
                        value=state.tuning_target_rpm, min=20, max=200, step=10, format="%.0f",
                    ).bind_value(state, "tuning_target_rpm").props(
                        "dense outlined dark label='Target RPM' color=cyan"
                    ).classes("w-full mt-2")

                    # Motor selector
                    tuning_motor_selector = ui.select(
                        label="Motor", options=["FL", "FR", "BL", "BR"], value="FL",
                    ).props("dense outlined dark color=cyan").classes("w-full mt-1")

                    # Action buttons
                    with ui.row().classes("gap-1 mt-2 w-full"):
                        _btn("#ff9f1c", "Run Tuning", run_auto_tuning, "flex-1")
                        _btn("#ff3333", "Stop", stop_tuning, "flex-1")

                    tuning_status = ui.label("Status: Idle").classes("text-[10px] font-mono mt-1 text-[#8888a0]")

                # Right: results + progress
                with ui.column().classes("flex-1 gap-2"):
                    # Progress phases
                    with ui.card().classes("w-full p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                        ui.label("◄ PROGRESS ►").classes(
                            "text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]"
                        )
                        tuning_phases = ui.markdown("No tuning running.").classes(
                            "text-[10px] font-mono text-[#8888a0]"
                        )

                    # Results
                    with ui.card().classes("w-full p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                        ui.label("◄ RESULTS ►").classes(
                            "text-[10px] font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]"
                        )
                        tuning_result_label = ui.markdown("Run auto-tuning to see results.").classes(
                            "text-[10px] font-mono text-[#8888a0]"
                        )
                        _btn("#39ff14", "Apply Gains to ESP32", apply_tuning, "w-full mt-2")


def make_tuning_params_container():
    """Create the container for dynamic method parameters."""
    global tuning_params_container
    if tuning_params_container is not None:
        return
    # We'll create it inside the tuning controls card after the method selector
    # Use a placeholder approach: create a container that gets populated later
    tuning_params_container = ui.column().classes("w-full gap-0")
    _rebuild_tuning_params()


# ─── Auto-update timer ───────────────────────────────────────────────────

ui.timer(PLOT_UPDATE_MS / 1000, auto_update)

# ─── Startup Check ───────────────────────────────────────────────────────

if _is_agent_running():
    state.agent_running = True
    agent_status.set_text("Agent: ● Running (UDP 8888)").style("color: cyan")

# ─── Launch ──────────────────────────────────────────────────────────────


def main():
    import argparse
    parser = argparse.ArgumentParser(description="RocBot PID Tuner")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--host", default="0.0.0.0", help="Web server host")
    parser.add_argument("--port-web", type=int, default=8080, help="Web server port")
    args = parser.parse_args()

    port_input.value = args.port
    baud_input.value = str(args.baud)

    ui.run(
        host=args.host,
        port=args.port_web,
        title="RocBot PID Tuner",
        favicon="🤖",
        dark=True,
        reload=False,
    )


if __name__ in {"__main__", "__mp_main__"}:
    main()
