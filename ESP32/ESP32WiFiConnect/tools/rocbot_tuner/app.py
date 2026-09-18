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
from rocbot_tuner.transport.ros2 import Ros2Transport, HAS_RCLPY
from rocbot_tuner.models import ControllerState, MotorState
from rocbot_tuner.parser import parse_debug_line
from rocbot_tuner.logger import DataLogger
from rocbot_tuner.commlog import CommLogger, AgentStatus
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
    "RL": {"target": "#39ff14", "rpm": "#39ff14", "rpm_filt": "#39ff14", "pwr": "#39ff14"},
    "RR": {"target": "#ff9f1c", "rpm": "#ff9f1c", "rpm_filt": "#ff9f1c", "pwr": "#ff9f1c"},
    # Legacy IDs from old logs (BL/BR = rear wheels before RL/RR naming)
    "BL": {"target": "#39ff14", "rpm": "#39ff14", "rpm_filt": "#39ff14", "pwr": "#39ff14"},
    "BR": {"target": "#ff9f1c", "rpm": "#ff9f1c", "rpm_filt": "#ff9f1c", "pwr": "#ff9f1c"},
}

MOTOR_ACCENTS = {
    "FL": ("#00f0ff", "0,240,255"),
    "FR": ("#ff00ff", "255,0,255"),
    "RL": ("#39ff14", "57,255,20"),
    "RR": ("#ff9f1c", "255,159,28"),
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
        self.comm_log = CommLogger()
        self.transport_type = "-"
        self.buffers: dict[str, MotorBuffer] = {}
        self.connected = False
        self.logging = False
        self.log_file: Optional[str] = None

        # Comm-log / agent-monitor state
        self.last_rx_time: Optional[float] = None
        self.last_rx_summary = 0.0
        self.rx_verbosity = "summary"  # summary (1 Hz) | all | off
        self.log_paused = False
        self.log_direction = "all"
        self.log_min_level = "DEBUG"
        self.log_search = ""
        self._prev_transport_connected = False

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
        self.calibration_running = False
        self.cal_buffers: dict[str, MotorBuffer] = {}
        self.cal_pwm_targets: dict[str, float] = {}
        self.calibration_result: Optional[CalibrationResult] = None
        self.cal_motors = "FL,FR,RL,RR"

        # Tuning state
        self.tuning_method_id = "ziegler_nichols"
        self.tuning_target_rpm = 60
        self.tuning_result: Optional[TuningResult] = None
        self.tuning_running = False
        self.tuning_buffers: dict[str, MotorBuffer] = {}

        # Chart Y-axis settings (fixed by default for RPM/PWM, auto for error)
        self.rpm_y_auto = False
        self.rpm_y_min = -330.0
        self.rpm_y_max = 330.0
        self.pwr_y_auto = False
        self.pwr_y_min = -260.0
        self.pwr_y_max = 260.0
        self.err_y_auto = True
        self.err_y_min = -100.0
        self.err_y_max = 100.0

    def get_buffer(self, motor_id: str) -> MotorBuffer:
        if motor_id not in self.buffers:
            self.buffers[motor_id] = MotorBuffer()
        return self.buffers[motor_id]

    def get_cal_buffer(self, motor_id: str) -> MotorBuffer:
        if motor_id not in self.cal_buffers:
            self.cal_buffers[motor_id] = MotorBuffer(maxlen=5000)
        return self.cal_buffers[motor_id]

    def get_tuning_buffer(self, motor_id: str) -> MotorBuffer:
        if motor_id not in self.tuning_buffers:
            self.tuning_buffers[motor_id] = MotorBuffer(maxlen=5000)
        return self.tuning_buffers[motor_id]


state = AppState()
state.comm_log.sys(f"RocBot tuner started — comm log: {state.comm_log.file_path}")

# ─── ECharts Configuration ──────────────────────────────────────────────


def _chart_theme() -> dict:
    """Common dark/neon theme for ECharts."""
    return {
        "backgroundColor": "transparent",
        "textStyle": {"fontFamily": "monospace", "color": "#e0e0e0", "fontSize": 14},
        "tooltip": {
            "trigger": "axis",
            "backgroundColor": "#0f0f14",
            "borderColor": "#1a1a2e",
            "textStyle": {"color": "#e0e0e0", "fontSize": 14},
        },
        "legend": {"data": [], "top": 5, "textStyle": {"color": "#aab0c0", "fontSize": 13}},
    }


def _axis_min_max(auto: bool, vmin: float, vmax: float) -> dict:
    """Return ECharts yAxis min/max. Auto mode omits fixed bounds."""
    if auto:
        return {}
    lo, hi = (vmin, vmax) if vmin <= vmax else (vmax, vmin)
    return {"min": lo, "max": hi}


def build_rpm_chart() -> dict:
    theme = _chart_theme()
    theme.update({
        "grid": {"left": 60, "right": 20, "top": 40, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "nameTextStyle": {"color": "#8888a0", "fontSize": 14},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#aab0c0", "fontSize": 13},
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "RPM",
            "nameTextStyle": {"color": "#8888a0", "fontSize": 14},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#aab0c0", "fontSize": 13},
            "splitLine": {"lineStyle": {"type": "dashed", "color": "#1a1a2e"}},
            **_axis_min_max(state.rpm_y_auto, state.rpm_y_min, state.rpm_y_max),
        },
        "series": [],
    })
    return theme


def build_pwr_chart() -> dict:
    theme = _chart_theme()
    theme.update({
        "grid": {"left": 60, "right": 20, "top": 40, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "nameTextStyle": {"color": "#8888a0", "fontSize": 14},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#aab0c0", "fontSize": 13},
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "PWM",
            "nameTextStyle": {"color": "#8888a0", "fontSize": 14},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#aab0c0", "fontSize": 13},
            "splitLine": {"lineStyle": {"type": "dashed", "color": "#1a1a2e"}},
            **_axis_min_max(state.pwr_y_auto, state.pwr_y_min, state.pwr_y_max),
        },
        "series": [],
    })
    return theme


def build_error_chart() -> dict:
    theme = _chart_theme()
    theme.update({
        "grid": {"left": 60, "right": 20, "top": 40, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "nameTextStyle": {"color": "#8888a0", "fontSize": 14},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#aab0c0", "fontSize": 13},
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "Error (RPM)",
            "nameTextStyle": {"color": "#8888a0", "fontSize": 14},
            "axisLine": {"lineStyle": {"color": "#1a1a2e"}},
            "axisLabel": {"color": "#aab0c0", "fontSize": 13},
            "splitLine": {"lineStyle": {"type": "dashed", "color": "#1a1a2e"}},
            **_axis_min_max(state.err_y_auto, state.err_y_min, state.err_y_max),
        },
        "series": [],
    })
    return theme


# ─── Cyberpunk Theme ─────────────────────────────────────────────────────

ui.add_css("""
/* Global cyberpunk theme overrides — larger base type for readability */
body { background-color: #050508 !important; font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace !important; font-size: 16px !important; }
.q-dark { background-color: #0f0f14 !important; }
.q-drawer { background-color: #0f0f14 !important; border-right: 1px solid #1a1a2e !important; }
.q-header { background-color: #0f0f14 !important; border-bottom: 1px solid #1a1a2e !important; box-shadow: 0 0 15px rgba(0, 240, 255, 0.08) !important; padding-top: 4px !important; padding-bottom: 0 !important; }
.q-card { background-color: #0f0f14 !important; border: 1px solid #1a1a2e !important; }
.q-field__native, .q-field__input { color: #e0e0e0 !important; font-size: 15px !important; }
.q-field__label { color: #8888a0 !important; font-size: 14px !important; }
.q-field--outlined .q-field__control { border-color: #1a1a2e !important; }
.q-field--outlined.q-field--focused .q-field__control { border-color: #00f0ff !important; box-shadow: 0 0 6px rgba(0, 240, 255, 0.25) !important; }
.q-separator { background-color: #1a1a2e !important; }
.q-btn { text-transform: uppercase; letter-spacing: 0.08em; font-family: monospace; font-weight: 600; font-size: 13px !important; }
.q-menu { background-color: #0f0f14 !important; border: 1px solid #1a1a2e !important; }
.q-item__label, .q-checkbox__label, .q-toggle__label { font-size: 14px !important; }
/* Markdown / log readability */
.nicegui-markdown, .markdown-body { font-size: 14px !important; line-height: 1.5 !important; }
.nicegui-log { font-size: 14px !important; line-height: 1.5 !important; }
/* Scrollbar */
::-webkit-scrollbar { width: 5px; }
::-webkit-scrollbar-track { background: #050508; }
::-webkit-scrollbar-thumb { background: #1a1a2e; border-radius: 2px; }
::-webkit-scrollbar-thumb:hover { background: #00f0ff; }
/* Tab styling — blended into the header bar */
.q-tab { text-transform: uppercase; letter-spacing: 0.1em; font-weight: 700; font-size: 15px !important; padding: 10px 18px !important; }
.q-tab--active { color: #00f0ff !important; background: rgba(0,240,255,0.08) !important; }
.q-tabs { background: transparent !important; }
.q-tabs__indicator { background: #00f0ff !important; height: 3px !important; }
.q-tabs__content { background: transparent !important; }
/* Tabs living inside the header inherit its background so they visually merge */
.q-header .q-tabs { border-bottom: none !important; }
.q-header .q-tab { color: #aab0c0 !important; }
.q-header .q-tab--active { color: #00f0ff !important; }
/* Mode toggle */
.q-btn-toggle .q-btn {
    font-size: 13px !important;
    font-weight: 600;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    border: 2px solid #1a1a2e !important;
    border-radius: 6px !important;
    padding: 8px 14px;
}
.q-btn-toggle .q-btn--active {
    border-color: currentColor !important;
    box-shadow: 0 0 8px rgba(0, 240, 255, 0.2);
}
/* Wheel status cards */
.wheel-card { min-width: 200px; }
.wheel-name { font-size: 22px !important; font-weight: 800 !important; letter-spacing: 0.1em; }
.wheel-rpm { font-size: 20px !important; font-weight: 700 !important; }
.wheel-sub { font-size: 15px !important; }
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

# Mode toggle
mode_toggle: Optional[ui.toggle] = None

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
cal_rpm_chart: Optional[ui.echart] = None
cal_pwr_chart: Optional[ui.echart] = None

# Tuning UI refs
tuning_status: Optional[ui.label] = None
tuning_result_label: Optional[ui.markdown] = None
tuning_phases: Optional[ui.markdown] = None
tuning_params_container: Any = None
tuning_rpm_chart: Optional[ui.echart] = None
tuning_pwr_chart: Optional[ui.echart] = None

# Comm-log tab UI refs
log_view: Optional[Any] = None
log_agent_label: Optional[ui.label] = None
log_link_label: Optional[ui.label] = None
log_transport_label: Optional[ui.label] = None
log_last_rx_label: Optional[ui.label] = None
log_counts_label: Optional[ui.label] = None
log_file_label: Optional[ui.label] = None
log_last_seq = 0

LOG_LEVEL_ORDER = {"DEBUG": 0, "INFO": 1, "WARN": 2, "ERROR": 3}

# ─── Chart Update ────────────────────────────────────────────────────────


def _build_chart_option(buffers: dict[str, MotorBuffer], chart_builder, include_target: bool = True):
    """Build a chart option from a set of motor buffers."""
    option = chart_builder()
    for motor_id, buf in buffers.items():
        if not buf.timestamps:
            continue
        colors = MOTOR_COLORS.get(motor_id, MOTOR_COLORS["FL"])
        time_labels = [f"{t:.1f}" for t in buf.timestamps]

        if include_target:
            option["legend"]["data"].append(f"{motor_id} Target")
            option["series"].append({
                "name": f"{motor_id} Target",
                "type": "line",
                "data": list(buf.target_rpm),
                "lineStyle": {"type": "dashed", "width": 1},
                "itemStyle": {"color": colors["target"]},
                "symbol": "none",
            })
        option["legend"]["data"].append(f"{motor_id} RPM")
        option["series"].append({
            "name": f"{motor_id} RPM",
            "type": "line",
            "data": [round(v, 1) for v in buf.rpm_filt],
            "lineStyle": {"width": 2},
            "itemStyle": {"color": colors["rpm_filt"]},
            "symbol": "none",
        })
        option["xAxis"]["data"] = time_labels
    return option


def update_charts():
    """Update all dashboard, calibration, and tuning charts."""
    # Dashboard charts
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

    # Calibration charts
    if cal_rpm_chart:
        cal_option = _build_chart_option(state.cal_buffers, build_rpm_chart)
        cal_rpm_chart.options.clear()
        cal_rpm_chart.options.update(cal_option)
        cal_rpm_chart.update()

    if cal_pwr_chart:
        cal_pwr_option = build_pwr_chart()
        for motor_id, buf in state.cal_buffers.items():
            if not buf.timestamps:
                continue
            colors = MOTOR_COLORS.get(motor_id, MOTOR_COLORS["FL"])
            time_labels = [f"{t:.1f}" for t in buf.timestamps]
            cal_pwr_option["legend"]["data"].append(f"{motor_id} PWM")
            cal_pwr_option["series"].append({
                "name": f"{motor_id} PWM",
                "type": "line",
                "data": [round(v, 1) for v in buf.pwr_filt],
                "lineStyle": {"width": 2},
                "itemStyle": {"color": colors["pwr"]},
                "symbol": "none",
            })
            cal_pwr_option["xAxis"]["data"] = time_labels
        cal_pwr_chart.options.clear()
        cal_pwr_chart.options.update(cal_pwr_option)
        cal_pwr_chart.update()

    # Tuning charts
    if tuning_rpm_chart:
        tun_option = _build_chart_option(state.tuning_buffers, build_rpm_chart)
        tuning_rpm_chart.options.clear()
        tuning_rpm_chart.options.update(tun_option)
        tuning_rpm_chart.update()

    if tuning_pwr_chart:
        tun_pwr_option = build_pwr_chart()
        for motor_id, buf in state.tuning_buffers.items():
            if not buf.timestamps:
                continue
            colors = MOTOR_COLORS.get(motor_id, MOTOR_COLORS["FL"])
            time_labels = [f"{t:.1f}" for t in buf.timestamps]
            tun_pwr_option["legend"]["data"].append(f"{motor_id} PWM")
            tun_pwr_option["series"].append({
                "name": f"{motor_id} PWM",
                "type": "line",
                "data": [round(v, 1) for v in buf.pwr_filt],
                "lineStyle": {"width": 2},
                "itemStyle": {"color": colors["pwr"]},
                "symbol": "none",
            })
            tun_pwr_option["xAxis"]["data"] = time_labels
        tuning_pwr_chart.options.clear()
        tuning_pwr_chart.options.update(tun_pwr_option)
        tuning_pwr_chart.update()


def update_motor_cards():
    """Update motor state display cards."""
    if not state.latest_state:
        return

    for motor_id, motor in state.latest_state.motors.items():
        if motor_id in motor_cards:
            cards = motor_cards[motor_id]
            cards["rpm"].set_text(f"RPM: {motor.rpm:.1f}")
            cards["filt"].set_text(f"Filtered: {motor.rpm_filt:.1f}")
            cards["pwm"].set_text(f"PWM: {motor.pwr_filt:.1f}")
            cards["dir"].set_text(f"Dir: {motor.direction}")
            # In direct mode the concept of PID error is different:
            # show the RPM value itself as the deviation from 0
            if state.mode == "DIRECT":
                cards["err"].set_text(f"RPM: {motor.rpm:.1f}")
            else:
                cards["err"].set_text(f"Error: {motor.error:.1f}")


# ─── Serial Reader Task ──────────────────────────────────────────────────


async def serial_reader():
    """Background task that reads from serial and updates UI."""
    def on_state(new_state: ControllerState):
        state.latest_state = new_state
        state.last_rx_time = time.time()
        _log_rx_state(new_state)

        # Update live buffers (dashboard)
        for motor_id, motor in new_state.motors.items():
            buf = state.get_buffer(motor_id)
            buf.update(motor, new_state.timestamp)

            # Step test buffer
            if state.step_test_active:
                if motor_id not in state.step_test_buffers:
                    state.step_test_buffers[motor_id] = MotorBuffer(maxlen=5000)
                state.step_test_buffers[motor_id].update(motor, new_state.timestamp)

            # Calibration buffer
            if state.calibration_running:
                cal_buf = state.get_cal_buffer(motor_id)
                # Override target with current PWM level so charts show the step
                cal_motor = MotorState(
                    motor_id=motor.motor_id,
                    target_rpm=state.cal_pwm_targets.get(motor_id, motor.target_rpm),
                    rpm=motor.rpm,
                    rpm_filt=motor.rpm_filt,
                    pwr=motor.pwr,
                    pwr_filt=motor.pwr_filt,
                    direction=motor.direction,
                    pulses=motor.pulses,
                    timestamp=motor.timestamp,
                )
                cal_buf.update(cal_motor, new_state.timestamp)

            # Tuning buffer
            if state.tuning_running:
                tun_buf = state.get_tuning_buffer(motor_id)
                tun_buf.update(motor, new_state.timestamp)

        # Log if enabled
        if state.logging:
            state.logger.log(new_state)

    try:
        await state.transport.read_loop(on_state)
    except Exception as e:
        print(f"Reader error: {e}")
        state.comm_log.error(f"Reader error: {e}", transport=_transport_name())
        state.connected = False
        if status_label:
            status_label.set_text("● Disconnected")
            status_label.style("color: red")


# ─── Comm Log & Agent Monitor ──────────────────────────────────────────


def _transport_name() -> str:
    return state.transport_type or "-"


def _patch_transport_logging():
    """Wrap transport.send_command so EVERY TX (UI, tuning, calibration)
    is recorded in the comm log. Idempotent per transport instance."""
    t = state.transport
    if t is None or getattr(t, "_rocbot_log_patched", False):
        return
    orig_send = t.send_command

    async def logged_send(cmd: str):
        state.comm_log.tx(cmd, transport=_transport_name(), agent_running=state.agent_running)
        return await orig_send(cmd)

    t.send_command = logged_send  # type: ignore[method-assign]
    t._rocbot_log_patched = True  # type: ignore[attr-defined]


def _log_rx_state(new_state: ControllerState):
    """Record an RX state frame, honoring the verbosity setting."""
    mode = state.rx_verbosity or "summary"
    if mode == "off":
        return
    now = time.time()
    if mode == "summary" and (now - state.last_rx_summary) < 1.0:
        return
    state.last_rx_summary = now
    parts = [f"mode={new_state.mode} tgt={new_state.target_value:g}"]
    for mid in ("FL", "FR", "RL", "RR"):
        m = new_state.motors.get(mid)
        if m is not None:
            parts.append(f"{mid}:{m.rpm:.1f}/{m.pwr_filt:.0f}/{m.direction}")
    state.comm_log.rx(
        "<< " + " ".join(parts),
        transport=_transport_name(),
        agent_running=state.agent_running,
    )


def _on_raw_serial_line(line: str):
    """Log serial lines that carry no parsed state (boot text, errors...).
    Parsed frames (--- DEBUG / &...) are covered by the RX state summary."""
    if line.startswith("--- DEBUG") or line.startswith("&"):
        return
    state.comm_log.rx(f"<< [raw] {line[:300]}", transport="serial",
                      agent_running=state.agent_running)


def _on_ros2_debug(raw: str):
    """Log the raw rocbot/debug string published by the ESP32."""
    state.comm_log.rx(f"<< [rocbot/debug] {raw[:300]}", transport="ros2",
                      agent_running=state.agent_running)


def probe_agent_status() -> AgentStatus:
    """Snapshot the micro-ROS server side: Docker agent, transport, RX age."""
    from rocbot_tuner.commlog import is_docker_agent_running
    docker = is_docker_agent_running()
    tconn = False
    if state.transport is not None:
        try:
            tconn = bool(state.transport.is_connected)
        except Exception:
            tconn = False
    rclpy_ok = False
    if HAS_RCLPY:
        try:
            import rclpy
            rclpy_ok = bool(rclpy.ok())
        except Exception:
            rclpy_ok = False
    age = None
    if state.last_rx_time is not None:
        age = time.time() - state.last_rx_time
    return AgentStatus(
        docker_running=docker,
        transport_connected=tconn,
        transport_type=_transport_name(),
        rclpy_ok=rclpy_ok,
        last_rx_age_s=age,
        tx_count=state.comm_log.tx_count,
        rx_count=state.comm_log.rx_count,
    )


def monitor_agent():
    """1s timer: probe the micro-ROS server, log transitions, refresh badges."""
    if log_agent_label is None:
        return
    status = probe_agent_status()

    # Log transitions so every behaviour change is tracked
    if status.docker_running != state.agent_running:
        state.agent_running = status.docker_running
        if status.docker_running:
            state.comm_log.agent("micro-ROS agent UP (docker: rocbot_microros_agent, UDP 8888)")
        else:
            state.comm_log.agent("micro-ROS agent DOWN (container not running)", level="WARN")
    if status.transport_connected != state._prev_transport_connected:
        state._prev_transport_connected = status.transport_connected
        if status.transport_connected:
            state.comm_log.sys(f"Transport {status.transport_type} linked")
        else:
            state.comm_log.sys(f"Transport {status.transport_type} lost", level="WARN")

    # Badges
    if status.docker_running:
        log_agent_label.set_text("Agent (docker): ● UP").style("color: #39ff14")
    else:
        log_agent_label.set_text("Agent (docker): ○ DOWN").style("color: #ff3333")
    link_colors = {"LINKED": "#39ff14", "TRANSPORT-ONLY": "#ff9f1c",
                   "AGENT-ONLY": "#ff9f1c", "DOWN": "#ff3333"}
    log_link_label.set_text(f"Link: {status.link}").style(
        f"color: {link_colors.get(status.link, '#8888a0')}")
    tstate = "UP" if status.transport_connected else "DOWN"
    tcolor = "#39ff14" if status.transport_connected else "#8888a0"
    log_transport_label.set_text(
        f"Transport ({status.transport_type}): {tstate}  rclpy={'ok' if status.rclpy_ok else 'n/a'}"
    ).style(f"color: {tcolor}")

    # Keep the drawer badge truthful even if docker was stopped externally
    try:
        if agent_status is not None:
            if status.docker_running:
                agent_status.set_text("Agent: ● Running (UDP 8888)").style("color: cyan")
            else:
                agent_status.set_text("Agent: ○ Stopped").style("color: gray")
    except Exception:
        pass


def _event_visible(seq_ev) -> bool:
    """Apply the Logs-tab filters to one event."""
    if state.log_direction != "all" and seq_ev.direction != state.log_direction:
        return False
    min_order = LOG_LEVEL_ORDER.get(state.log_min_level, 0)
    if LOG_LEVEL_ORDER.get(seq_ev.level, 0) < min_order:
        return False
    if state.log_search:
        needle = state.log_search.lower()
        if needle not in seq_ev.message.lower() and needle not in seq_ev.transport.lower():
            return False
    return True


def _filtered_events(limit: int = 300) -> list:
    return [e for e in state.comm_log.snapshot(limit=5000)
            if _event_visible(e)][-limit:]


def refresh_log_view():
    """Rebuild the log view from scratch (filter change / reconnect)."""
    global log_last_seq
    if log_view is None:
        return
    log_view.clear()
    for ev in _filtered_events(limit=300):
        log_view.push(ev.format_line())
    log_last_seq = state.comm_log.last_seq


def drain_log_to_ui():
    """0.5s timer: append new comm events + refresh counters."""
    global log_last_seq
    if log_view is None:
        return
    if not state.log_paused:
        events = state.comm_log.snapshot(limit=5000)
        new_events = [e for e in events if e.seq > log_last_seq]
        if new_events:
            if len(new_events) > 300:  # cap catch-up after pause/reconnect
                new_events = new_events[-300:]
            for ev in new_events:
                if _event_visible(ev):
                    log_view.push(ev.format_line())
            log_last_seq = new_events[-1].seq
    # Counters / file labels (cheap, no docker probe here)
    if log_counts_label is not None:
        log_counts_label.set_text(
            f"TX: {state.comm_log.tx_count}  RX: {state.comm_log.rx_count}  "
            f"buffered: {len(state.comm_log)}"
        )
    if log_last_rx_label is not None:
        if state.last_rx_time is None:
            log_last_rx_label.set_text("Last RX: never").style("color: #8888a0")
        else:
            age = time.time() - state.last_rx_time
            stale = state.connected and age > 2.0
            log_last_rx_label.set_text(f"Last RX: {age:.1f}s ago").style(
                f"color: {'#ff3333' if stale else '#39ff14'}")
    if log_file_label is not None:
        log_file_label.set_text(f"File: {state.comm_log.file_path or '-'}  |  CSV: {state.log_file or 'OFF'}")


def export_comm_log():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(state.comm_log.log_dir or ".", f"commlog_export_{stamp}.log")
    try:
        state.comm_log.export(path)
        state.comm_log.sys(f"Comm log exported: {path}")
        ui.notify(f"Exported: {path}")
    except OSError as e:
        ui.notify(f"Export failed: {e}", type="negative")


def clear_comm_log():
    global log_last_seq
    state.comm_log.clear()
    log_last_seq = state.comm_log.last_seq
    if log_view is not None:
        log_view.clear()
    ui.notify("Comm log cleared")


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
            state.transport_type = "ros2"
            state.transport.debug_handler = _on_ros2_debug
            _patch_transport_logging()
            state.comm_log.sys(f"Connected via ROS2 (WiFi/UDP) — agent_up={state.agent_running}")
            status_label.set_text("● ROS2 (WiFi)")
            status_label.style("color: cyan")
            ui.notify("Connected via ROS2 (WiFi/UDP)")
            asyncio.create_task(serial_reader())
        else:
            state.comm_log.error("ROS2 connection failed — is the micro-ROS agent running?")
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
            state.transport_type = "serial"
            state.transport.raw_handler = _on_raw_serial_line
            _patch_transport_logging()
            state.comm_log.sys(f"Connected via Serial {port} @ {baud}")
            status_label.set_text("● Connected")
            status_label.style("color: green")
            ui.notify(f"Connected to {port}")
            asyncio.create_task(serial_reader())
        else:
            state.comm_log.error(f"Serial connection failed: {port} @ {baud}")
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
        state.comm_log.agent("Launch requested: docker run microros/micro-ros-agent:humble udp4 --port 8888")
        agent_status.set_text("Agent: ● Running (UDP 8888)").style("color: cyan")
        ui.notify("micro-ROS agent launched (UDP 8888)")
    except Exception as e:
        state.comm_log.error(f"Agent launch failed: {e}")
        ui.notify(f"Failed to launch agent: {e}", type="negative")


async def stop_agent():
    if not state.agent_running and not _is_agent_running():
        ui.notify("Agent not running", type="warning")
        return

    try:
        subprocess.run(["docker", "stop", "rocbot_microros_agent"], check=False, capture_output=True)
        state.agent_running = False
        state.agent_process = None
        state.comm_log.agent("Stop requested: docker stop rocbot_microros_agent")
        agent_status.set_text("Agent: ● Stopped").style("color: gray")
        ui.notify("micro-ROS agent stopped")
    except Exception as e:
        state.comm_log.error(f"Agent stop failed: {e}")
        ui.notify(f"Failed to stop agent: {e}", type="negative")


async def stop_all():
    """Stop all active procedures: motors, step test, calibration, tuning."""
    # Stop motors
    if state.connected and state.transport:
        try:
            await state.transport.send_command("s")
        except Exception:
            pass
    state.mode = "STOP"
    state.step_test_active = False
    state.calibration_running = False
    state.tuning_running = False

    # Stop calibration instance
    state.calibration.stop()

    # Stop tuning instance
    global _tuning_instance
    if _tuning_instance is not None:
        _tuning_instance.stop()
        _tuning_instance = None

    if mode_label:
        mode_label.set_text("Mode: STOP")
    ui.notify("All procedures stopped")


async def disconnect_serial():
    await stop_all()
    if state.transport:
        await state.transport.disconnect()
        state.transport = None
    state.connected = False
    state.comm_log.sys(f"Disconnected (was {state.transport_type})")
    state.transport_type = "-"
    if status_label:
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


async def _on_mode_change(e):
    """Dispatch mode change from toggle to the correct handler."""
    mode = e.value
    if mode == "PID":
        await set_mode_pid()
    elif mode == "DIRECT":
        await set_mode_direct()
    elif mode == "STOP":
        await stop_motors()


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
    await stop_all()


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
        state.comm_log.sys(f"CSV logging started: {state.log_file}")
        log_label.set_text(f"Logging: {os.path.basename(state.log_file)}")
        log_label.style("color: green")
        ui.notify("Logging started")
    else:
        state.logger.close()
        state.comm_log.sys(f"CSV logging stopped: {state.log_file}")
        state.logging = False
        state.log_file = None
        log_label.set_text("Logging: OFF")
        log_label.style("color: gray")
        ui.notify("Logging stopped")


async def clear_buffers():
    for buf in state.buffers.values():
        buf.clear()
    for buf in state.cal_buffers.values():
        buf.clear()
    for buf in state.tuning_buffers.values():
        buf.clear()
    ui.notify("Buffers cleared")


# ─── Calibration Handlers ────────────────────────────────────────────────


async def run_calibration():
    """Run the channel imbalance calibration."""
    if not state.connected:
        ui.notify("Not connected", type="warning")
        return

    # Read calibration params from UI
    motors_text = getattr(state, "cal_motors", "FL,FR,RL,RR")
    motor_ids = [m.strip() for m in motors_text.split(",")]
    pwm_levels_text = cal_pwm_input.value or "80,120,160,200"
    try:
        pwm_levels = [int(x.strip()) for x in pwm_levels_text.split(",")]
    except ValueError:
        ui.notify("Invalid PWM levels format", type="negative")
        return

    direction = cal_dir_toggle.value  # "forward" or "reverse"

    # Reset data
    state.cal_buffers.clear()
    state.cal_pwm_targets.clear()
    state.calibration_running = True
    state.calibration.status = "running"
    state.calibration_result = None

    cal_status.set_text("Status: Running...")
    cal_status.style("color: #ff9f1c")
    cal_results.set_text("Collecting data...")
    cal_progress.set_value(0.0)

    total_steps = len(pwm_levels) + 2  # stop + levels + stop + compute
    step = 0

    # Step 1: Stop
    step += 1
    cal_status.set_text(f"[{step}/{total_steps}] Stopping motors...")
    await state.transport.send_command("s")
    await asyncio.sleep(0.3)

    if not state.calibration_running:
        return

    # Step 2-N: Run each PWM level
    for i, pwm in enumerate(pwm_levels):
        if not state.calibration_running:
            break
        step += 1
        cal_status.set_text(f"[{step}/{total_steps}] Testing PWM {pwm} ({direction})...")
        cal_progress.set_value(step / total_steps)

        if direction == "reverse":
            await state.transport.send_command(f"D{pwm}")
        else:
            await state.transport.send_command(f"d{pwm}")

        # Set target RPM so the chart shows the expected value
        for mid in motor_ids:
            state.cal_pwm_targets[mid] = float(pwm)

        await asyncio.sleep(0.5)  # settle
        if not state.calibration_running:
            break
        await asyncio.sleep(2.0)  # hold / collect

    # Step N+1: Stop
    step += 1
    cal_status.set_text(f"[{step}/{total_steps}] Stopping...")
    await state.transport.send_command("s")
    state.calibration_running = False
    state.cal_pwm_targets.clear()

    # Step N+2: Compute
    step += 1
    cal_status.set_text(f"[{step}/{total_steps}] Computing results...")
    cal_progress.set_value(1.0)

    result = _compute_calibration_result(motor_ids, pwm_levels)

    cal_results.set_text(result.summary())
    cal_status.set_text("Status: Complete").style("color: #39ff14")
    state.calibration.status = "complete"
    ui.notify("Calibration complete!")
    state.calibration_result = result


def _compute_calibration_result(motor_ids, pwm_levels):
    """Compute CalibrationResult from state.cal_buffers."""
    from rocbot_tuner.calibration import CalibrationResult, MotorCalibration, CalibrationPoint
    result = CalibrationResult(pwm_levels_tested=pwm_levels)
    for mid in motor_ids:
        cal = MotorCalibration(motor_id=mid)
        buf = state.cal_buffers.get(mid)
        if not buf:
            continue
        # Split buffer into segments by target_rpm changes
        targets = list(buf.target_rpm)
        rpms = list(buf.rpm_filt)
        if not targets or not rpms:
            continue
        current_pwm = int(targets[0])
        segment_rpms = []
        for t, r in zip(targets, rpms):
            if int(t) != current_pwm:
                if segment_rpms:
                    cal.points.append(CalibrationPoint(
                        pwm=current_pwm,
                        avg_rpm=sum(segment_rpms) / len(segment_rpms),
                        samples=len(segment_rpms),
                    ))
                segment_rpms = []
                current_pwm = int(t)
            segment_rpms.append(r)
        if segment_rpms:
            cal.points.append(CalibrationPoint(
                pwm=current_pwm,
                avg_rpm=sum(segment_rpms) / len(segment_rpms),
                samples=len(segment_rpms),
            ))
        result.motors[mid] = cal

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
    return result


def stop_calibration():
    state.calibration_running = False
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
            ui.label("Unknown method").classes("text-[#ff3333] text-sm")
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
                ui.checkbox(label, value=default).bind_value(state, f"tuning_{pid}", strict=False).classes("text-sm text-[#8888a0] mt-1")
            else:
                min_v = param.get("min", 0)
                max_v = param.get("max", 100)
                step = param.get("step", 1)
                ui.number(
                    value=default, min=min_v, max=max_v, step=step,
                    format="%.2f" if ptype == "float" else "%.0f",
                ).bind_value(state, f"tuning_{pid}", strict=False).props(
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
    state.tuning_buffers.clear()
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
            state_source=lambda: state.latest_state,
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
    if _tuning_instance is not None:
        _tuning_instance.stop()
    if state.connected and state.transport:
        try:
            await state.transport.send_command("s")
        except Exception:
            pass
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
    return ui.button(label, on_click=onclick).props("dense flat size=md").classes(
        f"border border-[{color}] bg-[{color}]/10 text-[{color}] hover:bg-[{color}]/20 uppercase tracking-wider text-sm font-bold {cls}"
    )


def _small_btn(color: str, label: str, onclick, cls: str = ""):
    """Create an extra-small cyberpunk button."""
    return ui.button(label, on_click=onclick).props("dense flat size=sm").classes(
        f"border border-[{color}] bg-[{color}]/10 text-[{color}] hover:bg-[{color}]/20 uppercase tracking-wider text-xs font-bold {cls}"
    )


# ─── Header (tabs blended into the header bar) ─────────────────────────

with ui.header().classes("items-center justify-between gap-2 bg-[#0f0f14] text-[#e0e0e0] px-4 border-b border-[#1a1a2e]"):
    with ui.row().classes("items-center gap-3"):
        ui.button("≡", on_click=lambda: drawer.toggle()).props("flat dense size=md").classes("text-[#00f0ff] text-xl")
        ui.label("◈ ROCBOT PID TUNER").classes(
            "text-2xl font-bold tracking-[0.2em] text-[#00f0ff] drop-shadow-[0_0_8px_rgba(0,240,255,0.4)]"
        )
    with ui.tabs().classes("bg-transparent") as tabs:
        ui.tab("Dashboard", icon="dashboard")
        ui.tab("Calibration", icon="tune")
        ui.tab("Auto-Tuning", icon="auto_fix_high")
        ui.tab("Logs", icon="terminal")
    with ui.row().classes("items-center gap-4"):
        status_label = ui.label("● DISCONNECTED").classes("text-sm font-mono tracking-wider")
        mode_label = ui.label("MODE: STOP").classes("text-sm font-mono tracking-wider text-[#8888a0]")
        log_label = ui.label("LOG: OFF").classes("text-sm font-mono tracking-wider text-[#8888a0]")

# ─── Left Drawer ─────────────────────────────────────────────────────────

with ui.left_drawer(fixed=True).props("bordered").classes("bg-[#0f0f14] w-72 border-r border-[#1a1a2e]") as drawer:
    with ui.scroll_area().classes("fit p-3"):
        # Connection
        ui.label("◄ CONNECTION ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        transport_type = ui.toggle({"serial": "Serial (USB)", "ros2": "ROS2 (WiFi)"}, value="ros2").props("dense color=cyan")
        transport_type.classes("text-sm")
        port_input = ui.input(value="/dev/ttyUSB0").classes("w-full mt-1 text-[15px]").props("dense outlined dark label='Serial Port' color=cyan")
        baud_input = ui.input(value="115200").classes("w-full mt-1 text-[15px]").props("dense outlined dark label='Serial Baud' color=cyan")
        with ui.row().classes("gap-1 mt-1 w-full"):
            _btn("#39ff14", "Connect", lambda: connect_serial(transport_type.value), "flex-1")
            _btn("#ff3333", "Disconnect", disconnect_serial, "flex-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # micro-ROS Agent
        ui.label("◄ AGENT (UDP) ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        agent_status = ui.label("Agent: ○ STOPPED").classes("text-sm font-mono text-[#8888a0]")
        with ui.row().classes("gap-1 w-full"):
            _btn("#00f0ff", "Launch", launch_agent, "flex-1")
            _btn("#ff3333", "Stop", stop_agent, "flex-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Mode
        ui.label("◄ MODE ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        mode_toggle = ui.toggle(
            {"PID": "PID", "DIRECT": "Direct", "STOP": "Stop"},
            on_change=_on_mode_change,
        ).bind_value(state, "mode").classes("w-full").props("rounded spread no-caps toggle-color=cyan color=grey-9")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Target
        ui.label("◄ TARGET ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=30, min=-330, max=330, step=1, format="%.0f").bind_value(state, "target_rpm").props("dense outlined dark label=RPM color=cyan").classes("w-full text-[15px]")
        _btn("#00f0ff", "Set Target", set_target, "w-full mt-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Direct PWM
        ui.label("◄ DIRECT PWM ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=100, min=0, max=255, step=1, format="%.0f").bind_value(state, "direct_pwm").props("dense outlined dark label=PWM color=orange").classes("w-full text-[15px]")
        _btn("#ff9f1c", "Set Direct", set_mode_direct, "w-full mt-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # PID Parameters
        ui.label("◄ PID GAINS ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=1.0, min=0, max=100, step=0.1, format="%.2f").bind_value(state, "kp").props("dense outlined dark label=Kp color=cyan").classes("w-full text-[15px]")
        ui.number(value=0.0, min=0, max=10, step=0.01, format="%.3f").bind_value(state, "ki").props("dense outlined dark label=Ki color=cyan").classes("w-full mt-1 text-[15px]")
        ui.number(value=0.0, min=0, max=10, step=0.01, format="%.3f").bind_value(state, "kd").props("dense outlined dark label=Kd color=cyan").classes("w-full mt-1 text-[15px]")
        ui.number(value=10.0, min=1, max=100, step=1, format="%.0f").bind_value(state, "output_scale").props("dense outlined dark label='Out Scale' color=cyan").classes("w-full mt-1 text-[15px]")
        _btn("#00f0ff", "Apply PID", send_pid_params, "w-full mt-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Step Test
        ui.label("◄ STEP TEST ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        ui.number(value=5.0, min=1, max=30, step=0.5, format="%.1f").bind_value(state, "step_test_duration").props("dense outlined dark label=Duration color=orange").classes("w-full text-[15px]")
        _btn("#ff9f1c", "Run Test", start_step_test, "w-full mt-1")

        ui.separator().classes("my-2 bg-[#1a1a2e]")

        # Utilities
        ui.label("◄ UTILITIES ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
        with ui.row().classes("gap-1"):
            _btn("#8888a0", "Log", toggle_logging, "flex-1")
            _btn("#ff3333", "Clear", clear_buffers, "flex-1")


def make_tuning_params_container():
    """Create the container for dynamic method parameters."""
    global tuning_params_container
    if tuning_params_container is not None:
        return
    # We'll create it inside the tuning controls card after the method selector
    # Use a placeholder approach: create a container that gets populated later
    tuning_params_container = ui.column().classes("w-full gap-0")
    _rebuild_tuning_params()


# ─── Main Content with Tabs (tab bar lives in the header above) ─────────

with ui.column().classes("w-full flex-1 p-3 gap-0"):
    # Tab panels
    with ui.tab_panels(tabs, value="Dashboard").classes("w-full flex-1"):
        # ── Dashboard Tab ──────────────────────────────────────────────────
        with ui.tab_panel("Dashboard").classes("p-2 gap-3"):
            # Motor status bar — large readable wheel cards
            with ui.row().classes("w-full gap-3"):
                for motor_id in ["FL", "FR", "RL", "RR"]:
                    accent, glow = MOTOR_ACCENTS[motor_id]
                    with ui.card().classes(
                        f"flex-1 p-4 bg-[#0f0f14] border border-[{accent}]/40 shadow-[0_0_12px_rgba({glow},0.08)] wheel-card"
                    ):
                        with ui.column().classes("w-full gap-1"):
                            ui.label(f"◈ {motor_id}").classes(f"wheel-name font-mono text-[{accent}]")
                            motor_cards[motor_id] = {
                                "rpm": ui.label("RPM: 0.0").classes("wheel-rpm font-mono text-[#ffffff]"),
                                "filt": ui.label("Filtered: 0.0").classes("wheel-sub font-mono text-[#aab0c0]"),
                                "pwm": ui.label("PWM: 0.0").classes("wheel-sub font-mono text-[#e0e0e0]"),
                                "dir": ui.label("Dir: STP").classes("wheel-sub font-mono text-[#aab0c0]"),
                                "err": ui.label("Error: 0.0").classes("wheel-sub font-mono text-[#ff9f1c]"),
                            }

            # Chart Y-axis controls
            with ui.card().classes("w-full p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                ui.label("◄ CHART Y-AXIS ►").classes("text-sm font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]")
                with ui.row().classes("w-full gap-4 flex-wrap"):
                    with ui.column().classes("flex-1 gap-1 min-w-[220px]"):
                        with ui.row().classes("items-center gap-2"):
                            ui.label("RPM").classes("text-base font-bold text-[#00f0ff] w-12")
                            ui.checkbox("Auto").bind_value(state, "rpm_y_auto").classes("text-sm text-[#aab0c0]")
                        with ui.row().classes("gap-2 w-full"):
                            ui.number(label="Min", value=-330.0, min=-1000, max=1000, step=10, format="%.0f").bind_value(
                                state, "rpm_y_min").props("dense outlined dark color=cyan").classes("flex-1 text-[15px]")
                            ui.number(label="Max", value=330.0, min=-1000, max=1000, step=10, format="%.0f").bind_value(
                                state, "rpm_y_max").props("dense outlined dark color=cyan").classes("flex-1 text-[15px]")
                    with ui.column().classes("flex-1 gap-1 min-w-[220px]"):
                        with ui.row().classes("items-center gap-2"):
                            ui.label("PWM").classes("text-base font-bold text-[#ff9f1c] w-12")
                            ui.checkbox("Auto").bind_value(state, "pwr_y_auto").classes("text-sm text-[#aab0c0]")
                        with ui.row().classes("gap-2 w-full"):
                            ui.number(label="Min", value=-260.0, min=-1000, max=1000, step=10, format="%.0f").bind_value(
                                state, "pwr_y_min").props("dense outlined dark color=orange").classes("flex-1 text-[15px]")
                            ui.number(label="Max", value=260.0, min=-1000, max=1000, step=10, format="%.0f").bind_value(
                                state, "pwr_y_max").props("dense outlined dark color=orange").classes("flex-1 text-[15px]")
                    with ui.column().classes("flex-1 gap-1 min-w-[220px]"):
                        with ui.row().classes("items-center gap-2"):
                            ui.label("ERR").classes("text-base font-bold text-[#39ff14] w-12")
                            ui.checkbox("Auto").bind_value(state, "err_y_auto").classes("text-sm text-[#aab0c0]")
                        with ui.row().classes("gap-2 w-full"):
                            ui.number(label="Min", value=-100.0, min=-1000, max=1000, step=10, format="%.0f").bind_value(
                                state, "err_y_min").props("dense outlined dark color=green").classes("flex-1 text-[15px]")
                            ui.number(label="Max", value=100.0, min=-1000, max=1000, step=10, format="%.0f").bind_value(
                                state, "err_y_max").props("dense outlined dark color=green").classes("flex-1 text-[15px]")
                ui.label("Uncheck Auto to lock the axis to Min/Max. Defaults: RPM −330…330, PWM −260…260, Error auto.").classes(
                    "text-sm text-[#8888a0] mt-1")

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
                    ui.label("◄ STEP RESPONSE ►").classes("text-sm font-bold mr-4 text-[#8888a0] tracking-[0.15em]")
                    metrics_label = ui.markdown("Run a step test to see metrics.").classes("text-sm flex-1 font-mono text-[#8888a0]")

        # ── Calibration Tab ────────────────────────────────────────────────
        with ui.tab_panel("Calibration").classes("p-2 gap-3"):
            with ui.row().classes("w-full gap-3"):
                # Left: controls
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ CALIBRATION SETTINGS ►").classes(
                        "text-sm font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )

                    ui.label("This test measures L298N channel imbalance by running each motor at multiple PWM levels and computing output scale ratios.").classes(
                        "text-sm text-[#8888a0] mb-2"
                    )

                    ui.select(label="Motors", options=["FL,FR,RL,RR", "FL,FR", "RL,RR", "FL", "FR", "RL", "RR"], value="FL,FR,RL,RR").props(
                        "dense outlined dark color=cyan"
                    ).classes("w-full mt-1").bind_value(state, "cal_motors")

                    cal_pwm_input = ui.input(
                        value="80,120,160,200",
                    ).classes("w-full mt-1").props("dense outlined dark label='PWM Levels' color=cyan")

                    cal_dir_toggle = ui.toggle(
                        {"forward": "Forward (d)", "reverse": "Reverse (D)"},
                        value="forward",
                    ).props("dense color=cyan").classes("text-sm mt-1")

                    with ui.row().classes("gap-1 mt-2 w-full"):
                        _btn("#ff9f1c", "Run Calibration", run_calibration, "flex-1")
                        _btn("#ff3333", "Stop", stop_calibration, "flex-1")

                    cal_status = ui.label("Status: Idle").classes("text-sm font-mono mt-1 text-[#8888a0]")
                    cal_progress = ui.linear_progress(value=0).classes("mt-1").props("color=cyan")

                # Right: results
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ RESULTS ►").classes(
                        "text-sm font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )
                    cal_results = ui.markdown("Run a calibration to see results.").classes(
                        "text-sm font-mono text-[#8888a0]"
                    )
                    _btn("#39ff14", "Apply Scales to ESP32", apply_calibration, "w-full mt-2")

            # Calibration charts
            with ui.column().classes("w-full gap-2"):
                with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e]"):
                    cal_rpm_chart = ui.echart(build_rpm_chart()).classes("w-full h-64")
                with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e]"):
                    cal_pwr_chart = ui.echart(build_pwr_chart()).classes("w-full h-48")

            with ui.card().classes("w-full p-2 bg-[#0f0f14] border border-[#1a1a2e]"):
                ui.label("◄ ABOUT ►").classes("text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]")
                ui.markdown(
                    "L298N dual-H-bridge channels have slightly different output characteristics due to "
                    "transistor mismatch and PCB layout. This calibration measures the RPM/PWM efficiency "
                    "of each motor and computes `os<scale>` values so both channels produce equal speed at "
                    "the same command. The reference motor (highest efficiency) stays at os=1.0 and others "
                    "are scaled down to match."
                ).classes("text-sm text-[#8888a0]")

        # ── Auto-Tuning Tab ────────────────────────────────────────────────
        with ui.tab_panel("Auto-Tuning").classes("p-2 gap-3"):
            with ui.row().classes("w-full gap-3"):
                # Left: controls
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ TUNING CONTROLS ►").classes(
                        "text-sm font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )

                    ui.markdown(
                        "Select a tuning method and its parameters below. "
                        "The strategy pattern makes it trivial to add new algorithms."
                    ).classes("text-sm text-[#8888a0] mb-1")

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
                        label="Motor", options=["FL", "FR", "RL", "RR"], value="FL",
                    ).props("dense outlined dark color=cyan").classes("w-full mt-1")

                    # Action buttons
                    with ui.row().classes("gap-1 mt-2 w-full"):
                        _btn("#ff9f1c", "Run Tuning", run_auto_tuning, "flex-1")
                        _btn("#ff3333", "Stop", stop_tuning, "flex-1")

                    tuning_status = ui.label("Status: Idle").classes("text-sm font-mono mt-1 text-[#8888a0]")

                # Right: results + progress
                with ui.column().classes("flex-1 gap-2"):
                    # Progress phases
                    with ui.card().classes("w-full p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                        ui.label("◄ PROGRESS ►").classes(
                            "text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]"
                        )
                        tuning_phases = ui.markdown("No tuning running.").classes(
                            "text-sm font-mono text-[#8888a0]"
                        )

                    # Results
                    with ui.card().classes("w-full p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                        ui.label("◄ RESULTS ►").classes(
                            "text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]"
                        )
                        tuning_result_label = ui.markdown("Run auto-tuning to see results.").classes(
                            "text-sm font-mono text-[#8888a0]"
                        )
                        _btn("#39ff14", "Apply Gains to ESP32", apply_tuning, "w-full mt-2")

            # Tuning charts
            with ui.column().classes("w-full gap-2"):
                with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e]"):
                    tuning_rpm_chart = ui.echart(build_rpm_chart()).classes("w-full h-64")
                with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e]"):
                    tuning_pwr_chart = ui.echart(build_pwr_chart()).classes("w-full h-48")

        # ── Logs Tab (comm log + micro-ROS server status) ──────────────────
        with ui.tab_panel("Logs").classes("p-2 gap-3"):
            with ui.row().classes("w-full gap-3"):
                # Server status
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ MICRO-ROS SERVER ►").classes(
                        "text-sm font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )
                    log_agent_label = ui.label("Agent (docker): ○ DOWN").classes(
                        "text-[15px] font-mono text-[#8888a0]")
                    log_transport_label = ui.label("Transport (-): DOWN  rclpy=n/a").classes(
                        "text-[15px] font-mono text-[#8888a0]")
                    log_link_label = ui.label("Link: DOWN").classes(
                        "text-[15px] font-mono font-bold text-[#8888a0]")
                    log_last_rx_label = ui.label("Last RX: never").classes(
                        "text-[15px] font-mono text-[#8888a0]")
                # Traffic counters
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ TRAFFIC ►").classes(
                        "text-sm font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )
                    log_counts_label = ui.label("TX: 0  RX: 0  buffered: 0").classes(
                        "text-[15px] font-mono text-[#e0e0e0]")
                    log_file_label = ui.label("File: -  |  CSV: OFF").classes(
                        "text-sm font-mono text-[#8888a0]")
                    ui.label("TX = app→ESP32 commands · RX = ESP32→app frames · "
                             "AGENT = micro-ROS server transitions · SYS = app events").classes(
                        "text-sm text-[#8888a0] mt-2")
                # Filters
                with ui.card().classes("flex-1 p-3 bg-[#0f0f14] border border-[#1a1a2e]"):
                    ui.label("◄ FILTERS ►").classes(
                        "text-sm font-bold text-[#8888a0] mb-2 uppercase tracking-[0.15em]"
                    )
                    with ui.row().classes("w-full gap-1"):
                        ui.select(
                            label="Direction",
                            options={"all": "ALL", "TX": "TX", "RX": "RX",
                                     "AGENT": "AGENT", "SYS": "SYS"},
                            value="all", on_change=lambda e: refresh_log_view(),
                        ).bind_value(state, "log_direction").props(
                            "dense outlined dark color=cyan").classes("flex-1")
                        ui.select(
                            label="Min level",
                            options=["DEBUG", "INFO", "WARN", "ERROR"],
                            value="DEBUG", on_change=lambda e: refresh_log_view(),
                        ).bind_value(state, "log_min_level").props(
                            "dense outlined dark color=cyan").classes("flex-1")
                    ui.input(
                        placeholder="Search text...",
                        on_change=lambda e: refresh_log_view(),
                    ).bind_value(state, "log_search").props(
                        "dense outlined dark color=cyan").classes("w-full mt-1")
                    with ui.row().classes("w-full gap-1 mt-1 items-center"):
                        ui.select(
                            label="RX verbosity",
                            options={"summary": "RX summary (1Hz)",
                                     "all": "RX all frames", "off": "RX off"},
                            value="summary",
                        ).bind_value(state, "rx_verbosity").props(
                            "dense outlined dark color=cyan").classes("flex-1")
                    with ui.row().classes("w-full gap-2 mt-1 items-center"):
                        ui.checkbox("Pause", value=False).bind_value(
                            state, "log_paused").classes("text-sm text-[#8888a0]")
                        ui.label("Pause freezes the view (buffer keeps recording).").classes(
                            "text-sm text-[#8888a0]")
                    with ui.row().classes("gap-1 mt-1 w-full"):
                        _btn("#00f0ff", "Refresh", refresh_log_view, "flex-1")
                        _btn("#8888a0", "Export", export_comm_log, "flex-1")
                        _btn("#ff3333", "Clear", clear_comm_log, "flex-1")

            with ui.card().classes("w-full bg-[#0f0f14] border border-[#1a1a2e] p-2"):
                ui.label("◄ COMMUNICATION LOG ►").classes(
                    "text-sm font-bold text-[#8888a0] mb-1 uppercase tracking-[0.15em]"
                )
                log_view = ui.log(max_lines=2000).classes(
                    "w-full h-[500px] font-mono text-[15px]")


# ─── Auto-update timer ───────────────────────────────────────────────────

ui.timer(PLOT_UPDATE_MS / 1000, auto_update)
ui.timer(0.5, drain_log_to_ui)
ui.timer(1.0, monitor_agent)

# ─── Startup Check ───────────────────────────────────────────────────────

if _is_agent_running():
    state.agent_running = True
    state.comm_log.agent("micro-ROS agent detected at startup (docker: rocbot_microros_agent)")
    agent_status.set_text("Agent: ● Running (UDP 8888)").style("color: cyan")
else:
    state.comm_log.agent("No micro-ROS agent detected at startup", level="WARN")

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
