#!/usr/bin/env python3
"""
RocBot PID Tuner - Integrated Motor Control Dashboard

Real-time visualization, PID tuning, and step-response analysis
for the RocBot ESP32 motor controller.

Usage:
    python app.py                     # Default: /dev/ttyUSB0
    python app.py --port /dev/ttyACM0
    python app.py --port /dev/ttyUSB0 --baud 115200
"""

import asyncio
import sys
import os
import time
from collections import deque
from datetime import datetime
from typing import Optional

# Add parent dir (tools/) to path so rocbot_tuner is importable as a package
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nicegui import ui, app
import numpy as np

from rocbot_tuner.transport.serial import SerialTransport
from rocbot_tuner.models import ControllerState, MotorState
from rocbot_tuner.parser import parse_debug_line
from rocbot_tuner.logger import DataLogger
from rocbot_tuner.metrics import StepResponseMetrics, calculate_step_response

# ─── Configuration ───────────────────────────────────────────────────────

MAX_PLOT_POINTS = 500
PLOT_UPDATE_MS = 50

MOTOR_COLORS = {
    "FL": {"target": "#60a5fa", "rpm": "#1f77b4", "rpm_filt": "#1565c0", "pwr": "#1f77b4"},
    "FR": {"target": "#f87171", "rpm": "#d62728", "rpm_filt": "#b71c1c", "pwr": "#d62728"},
    "BL": {"target": "#4ade80", "rpm": "#2ca02c", "rpm_filt": "#1b5e20", "pwr": "#2ca02c"},
    "BR": {"target": "#fb923c", "rpm": "#ff7f0e", "rpm_filt": "#e65100", "pwr": "#ff7f0e"},
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
        self.transport: Optional[SerialTransport] = None
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
        self.mode = "STOP"  # STOP, PID, DIRECT

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

    def get_buffer(self, motor_id: str) -> MotorBuffer:
        if motor_id not in self.buffers:
            self.buffers[motor_id] = MotorBuffer()
        return self.buffers[motor_id]


state = AppState()

# ─── ECharts Configuration ──────────────────────────────────────────────


def build_rpm_chart() -> dict:
    return {
        "tooltip": {"trigger": "axis"},
        "legend": {"data": [], "top": 5},
        "grid": {"left": 50, "right": 20, "top": 35, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "RPM",
            "splitLine": {"lineStyle": {"type": "dashed"}},
        },
        "series": [],
    }


def build_pwr_chart() -> dict:
    return {
        "tooltip": {"trigger": "axis"},
        "legend": {"data": [], "top": 5},
        "grid": {"left": 50, "right": 20, "top": 35, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "PWM",
            "min": 0,
            "max": 280,
            "splitLine": {"lineStyle": {"type": "dashed"}},
        },
        "series": [],
    }


def build_error_chart() -> dict:
    return {
        "tooltip": {"trigger": "axis"},
        "legend": {"data": [], "top": 5},
        "grid": {"left": 50, "right": 20, "top": 35, "bottom": 30},
        "xAxis": {
            "type": "category",
            "name": "Time (s)",
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "name": "Error (RPM)",
            "splitLine": {"lineStyle": {"type": "dashed"}},
        },
        "series": [],
    }


# ─── UI Components ───────────────────────────────────────────────────────

# ECharts instances
rpm_chart: Optional[ui.echart] = None
pwr_chart: Optional[ui.echart] = None
error_chart: Optional[ui.echart] = None

# Status indicators
status_label: Optional[ui.label] = None
mode_label: Optional[ui.label] = None
log_label: Optional[ui.label] = None

# PID sliders
kp_slider: Optional[ui.number] = None
ki_slider: Optional[ui.number] = None
kd_slider: Optional[ui.number] = None
os_slider: Optional[ui.number] = None

# Motor state cards
motor_cards: dict[str, dict] = {}

# Metrics display
metrics_label: Optional[ui.markdown] = None

# ─── Chart Update ────────────────────────────────────────────────────────


def update_charts():
    """Update all three ECharts with current buffer data."""
    if not rpm_chart:
        return

    # RPM Chart
    rpm_option = build_rpm_chart()
    pwr_option = build_pwr_chart()
    error_option = build_error_chart()

    for motor_id, buf in state.buffers.items():
        if not buf.timestamps:
            continue
        colors = MOTOR_COLORS.get(motor_id, MOTOR_COLORS["FL"])
        time_labels = [f"{t:.1f}" for t in buf.timestamps]

        # RPM series
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

        # PWM series
        pwr_option["legend"]["data"].append(f"{motor_id} PWM")
        pwr_option["series"].append({
            "name": f"{motor_id} PWM",
            "type": "line",
            "data": [round(v, 1) for v in buf.pwr_filt],
            "lineStyle": {"width": 2},
            "itemStyle": {"color": colors["pwr"]},
            "symbol": "none",
        })

        # Error series
        error_option["legend"]["data"].append(f"{motor_id} Error")
        error_option["series"].append({
            "name": f"{motor_id} Error",
            "type": "line",
            "data": [round(v, 1) for v in buf.error],
            "lineStyle": {"width": 2},
            "itemStyle": {"color": colors["rpm"]},
            "symbol": "none",
        })

        # X-axis (use last motor's timestamps)
        rpm_option["xAxis"]["data"] = time_labels
        pwr_option["xAxis"]["data"] = time_labels
        error_option["xAxis"]["data"] = time_labels

    # NiceGUI 3.x: update options dict directly
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

        # Update buffers
        for motor_id, motor in new_state.motors.items():
            buf = state.get_buffer(motor_id)
            buf.update(motor, new_state.timestamp)

            # Step test buffer
            if state.step_test_active:
                if motor_id not in state.step_test_buffers:
                    state.step_test_buffers[motor_id] = MotorBuffer(maxlen=5000)
                state.step_test_buffers[motor_id].update(motor, new_state.timestamp)

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


# ─── UI Event Handlers ───────────────────────────────────────────────────


async def connect_serial():
    """Connect to ESP32."""
    port = port_input.value or "/dev/ttyUSB0"
    baud = int(baud_input.value or 115200)

    state.transport = SerialTransport(port=port, baud=baud)
    success = await state.transport.connect()

    if success:
        state.connected = True
        state.start_time = time.time()
        status_label.set_text("● Connected")
        status_label.style("color: green")
        ui.notify(f"Connected to {port}")

        # Start background reader
        asyncio.create_task(serial_reader())
    else:
        status_label.set_text("● Failed")
        status_label.style("color: red")
        ui.notify("Connection failed", type="negative")


async def disconnect_serial():
    """Disconnect from ESP32."""
    if state.transport:
        await state.transport.disconnect()
        state.connected = False
        status_label.set_text("● Disconnected")
        status_label.style("color: red")
        ui.notify("Disconnected")


async def send_pid_params():
    """Send current PID parameters to ESP32."""
    if not state.connected:
        ui.notify("Not connected", type="warning")
        return

    await state.transport.send_command(f"kp{state.kp}")
    await state.transport.send_command(f"ki{state.ki}")
    await state.transport.send_command(f"kd{state.kd}")
    await state.transport.send_command(f"os{state.output_scale}")
    ui.notify(f"PID: Kp={state.kp} Ki={state.ki} Kd={state.kd} OS={state.output_scale}")


async def set_mode_pid():
    """Enable PID mode."""
    if not state.connected:
        return
    await state.transport.send_command("p")
    state.mode = "PID"
    mode_label.set_text("Mode: PID")
    ui.notify("PID mode enabled")


async def set_mode_direct():
    """Enable direct PWM mode."""
    if not state.connected:
        return
    await state.transport.send_command(f"d{state.direct_pwm}")
    state.mode = "DIRECT"
    mode_label.set_text(f"Mode: DIRECT {state.direct_pwm}")
    ui.notify(f"Direct PWM: {state.direct_pwm}")


async def stop_motors():
    """Stop all motors."""
    if not state.connected:
        return
    await state.transport.send_command("s")
    state.mode = "STOP"
    mode_label.set_text("Mode: STOP")
    ui.notify("Motors stopped")


async def set_target():
    """Send target RPM."""
    if not state.connected:
        return
    await state.transport.send_command(str(state.target_rpm))
    ui.notify(f"Target: {state.target_rpm} RPM")


async def start_step_test():
    """Run a step response test."""
    if not state.connected:
        ui.notify("Not connected", type="warning")
        return

    # Clear step test buffers
    state.step_test_buffers.clear()
    state.step_test_active = True
    state.step_test_start = time.time()
    state.step_test_target = state.target_rpm

    # Enable PID and set target
    await state.transport.send_command("p")
    await asyncio.sleep(0.1)
    await state.transport.send_command(str(state.step_test_target))

    mode_label.set_text(f"Mode: STEP TEST → {state.step_test_target}")
    ui.notify(f"Step test started: {state.step_test_target} RPM")

    # Wait for test duration
    await asyncio.sleep(state.step_test_duration)

    # Stop and analyze
    await state.transport.send_command("s")
    state.step_test_active = False
    mode_label.set_text("Mode: STOP (test complete)")

    # Calculate metrics for each motor
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
    """Start/stop CSV logging."""
    if not state.logging:
        state.log_file = state.logger.start_session()
        state.logging = True
        log_label.set_text(f"📝 Logging: {os.path.basename(state.log_file)}")
        log_label.style("color: green")
        ui.notify("Logging started")
    else:
        state.logger.close()
        state.logging = False
        state.log_file = None
        log_label.set_text("📝 Logging: OFF")
        log_label.style("color: gray")
        ui.notify("Logging stopped")


async def clear_buffers():
    """Clear all plot buffers."""
    for buf in state.buffers.values():
        buf.clear()
    ui.notify("Buffers cleared")


# ─── Auto-update Timer ───────────────────────────────────────────────────


def auto_update():
    """Called periodically to update UI."""
    update_charts()
    update_motor_cards()


# ─── Main UI Layout ──────────────────────────────────────────────────────

# Global state for drawer toggle
sidebar_open = True

# Header
with ui.header().classes("items-center justify-between bg-gray-900 text-white px-4"):
    with ui.row().classes("items-center gap-3"):
        ui.button("☰", on_click=lambda: drawer.toggle()).props("flat dense color=white size=sm")
        ui.label("RocBot PID Tuner").classes("text-lg font-bold")
    with ui.row().classes("items-center gap-4"):
        status_label = ui.label("● Disconnected").classes("text-xs")
        mode_label = ui.label("Mode: STOP").classes("text-xs")
        log_label = ui.label("Logging: OFF").classes("text-xs text-gray-400")

# Left Drawer (collapsible side menu)
with ui.left_drawer(fixed=True).props("bordered").classes("bg-gray-900 w-64") as drawer:
    with ui.scroll_area().classes("fit p-3"):
        # Connection
        ui.label("Connection").classes("text-xs font-bold text-gray-400 mb-1 uppercase tracking-wide")
        port_input = ui.input(value="/dev/ttyUSB0").classes("w-full").props("dense outlined dark label=Port color=primary")
        baud_input = ui.input(value="115200").classes("w-full mt-1").props("dense outlined dark label=Baud color=primary")
        with ui.row().classes("gap-1 mt-1 w-full"):
            ui.button("Connect", on_click=connect_serial).props("dense color=green size=sm").classes("flex-1")
            ui.button("Disconnect", on_click=disconnect_serial).props("dense color=red size=sm").classes("flex-1")

        ui.separator().classes("my-2 bg-gray-700")

        # Mode
        ui.label("Mode").classes("text-xs font-bold text-gray-400 mb-1 uppercase tracking-wide")
        with ui.row().classes("gap-1"):
            ui.button("PID", on_click=set_mode_pid).props("dense color=blue size=sm").classes("flex-1")
            ui.button("Direct", on_click=set_mode_direct).props("dense color=orange size=sm").classes("flex-1")
            ui.button("Stop", on_click=stop_motors).props("dense color=red size=sm").classes("flex-1")

        ui.separator().classes("my-2 bg-gray-700")

        # Target
        ui.label("Target").classes("text-xs font-bold text-gray-400 mb-1 uppercase tracking-wide")
        ui.number(value=30, min=-330, max=330, step=1, format="%.0f").bind_value(state, "target_rpm").props("dense outlined dark label=RPM color=primary").classes("w-full")
        ui.button("Set Target", on_click=set_target).props("dense color=primary size=sm").classes("w-full mt-1")

        ui.separator().classes("my-2 bg-gray-700")

        # Direct PWM
        ui.label("Direct PWM").classes("text-xs font-bold text-gray-400 mb-1 uppercase tracking-wide")
        ui.number(value=100, min=0, max=255, step=1, format="%.0f").bind_value(state, "direct_pwm").props("dense outlined dark label=PWM color=primary").classes("w-full")

        ui.separator().classes("my-2 bg-gray-700")

        # PID Parameters
        ui.label("PID Parameters").classes("text-xs font-bold text-gray-400 mb-1 uppercase tracking-wide")
        ui.number(value=1.0, min=0, max=100, step=0.1, format="%.2f").bind_value(state, "kp").props("dense outlined dark label=Kp color=primary").classes("w-full")
        ui.number(value=0.0, min=0, max=10, step=0.01, format="%.3f").bind_value(state, "ki").props("dense outlined dark label=Ki color=primary").classes("w-full mt-1")
        ui.number(value=0.0, min=0, max=10, step=0.01, format="%.3f").bind_value(state, "kd").props("dense outlined dark label=Kd color=primary").classes("w-full mt-1")
        ui.number(value=10.0, min=1, max=100, step=1, format="%.0f").bind_value(state, "output_scale").props("dense outlined dark label=Output Scale color=primary").classes("w-full mt-1")
        ui.button("Apply PID", on_click=send_pid_params).props("dense color=primary size=sm").classes("w-full mt-1")

        ui.separator().classes("my-2 bg-gray-700")

        # Step Test
        ui.label("Step Test").classes("text-xs font-bold text-gray-400 mb-1 uppercase tracking-wide")
        ui.number(value=5.0, min=1, max=30, step=0.5, format="%.1f").bind_value(state, "step_test_duration").props("dense outlined dark label=Duration (s) color=primary").classes("w-full")
        ui.button("▶ Run Step Test", on_click=start_step_test).props("dense color=orange size=sm").classes("w-full mt-1")

        ui.separator().classes("my-2 bg-gray-700")

        # Utilities
        ui.label("Utilities").classes("text-xs font-bold text-gray-400 mb-1 uppercase tracking-wide")
        with ui.row().classes("gap-1"):
            ui.button("📝 Log", on_click=toggle_logging).props("dense color=secondary size=sm").classes("flex-1")
            ui.button("🗑 Clear", on_click=clear_buffers).props("dense color=grey size=sm").classes("flex-1")

# Main content area
with ui.column().classes("w-full flex-1 p-3 gap-3"):
    # Motor status bar
    with ui.row().classes("w-full gap-3"):
        for motor_id in ["FL", "FR"]:
            with ui.card().classes("flex-1 p-3"):
                with ui.row().classes("w-full items-center"):
                    ui.label(motor_id).classes("text-sm font-bold mr-4 w-8")
                    motor_cards[motor_id] = {
                        "rpm": ui.label("RPM: 0.0").classes("text-xs w-24"),
                        "filt": ui.label("F: 0.0").classes("text-xs w-24"),
                        "pwm": ui.label("PWM: 0.0").classes("text-xs w-24"),
                        "dir": ui.label("Dir: STP").classes("text-xs w-20"),
                        "err": ui.label("Err: 0.0").classes("text-xs"),
                    }

    # Charts - fill remaining space with explicit heights
    with ui.column().classes("w-full gap-2"):
        rpm_chart = ui.echart(build_rpm_chart()).classes("w-full h-80")
        pwr_chart = ui.echart(build_pwr_chart()).classes("w-full h-64")
        error_chart = ui.echart(build_error_chart()).classes("w-full h-64")

    # Metrics bar at bottom
    with ui.card().classes("w-full p-2"):
        with ui.row().classes("w-full items-center"):
            ui.label("Step Response").classes("text-xs font-bold mr-4")
            metrics_label = ui.markdown("Run a step test to see metrics.").classes("text-xs flex-1")

# ─── Auto-update timer ───────────────────────────────────────────────────

ui.timer(PLOT_UPDATE_MS / 1000, auto_update)

# ─── Launch ──────────────────────────────────────────────────────────────


def main():
    import argparse
    parser = argparse.ArgumentParser(description="RocBot PID Tuner")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--host", default="0.0.0.0", help="Web server host")
    parser.add_argument("--port-web", type=int, default=8080, help="Web server port")
    args = parser.parse_args()

    # Pre-fill port
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
