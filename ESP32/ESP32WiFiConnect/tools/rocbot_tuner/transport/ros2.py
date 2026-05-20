"""ROS2 transport for micro-ROS agent."""

import asyncio
import time
from typing import Callable, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64, String
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False

from rocbot_tuner.models import MotorState, ControllerState
from .base import Transport


class _Ros2Bridge(Node):
    """Internal ROS2 node that bridges topics to callbacks."""

    def __init__(self):
        super().__init__("rocbot_tuner")
        self._callbacks: list[Callable[[ControllerState], None]] = []

        # Subscribers (simplified to Float64 per motor)
        self._sub_fl_rpm = self.create_subscription(
            Float64, "rocbot/motor_fl/rpm", self._on_fl_rpm, 10)
        self._sub_fl_pwm = self.create_subscription(
            Float64, "rocbot/motor_fl/pwm", self._on_fl_pwm, 10)
        self._sub_fr_rpm = self.create_subscription(
            Float64, "rocbot/motor_fr/rpm", self._on_fr_rpm, 10)
        self._sub_fr_pwm = self.create_subscription(
            Float64, "rocbot/motor_fr/pwm", self._on_fr_pwm, 10)
        self._sub_debug = self.create_subscription(
            String, "rocbot/debug", self._on_debug, 10)

        # Publishers
        self._pub_command = self.create_publisher(String, "rocbot/command", 10)

        # State
        self._fl_state = MotorState(motor_id="FL")
        self._fr_state = MotorState(motor_id="FR")
        self._last_emit = 0.0
        self._emit_interval = 0.05  # 20 Hz

    def register_callback(self, callback: Callable[[ControllerState], None]):
        self._callbacks.append(callback)

    def _on_fl_rpm(self, msg: Float64):
        self._fl_state.rpm = msg.data
        self._fl_state.rpm_filt = msg.data  # Approximation for now
        self._try_emit()

    def _on_fl_pwm(self, msg: Float64):
        self._fl_state.pwr_filt = msg.data
        self._try_emit()

    def _on_fr_rpm(self, msg: Float64):
        self._fr_state.rpm = msg.data
        self._fr_state.rpm_filt = msg.data
        self._try_emit()

    def _on_fr_pwm(self, msg: Float64):
        self._fr_state.pwr_filt = msg.data
        self._try_emit()

    def _on_debug(self, msg: String):
        # Parse debug string for mode/pid info
        import re
        state = ControllerState()
        state.timestamp = time.time()

        if "STEP_TEST->" in msg.data:
            state.mode = "STEP_TEST"
        elif "DIRECT:" in msg.data:
            state.mode = "DIRECT"
        elif " PID" in msg.data:
            state.mode = "PID"
        elif " STOP" in msg.data:
            state.mode = "STOP"

        kp_match = re.search(r"Kp:([\d.]+)", msg.data)
        ki_match = re.search(r"Ki:([\d.]+)", msg.data)
        kd_match = re.search(r"Kd:([\d.]+)", msg.data)
        tgt_match = re.search(r"Tgt:(-?\d+)", msg.data)
        if kp_match: state.kp = float(kp_match.group(1))
        if ki_match: state.ki = float(ki_match.group(1))
        if kd_match: state.kd = float(kd_match.group(1))
        if tgt_match: state.target_value = float(tgt_match.group(1))

        # Attach motor states
        self._fl_state.target_rpm = state.target_value
        self._fr_state.target_rpm = state.target_value
        state.motors["FL"] = self._fl_state
        state.motors["FR"] = self._fr_state

        if state.motors:
            for cb in self._callbacks:
                cb(state)

    def _try_emit(self):
        now = time.time()
        if now - self._last_emit < self._emit_interval:
            return

        self._last_emit = now
        state = ControllerState()
        state.timestamp = now
        state.motors["FL"] = self._fl_state
        state.motors["FR"] = self._fr_state

        for cb in self._callbacks:
            cb(state)

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._pub_command.publish(msg)


class Ros2Transport(Transport):
    """ROS2 communication via micro-ROS agent."""

    def __init__(self):
        self._node: Optional[_Ros2Bridge] = None
        self._running = False
        self._spin_task: Optional[asyncio.Task] = None

    @property
    def is_connected(self) -> bool:
        return self._node is not None and rclpy.ok()

    async def connect(self) -> bool:
        if not HAS_RCLPY:
            print("rclpy not available. Install ROS2 Humble or run: sudo apt install ros-humble-rclpy")
            return False

        try:
            # Fix: Only init if context is not already active
            if not rclpy.ok():
                rclpy.init()
            
            self._node = _Ros2Bridge()
            self._running = True
            self._spin_task = asyncio.create_task(self._spin_loop())
            print("ROS2 transport connected")
            return True
        except Exception as e:
            print(f"ROS2 connect failed: {e}")
            return False

    async def disconnect(self):
        self._running = False
        if self._spin_task:
            self._spin_task.cancel()
            try:
                await self._spin_task
            except asyncio.CancelledError:
                pass
        if self._node:
            self._node.destroy_node()
            self._node = None
            
        # Fix: Properly shutdown context to allow reconnection
        if rclpy.ok():
            rclpy.shutdown()
            
        print("ROS2 transport disconnected")

    async def send_command(self, cmd: str):
        if self._node:
            self._node.send_command(cmd)

    async def read_loop(self, callback: Callable[[ControllerState], None]):
        if self._node:
            self._node.register_callback(callback)
        while self._running:
            await asyncio.sleep(0.1)

    async def _spin_loop(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                break
            await asyncio.sleep(0.01)
