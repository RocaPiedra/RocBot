"""Serial transport for ESP32 debug firmware."""

import asyncio
import inspect
import time
from typing import Awaitable, Callable, Optional

import serial

from rocbot_tuner.models import MotorState, ControllerState
from .base import Transport
from ..parser import parse_debug_line, parse_motor_state_block


class SerialTransport(Transport):
    """Serial communication with ESP32 debug firmware."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._start_time = 0.0

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    async def connect(self) -> bool:
        """Connect to ESP32 serial port."""
        for attempt in range(10):
            try:
                self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
                # Wait for ESP32 to boot
                await asyncio.sleep(1.0)
                # Drain any boot messages
                self._serial.reset_input_buffer()
                self._start_time = time.time()
                self._running = True
                print(f"Connected to {self.port} at {self.baud} baud")
                return True
            except serial.SerialException as e:
                print(f"Connect attempt {attempt + 1}/10 failed: {e}")
                await asyncio.sleep(1)
        return False

    async def disconnect(self):
        """Close serial connection."""
        self._running = False
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
            print("Disconnected")

    async def send_command(self, cmd: str):
        """Send command to ESP32."""
        if self._serial and self._serial.is_open:
            self._serial.write((cmd + "\n").encode())
            await asyncio.sleep(0.01)  # Small delay for ESP32 to process

    async def read_loop(self, callback: Callable[[ControllerState], None | Awaitable[None]]):
        """Read serial data and parse into ControllerState objects."""
        buffer = ""
        lines_parsed = 0
        lines_matched = 0

        while self._running:
            try:
                if self._serial and self._serial.in_waiting:
                    data = self._serial.read(self._serial.in_waiting)
                    text = data.decode("utf-8", errors="ignore")
                    buffer += text

                    # Process complete lines
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        lines_parsed += 1
                        state = self._parse_line(line)
                        if state:
                            lines_matched += 1
                            result = callback(state)
                            if inspect.isawaitable(result):
                                await result

                await asyncio.sleep(0.005)  # 5ms poll interval
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break
            except Exception as e:
                print(f"Read error: {e}")
                break

        print(f"Serial reader stopped. Parsed: {lines_parsed}, Matched: {lines_matched}")

    def _parse_line(self, line: str) -> Optional[ControllerState]:
        """Parse a single line from serial output."""
        # Debug format: --- DEBUG T:12345 --- PID Tgt:30 Kp:1.00 ... | FL RPM:28.5 F:28.1 PWM:120.5 ...
        if line.startswith("--- DEBUG"):
            return parse_debug_line(line, self._start_time)

        # Motor state format: &FL;target:30;rpm:28.5;...
        if line.startswith("&"):
            return self._parse_motor_state_line(line)

        return None

    def _parse_motor_state_line(self, line: str) -> Optional[ControllerState]:
        """Parse motor state response from 'g' command."""
        state = ControllerState()
        state.timestamp = time.time() - self._start_time

        blocks = line.split("&")
        for block in blocks:
            block = block.strip()
            if not block:
                continue
            motor = parse_motor_state_block(block)
            if motor:
                state.motors[motor.motor_id] = motor

        return state if state.motors else None
