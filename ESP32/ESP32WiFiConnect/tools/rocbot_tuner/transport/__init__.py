# RocBot Transport Layer
"""
Transport abstraction for RocBot motor controller communication.

Current: SerialTransport (ESP32 debug firmware over USB serial)
Future:  Ros2Transport (micro-ROS agent over DDS)
"""

from rocbot_tuner.models import MotorState
from .base import Transport
from .serial import SerialTransport

__all__ = ['Transport', 'MotorState', 'SerialTransport']
