# RocBot micro-ROS Setup

## Overview

This document describes how to set up and use micro-ROS with the RocBot ESP32 motor controller.

## Files

| File | Description |
|------|-------------|
| `main.cpp` | Standard version (WiFi connected, serial control) |
| `main_debug.cpp` | Debug version with detailed logging and test modes |
| `main_microros.cpp` | Full micro-ROS integration with ROS 2 |
| `motor_visualizer.py` | Python visualizer with logging |

## Building

### Standard Version
```bash
cd ESP32/ESP32WiFiConnect
pio run -t upload
```

### Debug Version
```bash
# Modify platformio.ini to use main_debug.cpp instead of main.cpp
# Or copy main_debug.cpp to main.cpp
pio run -t upload
```

### micro-ROS Version
```bash
# Build with micro-ROS environment
pio run -e microros -t upload
```

## micro-ROS Agent (on Jetson)

### Installation

```bash
# Install micro-ROS agent
sudo apt install ros-humble-micro-ros-agent

# Or build from source
cd ~/ros2_ws
git clone https://github.com/micro-ROS/micro_ros_agent.git
cd micro_ros_agent
colcon build
source install/setup.bash
```

### Running the Agent

```bash
# Connect ESP32 to Jetson via USB (ttyUSB1 at 115200 baud)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200

# Or with debug output
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200 -v
```

### Verify Connection

```bash
# List topics
ros2 topic list

# Should see:
# /motor_fl/target_rpm
# /motor_fr/target_rpm
# /motor_bl/target_rpm
# /motor_br/target_rpm
# /joint_states

# Echo joint states
ros2 topic echo /joint_states

# Set motor speed
ros2 topic pub /motor_fl/target_rpm std_msgs/Float64 "{data: 60}"
```

## Serial Connection (without micro-ROS)

The ESP32 can still be controlled via serial (fallback mode):

```bash
# Monitor serial output
pio device monitor

# Commands:
# g      - Get motor state
# 60     - Set target RPM (all motors)
# f60    - Set FL target RPM
# r60    - Set FR target RPM
# d100   - Direct PWM test (0-255)
# e      - Encoder debug info
# p      - Enable PID mode
# r      - Reverse test
# s      - Stop
```

## Python Visualizer

### Installation
```bash
pip install pyserial matplotlib
```

### Running
```bash
# Default (Linux)
python motor_visualizer.py

# Windows
python motor_visualizer.py --port COM5

# Logging only (no plot)
python motor_visualizer.py --no-plot

# Custom log file
python motor_visualizer.py --log /path/to/log.csv
```

### Output
- Real-time plots: RPM vs Target, Power, Error
- Log file: `logs/motor_log_YYYYMMDD_HHMMSS.csv`

## Troubleshooting

### Motors always turn in one direction

1. **Check L298N wiring**:
   - IN1=HIGH, IN2=LOW = Forward
   - IN1=LOW, IN2=HIGH = Reverse

2. **Check encoder direction**:
   - ENCB level on ENCA rising edge determines direction
   - If always incrementing, encoder direction is wrong

3. **Use debug mode**:
   ```bash
   # Upload main_debug.cpp
   # Send 'e' to see encoder values
   # Send 'r' to test reverse direction
   # Send 'd100' for direct PWM test
   ```

### micro-ROS connection fails

1. Check ESP32 is connected to Jetson USB
2. Verify correct tty device: `ls -la /dev/ttyUSB*`
3. Check baud rate matches (115200)
4. Run agent with verbose: `-v`
5. Check ESP32 serial output for errors

### Performance issues

- Reduce DEBUG_PRINT_INTERVAL in code
- Use filtered RPM values for control
- Ensure 5ms control loop is consistent