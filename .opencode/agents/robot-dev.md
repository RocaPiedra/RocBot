---
name: robot-dev
description: RocBot robotics development — motor control, ESP32, ROS 2, and web app
---

# RocBot Robotics Development Agent

This agent contains the project context, build commands, and workflows for the RocBot autonomous mobile robot project.

## Project Overview

- **Goal**: Omnidirectional AMR with ESP32 motor control → ROS 2 on Jetson → web/mobile control
- **Hardware**: ESP32, Arduino Mega/Uno, Jetson AGX Xavier, L298N, BNO080 IMU, 4x DC motors with encoders
- **Languages**: C++ (Arduino/ESP32 firmware), Python (plotters, ROS 2 nodes), Web (JS/TS)

## Repository Layout

```
RocBot/
├── Docs/                          # Project documentation
│   ├── index.md                   # Overview and navigation
│   ├── architecture.md            # System architecture
│   ├── motor_control.md           # Motor control subsystem
│   ├── pid_controller.md          # PID implementation details
│   ├── imu_and_i2c.md             # IMU and I2C communication
│   ├── esp32.md                   # ESP32 port information
│   ├── plotting_tools.md          # Python real-time plotters
│   ├── hardware.md                # Hardware reference
│   └── migration_plan.md          # Implementation phases
├── Motors/
│   ├── MotorParameters.xml        # Motor specs (CPR:1320, RPM:330, 12V)
│   ├── MotorControlPID/           # Latest OOP motor control firmware
│   │   ├── MotorControlPID.ino    # Main sketch (2 motors, 5ms loop)
│   │   └── src/
│   │       ├── MotorController.*   # Motor class with PID + encoder
│   │       ├── PIDClass.*          # PID + IIR filter
│   │       └── AuxFunctions.*      # RPMtoMpS, plotter_logger
│   ├── pid_roc_filtered/          # PID with low-pass filters
│   ├── pid_roc/                   # Basic PID controller
│   ├── motor_calibration/         # Initial calibration sketch
│   ├── ArduinoMotorPlotter.py     # Multi-motor real-time plotter
│   └── graphs/                    # Performance plots (PNG)
├── IMU/I2C/
│   ├── adafruit_demo/             # Adafruit BNO08x library demo
│   ├── imu_reader/                # Raw I2C IMU reader
│   └── IMUPlotter.py              # IMU data plotter
├── I2C/
│   ├── i2c_scanner/               # I2C bus scanner
│   ├── arduino_i2c_master/        # I2C master transmitter
│   └── arduino_i2c_slave/         # I2C slave receiver
├── ESP32/ESP32WiFiConnect/        # PlatformIO ESP32 project
│   └── src/                       # Same C++ classes (ported)
├── ArduinoPlotter.py              # Single-motor real-time plotter
└── Diagrams/                      # Hardware reference images
```

## Build and Flash Commands

### ESP32 (PlatformIO)

```bash
cd ESP32/ESP32WiFiConnect
platformio run -t upload          # Build and flash
platformio device monitor         # Serial monitor (115200 baud)
pio run -t upload && pio device monitor  # Flash + monitor
```

### Arduino AVR (Arduino CLI)

```bash
# Single file sketches (in their directory)
arduino-cli compile --fqbn arduino:avr:uno Motors/motor_calibration/
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno Motors/motor_calibration/

# MotorControlPID (multi-file sketch with src/)
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 Motors/MotorControlPID/
```

### Python Plotters

```bash
# Multi-motor plotter (current: FL + FR)
python Motors/ArduinoMotorPlotter.py

# Single-motor plotter
python ArduinoPlotter.py
```

## Serial Protocol

| Direction | Format | Description |
|-----------|--------|-------------|
| PC → MCU | `'g'` (0x67) | Request motor state |
| PC → MCU | integer (e.g. `120`) | Set target RPM |
| MCU → PC | `&FL;target:60;rpm:123.4;rpm_filt:122.1;pwr:150;pwr_filt:148.2` | Motor state response |

Blocks separated by `&`, key:value pairs separated by `;`.

## Code Conventions

### PID Gains (calibrated Jan 22)
- Kp = 0.8, Ki = 1.0, Kd = 0.1
- Character: stable, slow, no offset error

### IIR Low-Pass Filter
```
y[n] = 0.854*y[n-1] + 0.0728*x[n] + 0.0728*x[n-1]
```
Applied to both RPM measurement and motor power.

### Encoder
- CPR: 1320 (raw), 330 (gearbox output = MAXCPR)
- RISING edge on channel A, direction from channel B

### Motor Control Loop Timing
- Latest: 5ms (MotorControlPID)
- Previous: 50ms (pid_roc), 100ms (motor_calibration)

### Pin Naming
- Motor ID: FL, FR, BL, BR (Front-Left, Front-Right, Back-Left, Back-Right)
- Pin naming: PWM, ENCA, ENCB, IN1, IN2

## Test Workflow

1. Flash firmware to ESP32 or Arduino
2. Open serial monitor to verify boot and communication
3. Send `'g'` byte to verify motor state response
4. Send target RPM value (e.g. `60`) to test motor response
5. Launch multi-motor plotter for real-time visualization
6. Verify filter behavior and PID tracking

## ROS 2 Setup (Future)

```bash
# On Jetson
sudo apt install ros-humble-ros2-control ros-humble-micro-ros-agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# Build custom packages
colcon build --packages-select rocbot_bringup rocbot_description
```

## Web App (Future)

```bash
# WebSocket bridge
ros2 launch web_bridge websocket.launch.py

# PWA (React/Svelte)
cd web_app
npm install && npm run dev
```

## Common Issues

- **Serial port permission**: `sudo usermod -aG dialout $USER` then logout/login
- **AVR memory limit**: Arduino Uno has 2KB RAM — use Mega or ESP32 for complex sketches
- **ESP32 flash**: Hold BOOT button while powering on, or use `--before default_reset`
- **I2C address conflict**: Use `i2c_scanner` sketch to verify device addresses
