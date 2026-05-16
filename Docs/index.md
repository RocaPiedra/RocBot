# RocBot Documentation

RocBot is a personal robotics project building an omnidirectional Autonomous Mobile Robot (AMR) from commercial, low-cost electronics.

## Project Goals

- Build a reliable 4-wheel omnidirectional robot platform
- Implement precise motor speed control via PID with low-pass filtering
- Integrate a BNO080/BNO085 IMU for orientation sensing
- Use ESP32 for WiFi-enabled remote control
- Eventually integrate laser rangefinders and cameras for autonomy
- A Jetson-class computer will serve as the high-level "brain"

## Current Status

The motor control firmware is the most mature subsystem. The project has evolved through several iterations:

| Version | Description |
|---|---|
| `motor_calibration/` | Initial single-motor PID calibration sketch |
| `pid_roc/` | Dual-motor capable PID with 50ms refresh |
| `pid_roc_filtered/` | Added IIR low-pass filters on RPM and power |
| `MotorControlPID/` | Object-oriented design with 5ms refresh, dual motor |

## Repository Layout

```
RocBot/
├── Docs/                    # This documentation
├── Diagrams/                # Hardware pinout and wiring images
├── Motors/                  # Motor control firmware (Arduino/AVR)
│   ├── MotorParameters.xml  # Motor specifications
│   ├── motor_calibration/   # Initial calibration sketch
│   ├── pid_roc/             # Basic PID controller
│   ├── pid_roc_filtered/    # PID with low-pass filtering
│   ├── MotorControlPID/     # Latest OOP motor control
│   │   └── src/             # C++ classes (MotorController, PIDClass, AuxFunctions)
│   ├── ArduinoMotorPlotter.py  # Multi-motor real-time plotter
│   └── graphs/              # Performance plots
├── IMU/
│   └── I2C/                 # IMU firmware and tools
│       ├── adafruit_demo/   # Adafruit BNO08x library demo
│       ├── imu_reader/      # Raw I2C IMU reader
│       └── IMUPlotter.py    # IMU data plotter
├── I2C/                     # I2C protocol experiments
│   ├── i2c_scanner/
│   ├── arduino_i2c_master/
│   └── arduino_i2c_slave/
├── ESP32/
│   └── ESP32WiFiConnect/    # PlatformIO ESP32 port (WiFi in progress)
│       └── src/             # Same C++ classes ported to ESP32
├── ArduinoPlotter.py         # Single-motor real-time plotter
└── README.md                # Original project README
```

## Subsystems

- [Motor Control](motor_control.md) - PWM, encoder reading, speed regulation
- [PID Controller](pid_controller.md) - PID gains, filtering, anti-windup
- [IMU and I2C](imu_and_i2c.md) - BNO080 communication, I2C protocol
- [ESP32 Port](esp32.md) - WiFi-enabled motor control
- [Plotting Tools](plotting_tools.md) - Real-time data visualization in Python
- [Hardware Reference](hardware.md) - Pin assignments, connections, parts

## Key Design Decisions

- **IIR low-pass filter** (y[n] = 0.854\*y[n-1] + 0.0728\*x[n] + 0.0728\*x[n-1]) applied to both RPM measurement and motor power output to reduce jitter
- **5ms control loop** in the latest version for responsive regulation
- **Single-channel encoder reading** with direction detection via channel B level
- **Object-oriented C++** in the latest version for multi-motor scalability
- **PlatformIO** for ESP32 development (Arduino IDE for AVR)

## Communication Protocol

Serial protocol between microcontroller and PC (or future Jetson):
- **Trigger**: Send `'g'` (0x67) byte
- **Response**: `&FL;target:60;rpm:123.4;rpm_filt:122.1;pwr:150;pwr_filt:148.2`
- Motor blocks separated by `&`, key:value pairs separated by `;`
- Integer input sets target RPM (clamped)
