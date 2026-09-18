# RocBot System Architecture

## Overview

RocBot is an omnidirectional Autonomous Mobile Robot (AMR) built from low-cost, commercial electronics. The architecture is designed for incremental migration from a simple serial-controlled motor driver to a fully integrated ROS 2 system with micro-ROS, web-based control, and advanced control algorithms.

```
┌─────────────────────────────────────────────────────────┐
│                    User Interfaces                        │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────┐  │
│  │ PWA Web App  │  │ Laptop/PC    │  │ Gamepad/Joystick│  │
│  │ (Android,    │  │ (Linux, Win) │  │ (Bluetooth/USB) │  │
│  │  Linux, Win) │  │              │  │                │  │
│  └──────┬───────┘  └──────┬───────┘  └───────┬────────┘  │
│         │                │                  │           │
│         └────────────────┼──────────────────┘           │
│                          │                               │
│  ┌──────────────────────────────────────────────────┐   │
│  │         PID Tuner Dashboard (NiceGUI)             │   │
│  │  - Real-time RPM/PWM/Error charts                │   │
│  │  - L298N channel calibration                     │   │
│  │  - Auto-tuning (ZN / Relay / Cohen-Coon)         │   │
│  │  - Serial (USB) or ROS2 (WiFi/UDP) transport     │   │
│  └──────────────────┬───────────────────────────────┘   │
│                     │ HTTP :8080                          │
└─────────────────────┼──────────────────────────────────┘
                           │
┌──────────────────────────┼──────────────────────────────┐
│                    Jetson AGX Xavier                     │
│  ┌──────────────────────────────────────────────────┐   │
│  │              ROS 2 (Humble/Jazzy)                 │   │
│  │                                                    │   │
│  │  ┌──────────────┐   ┌─────────────────────────┐   │   │
│  │  │ ros2_control  │   │  micro-ROS Agent (UART) │   │   │
│  │  │              │   │                         │   │   │
│  │  │ cmd_vel →    │   │  Serial bridge to ESP32 │   │   │
│  │  │ wheel RPMs   │   │                         │   │   │
│  │  │ via kinematics│  │                         │   │   │
│  │  └──────┬───────┘   └───────────┬─────────────┘   │   │
│  │         │                       │                  │   │
│  │  ┌──────┴───────┐   ┌───────────┴─────────────┐   │   │
│  │  │ Navigation   │   │  WebSocket Server        │   │   │
│  │  │ (SLAM, path) │   │  (for web app bridge)    │   │   │
│  │  └──────────────┘   └─────────────────────────┘   │   │
│  └──────────────────────────────────────────────────┘   │
└──────────────────────────┬──────────────────────────────┘
                           │ UART (Serial)
┌──────────────────────────┼──────────────────────────────┐
│                    ESP32                                │
│  ┌──────────────────────────────────────────────────┐   │
│  │              FreeRTOS on ESP32                    │   │
│  │                                                    │   │
│  │  ┌───────── Core 1 ──────────┐  ┌── Core 0 ────┐ │   │
│  │  │ micro-ROS Client          │  │ Motor Control │ │   │
│  │  │  - subscribes: wheel RPM  │  │  - 5ms loop  │ │   │
│  │  │  - publishes: joint_states│  │  - encoders  │ │   │
│  │  │  - publishes: imu/data    │  │  - PID loop  │ │   │
│  │  │  - WiFi (optional)        │  │  - control law│ │   │
│  │  └──────────────────────────┘  └──────┬───────┘ │   │
│  └──────────────────────────────────────────────────┘   │
│                       │                                 │
│         ┌─────────────┴─────────────┐                   │
│         │                           │                   │
│    ┌────┴──────────────────┐  ┌─────┴─────────────────┐ │
│    │ L298N #1 (front)      │  │ L298N #2 (rear)       │ │
│    │ ch A: Motor FR        │  │ ch A: Motor RR        │ │
│    │ ch B: Motor FL        │  │ ch B: Motor RL        │ │
│    │ + encoders            │  │ + encoders            │ │
│    └───────────────────────┘  └───────────────────────┘ │
│                                                         │
│    ┌────────────────────────────────────────────┐       │
│    │  IMU BNO080 (I2C: 0x4B)                    │       │
│    └────────────────────────────────────────────┘       │
└──────────────────────────────────────────────────────────┘
```

## Hardware Architecture

### Microcontrollers

| Device | Role | Interface |
|--------|------|-----------|
| **ESP32** | Real-time motor control + micro-ROS client | UART ↔ Jetson, I2C ↔ IMU, GPIO ↔ motors |
| **Jetson AGX Xavier** | High-level ROS 2 (nav, perception, control) | UART ↔ ESP32, USB/CSI ↔ cameras |

### Motor Configuration (4-wheel omnidirectional)

Each wheel has:
- DC motor with quadrature encoder (CPR: 1320 raw, 330 gearbox output)
- L298N H-bridge driver (PWM + 2 direction pins)
- Encoder channels A (interrupt, RISING) + B (direction)

### Pin Assignments (ESP32 — 4-motor omnidirectional)

L298N terminal strip per driver is `ENA – IN1 – IN2 – IN3 – IN4 – ENB`
(channel A = `ENA/IN1/IN2`, channel B = `IN3/IN4/ENB`).
`IN1/IN2` below are the firmware's per-motor direction-pin names.

| Motor | Driver / Channel | PWM (ENAx) | ENCA | ENCB | IN1 | IN2 |
|-------|------------------|------------|------|------|-----|-----|
| FR    | L298N #1, ch A | 14 (ENA) | 22 | 23 | 27 (IN1) | 26 (IN2) |
| FL    | L298N #1, ch B | 32 (ENB) | 35 | 34 | 33 (→IN4 ⚠️) | 25 (→IN3 ⚠️) |
| RR    | L298N #2, ch A | 18 (ENA) | 36 (VP) | 39 (VN) | 19 (IN1) | 21 (IN2) |
| RL    | L298N #2, ch B | 13 (ENB) | 16 (RX2) | 17 (TX2) | 4 (IN3) | 5 (IN4) |

Full terminal-level wiring + jumpers: see
[Electric Components](electric_components.md). FL is cross-wired
(IN3 = 25, IN4 = 33) for the mirror-mounted motors — verified
(`D100` = both front wheels forward). RL is straight, rear
still untested.

### IMU

- BNO080/BNO085 via I2C (address 0x4B)
- SCL=D20, SDA=D21 on Arduino Mega (current)
- Will migrate to ESP32 I2C bus

### Jetson ↔ ESP32 Connection

- UART serial (TX/RX, 2 wires + GND)
- Baud rate: 115200 (or higher: 460800/921600 for lower latency)
- No level shifting needed (both are 3.3V logic)

### PC/Dev ↔ ESP32 (PID Tuner Connection)

The `rocbot_tuner` web dashboard connects to the ESP32 via one of two transports:

| Transport | Connection | When to Use |
|-----------|-----------|-------------|
| **Serial (USB)** | USB cable directly to ESP32 (appears as `/dev/ttyUSB0`) | Tuning with debug firmware (`main_debug.cpp`) |
| **ROS2 (WiFi/UDP)** | ESP32 connects to WiFi, communicates via micro-ROS agent Docker container on the PC | Testing the micro-ROS firmware (`main_microros.cpp`) |

The tuner runs as a NiceGUI web app on the development PC at **http://localhost:8080**.

---

## Firmware Architecture (ESP32)

### FreeRTOS Task Layout

| Core | Task | Priority | Period | Description |
|------|------|----------|--------|-------------|
| 0 | MotorControl | high | 5ms | Read encoders, compute control law, set PWM |
| 0 | EncoderISR | highest | interrupt | RISING edge on encoder channel A |
| 0 | IMUReader | medium | 10ms | Read BNO080 over I2C, update orientation |
| 1 | MicroROS | high | event-driven | micro-ROS spin, pub/sub over UART |
| 1 | Watchdog | low | 1000ms | Monitor task health, report status |

### Control Loop (Core 0, 5ms)

```
for each motor:
  deltaT = micros() - prevT
  
  if deltaT >= 5ms:
    // Update RPM from encoder pulses
    pulses = atomic_read(encoder_count)
    currentRPM = (pulses / max_cpr) * (1000 / deltaTms) * 60
    filteredRPM = lowpass_filter(currentRPM)
    atomic_reset(encoder_count)
    
    // Compute control output
    output = controlLaw.calculate(filteredRPM, targetRPM, deltaT)
    
    // Apply PWM
    setDirection(output > 0)
    pwmValue = constrain(abs(output), 0, 255)
    analogWrite(pwmPin, pwmValue)
```

### Control Law Interface

The architecture supports swapping control algorithms via a common interface:

```
class ControlLaw {
  virtual float calculate(float measured, float target, float dt) = 0;
  virtual void reset() = 0;
  virtual String getName() = 0;
};
```

Available implementations:
- **PIDClass** — current implementation (Kp=0.8, Ki=1.0, Kd=0.1 + IIR filters)
- Future: Feedforward + PID, Cascaded PID, LQR, Gain-scheduled PID, Adaptive control

### micro-ROS Client (Core 1)

Subscribes to:
- `/wheel_fl/rpm_target` — Float64
- `/wheel_fr/rpm_target` — Float64
- `/wheel_bl/rpm_target` — Float64
- `/wheel_br/rpm_target` — Float64

Publishes:
- `/joint_states` — JointState (wheel velocities)
- `/imu/data` — Imu (orientation, angular velocity, acceleration)
- `/wheel_fl/rpm` — Float64 (measured)
- `/wheel_fr/rpm` — Float64
- `/wheel_bl/rpm` — Float64
- `/wheel_br/rpm` — Float64

---

## ROS 2 Integration (Jetson AGX Xavier)

### Components

| Package | Purpose |
|---------|---------|
| `micro_ros_agent` | Serial bridge to ESP32 (UART) |
| `ros2_control` | Hardware abstraction, controller management |
| `robot_localization` | Sensor fusion (IMU + wheel odometry) |
| `nav2` | SLAM, path planning, autonomous navigation |
| `web_bridge` | WebSocket server for web app |

### Data Flow: `/cmd_vel` → Wheel RPMs

```
/cmd_vel (Twist)
    ↓
omnidirectional_kinematics_node
    ↓  (4-wheel inverse kinematics)
/wheel_fl/rpm_target  /wheel_fr/rpm_target  /wheel_bl/rpm_target  /wheel_br/rpm_target
    ↓
micro_ros_agent (serializes over UART)
    ↓
ESP32 micro-ros client → MotorControl → PWM
```

### Data Flow: Wheel RPMs → `/odom`

```
ESP32 encoders → micro-ROS → /joint_states
    ↓
robot_localization (wheel odometry)
    ↓
/odom (filtered odometry with IMU correction)
```

---

## Web Control App

### Architecture

A Progressive Web Application (PWA) that runs in any modern browser on Android, Linux, and Windows.

```
┌─────────────────────────────────────────────────────┐
│                PWA Web Application                   │
│                                                      │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────┐ │
│  │ Virtual      │  │ Telemetry    │  │ Camera     │ │
│  │ Joystick     │  │ Display      │  │ Feed       │ │
│  │ (touch/mouse)│  │ (RPM, bat,   │  │ (future)   │ │
│  │              │  │  orientation)│  │            │ │
│  └──────┬───────┘  └──────────────┘  └────────────┘ │
│         │                                            │
│  ┌──────┴───────────────────────────────────────┐    │
│  │          WebSocket Client                    │    │
│  └──────────────────┬──────────────────────────┘    │
└─────────────────────┼───────────────────────────────┘
                      │ WebSocket
┌─────────────────────┼───────────────────────────────┐
│          Jetson AGX Xavier                          │
│  ┌──────────────────┴──────────────────────────┐    │
│  │          WebSocket Bridge Node              │    │
│  │  rosbridge_websocket or custom node         │    │
│  └─────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

### Technology Options

| Phase | Stack | Pros |
|-------|-------|------|
| Quick prototype | React + WebSocket + Canvas | Fast to build, huge ecosystem |
| Full PWA | SvelteKit + WebSocket | Lighter than React, better mobile feel |
| Native feel | Flutter + WebSocket | Single codebase, native widgets everywhere |

### Communication

- **Primary**: WebSocket ↔ `rosbridge_websocket` on Jetson (uses ROS 2 topics)
- **Fallback**: WebSocket directly to ESP32 (no Jetson needed, for testing)

---

## Communication Protocols

### ESP32 ↔ Jetson (UART)

Using micro-ROS serial transport (CDR serialization over UART):
- Framing: uXRCE-DDS packet format
- Transport: UART at 115200+ baud
- Topics: serialized as CDR (efficient binary)

### Jetson ↔ Web App (WebSocket)

Using `rosbridge_websocket` protocol:
- JSON messages over WebSocket
- Subscribe/publish to any ROS 2 topic
- `/cmd_vel` for velocity commands
- `/joint_states` for wheel feedback

---

## Extensibility

### Control Algorithms

The `ControlLaw` interface allows swapping in different algorithms:

```cpp
// Current:
PIDClass pid(kp, ki, kd);
controlLaw = &pid;

// Future:
FeedforwardPID ffpid(kp, ki, kd, feedforward_gain);
controlLaw = &ffpid;

// Future:
LQRController lqr(Q_matrix, R_matrix);
controlLaw = &lqr;

// Future:
GainScheduledPID gspid(low_speed_gains, high_speed_gains, threshold);
controlLaw = &gspid;
```

### Additional Hardware

- Lasers (LIDAR) — USB or serial to Jetson
- Cameras (RGB-D) — USB or CSI to Jetson
- Additional sensors — I2C or SPI to ESP32 or Jetson
