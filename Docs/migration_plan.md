# RocBot Migration Plan

Phased implementation roadmap from current serial-controlled motor driver to a fully integrated ROS 2 autonomous robot with web control.

---

## Phase 1: Current State ✅

**Status**: Working, deployed on ESP32 and Arduino Mega

- [x] Dual motor PID control (FL, FR) at 5ms loop
- [x] IIR low-pass filtering (RPM + power)
- [x] Serial protocol (`'g'` request, key:value response)
- [x] Multi-motor Python plotter
- [x] Encoder reading with direction detection
- [x] Motor parameter calibration (Kp=0.8, Ki=1.0, Kd=0.1)
- [x] ESP32 PlatformIO project (WiFi commented out)
- [x] I2C communication experiments
- [x] IMU BNO080 basic reading

**Remaining tasks in Phase 1**:
- [ ] Add BL and BR motor controllers (4-wheel configuration)
- [ ] Implement omnidirectional wheel kinematics (for signal generation)
- [ ] Refine PID gains for all 4 motors
- [ ] Migrate IMU from Arduino Mega to ESP32 I2C bus

---

## Phase 2: ESP32 WiFi Command Interface

**Goal**: Control the robot wirelessly via WiFi (no Jetson required for basic testing)

- [ ] Enable WiFi on ESP32 (create `ssid.hpp` with credentials)
- [ ] Implement TCP socket server on ESP32
- [ ] Accept RPM commands over TCP (same protocol as serial)
- [ ] Stream motor state over TCP to connected clients
- [ ] Write a simple Python test client for WiFi control
- [ ] Add OTA (over-the-air) firmware update capability
- [ ] Benchmark latency: WiFi vs serial

**Deliverable**: Robot controllable from any device on the local network via TCP socket.

---

## Phase 3: Jetson ROS 2 + micro-ROS

**Goal**: Replace direct serial/WiFi control with ROS 2 architecture using micro-ROS on ESP32

### 3a: Jetson Setup

- [ ] Install ROS 2 (Humble or Jazzy) on Jetson AGX Xavier
- [ ] Install `ros2_control` and `micro_ros_agent`
- [ ] Set up workspace (`colcon init`)
- [ ] Create `rocbot_bringup` package with launch files

### 3b: ESP32 micro-ROS Client

- [ ] Set up micro-ROS build environment (Docker or colcon workspace)
- [ ] Create micro-ROS client firmware for ESP32
  - [ ] Replace serial protocol with micro-ROS pub/sub
  - [ ] Publisher: `/joint_states` (wheel velocities)
  - [ ] Publisher: `/imu/data` (orientation from BNO080)
  - [ ] Subscriber: `/wheel_*/rpm_target` (per-wheel RPM setpoints)
- [ ] Configure UART transport between ESP32 and Jetson
- [ ] Verify: ESP32 publishes encoder data, receives RPM commands

### 3c: ROS 2 Control Integration

- [ ] Create `ros2_control` hardware interface for ESP32
  - [ ] `write()`: send target RPMs to micro-ROS topics
  - [ ] `read()`: receive measured RPMs from micro-ROS
- [ ] Configure controller: `diff_drive_controller` or custom omni controller
- [ ] Implement 4-wheel omnidirectional kinematics node
  - [ ] `/cmd_vel` (Twist) → per-wheel RPM (Float64)
  - [ ] Per-wheel RPM → `/odom` (Odometry)
- [ ] Launch file: start micro-ROS agent + ros2_control + kinematics

### 3d: IMU Integration

- [ ] Integrate BNO080 IMU with ESP32 I2C
- [ ] Publish `/imu/data` via micro-ROS
- [ ] IMU fusion with `robot_localization` on Jetson
  - [ ] `ekf_localization_node` fuses wheel odometry + IMU

**Deliverable**: Robot accepts `/cmd_vel` over ROS 2, runs control loop on ESP32, publishes odometry and IMU data.

---

## Phase 4: Extensible Control Algorithms

**Goal**: Improve motor control quality and support algorithm experimentation

- [ ] Refactor motor control to use the `ControlLaw` abstract interface
- [ ] Implement `FeedforwardPID` — velocity feedforward cancels known dynamics
- [ ] Implement gain scheduling — different PID gains for low/high speeds
- [ ] Add control algorithm selection parameter (runtime switchable)
- [ ] Implement cascaded PID (inner velocity loop + outer position loop)
- [ ] Benchmark: step response, tracking error, settling time for each algorithm
- [ ] Document algorithm performance with graphs in `Motors/graphs/`

**Deliverable**: Architecture supports hot-swappable control laws; robot tracks commands more precisely.

---

## Phase 5: Web App + PWA Control Interface

**Goal**: Control the robot from any browser on Android, Linux, and Windows

### 5a: WebSocket Bridge on Jetson

- [ ] Install `rosbridge_suite` on Jetson
- [ ] Launch `rosbridge_websocket` (exposes ROS 2 topics over WebSocket)
- [ ] Test: subscribe to `/joint_states` from browser
- [ ] Test: publish to `/cmd_vel` from browser

### 5b: PWA Frontend

- [ ] Create web app with framework of choice (React, Svelte, or vanilla)
- [ ] Implement virtual joystick (touch + mouse)
  - [ ] Touch events for mobile
  - [ ] Mouse drag for desktop
  - [ ] Optional: gamepad API for physical joystick
- [ ] Implement WebSocket client connecting to rosbridge
  - [ ] Publish: `/cmd_vel` from joystick input
  - [ ] Subscribe: `/joint_states` for RPM display
  - [ ] Subscribe: `/imu/data` for orientation display
- [ ] Build telemetry dashboard
  - [ ] Real-time RPM gauges for each wheel
  - [ ] Battery voltage display
  - [ ] Orientation indicator (roll/pitch/yaw)
- [ ] Add PWA manifest for installable app on Android
- [ ] Add camera stream display (future: from USB camera on Jetson)

### 5c: Direct ESP32 WiFi Fallback

- [ ] ESP32 serves a basic web interface when Jetson is not available
- [ ] Direct WebSocket connection to ESP32 (bypass Jetson)
- [ ] Useful for testing motor control without the full ROS stack

**Deliverable**: Full-stack robot control — open web app on phone/laptop, drive the robot with a virtual joystick.

---

## Phase 6: Full Autonomy

**Goal**: SLAM, path planning, autonomous navigation with obstacle avoidance

### 6a: Perception

- [ ] Integrate 2D LIDAR (RPLIDAR or similar) via USB to Jetson
- [ ] Integrate RGB-D camera (Intel RealSense or OAK-D) via USB/CSI
- [ ] Launch `slam_toolbox` for 2D SLAM (laser-based)
- [ ] Launch `rtabmap` for 3D SLAM (visual + laser)

### 6b: Navigation

- [ ] Configure `nav2` with:
  - [ ] Global planner (NavFn or Smac)
  - [ ] Local planner (DWA or Regulated Pure Pursuit)
  - [ ] Costmap layers (obstacles, inflation, static map)
  - [ ] Behavior tree (navigate to pose, follow path)
- [ ] Implement goal publishing from web app (click on map)
- [ ] Add emergency stop and safety behaviors

### 6c: Advanced Control

- [ ] Model Predictive Control (MPC) for trajectory tracking
- [ ] Integrate with `nav2` for smooth path following
- [ ] Adaptive control for changing payload/terrain

### 6d: System Hardening

- [ ] Watchdog timers on ESP32 and Jetson
- [ ] Battery monitoring and low-battery auto-dock
- [ ] Graceful degradation: if Jetson dies, ESP32 enters safe stop
- [ ] Logging and diagnostics (rosbag recording)

**Deliverable**: Fully autonomous mobile robot with web-based mission control.

---

## Summary Timeline

| Phase | What | Dependencies | Estimated Effort |
|-------|------|-------------|-----------------|
| 1 | Current state (complete remaining tasks) | None | 1-2 weeks |
| 2 | WiFi control | Phase 1 | 1 week |
| 3 | ROS 2 + micro-ROS | Phase 2 | 3-4 weeks |
| 4 | Advanced control algorithms | Phase 3 | 2-3 weeks |
| 5 | Web app + PWA | Phase 3 | 2-3 weeks |
| 6 | Full autonomy | Phase 3-5 | 4-8 weeks |

Phases 4 and 5 can be developed in parallel after Phase 3 is complete.
