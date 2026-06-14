# RocBot Controller App — Flutter Multiplatform

A native mobile and desktop application for controlling the RocBot ESP32 motor controller from Android phones and Linux desktops.

**Status**: Planning  
**Location**: `tools/rocbot_controller/` (Flutter project)  
**Target Platforms**: Android, Linux (initial), iOS/macOS/Windows (future)

---

## Overview

The RocBot Controller App provides a native interface to monitor and control the robot's motors in real-time. It communicates directly with the ESP32 over WiFi, eliminating the need for a separate bridge server or micro-ROS agent for basic control.

### Key Features

- **Real-time motor monitoring**: RPM, PWM, direction, error for each wheel
- **Direct motor control**: Set target RPM, enable/disable PID, stop motors
- **PID tuning**: Adjust Kp, Ki, Kd gains with live feedback
- **Calibration wizard**: L298N channel imbalance measurement
- **Connection management**: Auto-discover ESP32 on local network, manual IP entry
- **Dark theme**: Cyberpunk aesthetic matching the existing PID tuner dashboard
- **Responsive layout**: Works on phones (portrait) and desktops (landscape)

---

## Architecture

### High-Level Structure

```
┌─────────────────────────────────────────────────────────┐
│                    Flutter App                           │
│                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  UI Layer    │  │  State Mgmt  │  │  Services    │  │
│  │  (Widgets)   │  │  (Provider/  │  │  (Network,   │  │
│  │              │  │   Riverpod)  │  │   Logging)   │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
│         │                │                  │           │
│         └────────────────┼──────────────────┘           │
│                          │                               │
│  ┌──────────────────────────────────────────────────┐   │
│  │              Communication Layer                  │   │
│  │  ┌──────────────┐  ┌──────────────────────────┐  │   │
│  │  │ TCP Client   │  │ Protocol Parser/Builder  │  │   │
│  │  │ (WiFi)       │  │ (JSON messages)          │  │   │
│  │  └──────┬───────┘  └──────────────────────────┘  │   │
│  └─────────┼────────────────────────────────────────┘   │
│            │                                             │
└────────────┼─────────────────────────────────────────────┘
             │ TCP (WiFi)
             │
┌────────────▼─────────────────────────────────────────────┐
│                    ESP32                                  │
│  ┌──────────────────────────────────────────────────┐   │
│  │  Motor Controller (main_controller.cpp)           │   │
│  │  - JSON command parser                           │   │
│  │  - Real-time state publisher (10 Hz)             │   │
│  │  - PID control loop (5 ms)                       │   │
│  └──────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

### Directory Structure

```
tools/rocbot_controller/
├── lib/
│   ├── main.dart                    # App entry point
│   ├── app.dart                     # App configuration, theme, routing
│   │
│   ├── models/                      # Data models
│   │   ├── motor_state.dart         # MotorState (RPM, PWM, direction, error)
│   │   ├── controller_state.dart    # ControllerState (PID gains, mode, target)
│   │   └── calibration_result.dart  # CalibrationResult (scales, efficiency)
│   │
│   ├── services/                    # Business logic & communication
│   │   ├── esp32_client.dart        # TCP client for ESP32 communication
│   │   ├── protocol.dart            # JSON message builder/parser
│   │   ├── discovery.dart           # mDNS/UDP discovery for ESP32
│   │   └── logger.dart              # Local data logging (CSV/JSON)
│   │
│   ├── providers/                   # State management (Riverpod)
│   │   ├── connection_provider.dart # Connection state, client instance
│   │   ├── motor_provider.dart      # Motor state data, history buffers
│   │   └── settings_provider.dart   # User preferences, saved connections
│   │
│   ├── screens/                     # Full-screen pages
│   │   ├── home_screen.dart         # Main dashboard with motor cards
│   │   ├── control_screen.dart      # Direct motor control (joystick, RPM)
│   │   ├── pid_tuning_screen.dart   # PID gain adjustment
│   │   ├── calibration_screen.dart  # Channel imbalance calibration
│   │   └── settings_screen.dart     # Connection settings, preferences
│   │
│   ├── widgets/                     # Reusable UI components
│   │   ├── motor_card.dart          # Single motor status display
│   │   ├── rpm_gauge.dart           # Circular RPM gauge widget
│   │   ├── pid_slider.dart          # PID gain slider with label
│   │   ├── status_indicator.dart    # Connection/motor status dot
│   │   └── chart_widget.dart        # Real-time line chart (fl_chart)
│   │
│   └── theme/                       # Visual theme
│       ├── colors.dart              # Neon color palette
│       ├── text_styles.dart         # Monospace font styles
│       └── app_theme.dart           # Material theme configuration
│
├── android/                         # Android platform code
├── linux/                           # Linux platform code
├── test/                            # Unit and widget tests
├── pubspec.yaml                     # Dependencies
└── README.md                        # Project documentation
```

---

## Communication Protocol

### Transport Layer

**Protocol**: TCP over WiFi  
**Port**: 8080 (configurable on ESP32)  
**Format**: JSON messages with newline delimiters

The ESP32 runs a TCP server that accepts one client at a time. Messages are newline-delimited JSON objects.

### Message Format

#### Client → ESP32 (Commands)

**Set Target RPM**
```json
{"cmd": "set_rpm", "target": 60}
```

**Set PID Gains**
```json
{"cmd": "set_pid", "kp": 1.0, "ki": 0.05, "kd": 0.01}
```

**Set Output Scale**
```json
{"cmd": "set_scale", "scale": 10.0}
```

**Enable PID Mode**
```json
{"cmd": "set_mode", "mode": "pid"}
```

**Stop Motors**
```json
{"cmd": "stop"}
```

**Request Status** (one-shot)
```json
{"cmd": "get_state"}
```

**Start Streaming** (continuous updates at specified rate)
```json
{"cmd": "stream_start", "interval_ms": 100}
```

**Stop Streaming**
```json
{"cmd": "stream_stop"}
```

#### ESP32 → Client (Responses)

**Motor State Update** (sent at stream rate or on request)
```json
{
  "type": "state",
  "timestamp": 12345,
  "mode": "pid",
  "target_rpm": 60,
  "kp": 1.0,
  "ki": 0.05,
  "kd": 0.01,
  "motors": {
    "FL": {
      "rpm": 58.5,
      "rpm_filt": 58.2,
      "pwm": 120.5,
      "pwm_filt": 118.3,
      "direction": "FWD",
      "pulses": 1234,
      "error": 1.5
    },
    "FR": {
      "rpm": 59.1,
      "rpm_filt": 58.8,
      "pwm": 105.2,
      "pwm_filt": 103.8,
      "direction": "FWD",
      "pulses": 1256,
      "error": 0.9
    }
  }
}
```

**Command Acknowledgment**
```json
{"type": "ack", "cmd": "set_rpm", "status": "ok"}
```

**Error Response**
```json
{"type": "error", "cmd": "set_pid", "message": "Invalid Kp value"}
```

### Why TCP + JSON?

1. **Simple**: No ROS 2 dependency on the mobile app
2. **Reliable**: TCP ensures ordered delivery (no dropped packets)
3. **Debuggable**: Human-readable JSON for easy debugging
4. **Cross-platform**: Works on Android, Linux, iOS, Windows, macOS
5. **Lightweight**: No DDS broker, no micro-ROS agent needed

---

## ESP32 Firmware Changes

A new firmware `main_controller.cpp` (or modifications to `main.cpp`) will be needed to support the JSON protocol.

### Required Features

1. **WiFi Access Point** or **WiFi Station** mode
   - Station mode: Connects to existing WiFi network
   - AP mode: Creates its own network (for direct phone connection)

2. **TCP Server** on port 8080
   - Accept one client at a time
   - Parse incoming JSON commands
   - Send JSON state updates

3. **Streaming Mode**
   - When client requests streaming, send state at specified interval
   - Default: 10 Hz (100ms interval)
   - Configurable: 1 Hz to 50 Hz

4. **JSON Parser**
   - Lightweight JSON library (ArduinoJson or cJSON)
   - Parse commands and build state responses

### Implementation Approach

```cpp
// Simplified structure
#include <WiFi.h>
#include <WiFiServer.h>
#include <ArduinoJson.h>  // or cJSON

WiFiServer tcpServer(8080);
WiFiClient client;

bool streaming = false;
unsigned long streamInterval = 100; // ms
unsigned long lastStreamTime = 0;

void setup() {
    // Connect to WiFi or start AP
    WiFi.begin(ssid, password);
    tcpServer.begin();
}

void loop() {
    // Accept new client
    if (!client.connected()) {
        client = tcpServer.available();
    }
    
    // Read and parse incoming commands
    if (client.available()) {
        String line = client.readStringUntil('\n');
        parseCommand(line);
    }
    
    // Send streaming data
    if (streaming && millis() - lastStreamTime >= streamInterval) {
        sendState();
        lastStreamTime = millis();
    }
}
```

---

## Flutter App Features

### 1. Home Screen (Dashboard)

**Layout**: Grid of motor status cards + connection status

```
┌─────────────────────────────────────────┐
│  RocBot Controller           ● Connected│
├─────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐      │
│  │    FL        │  │    FR        │      │
│  │  ⌀ 58.2 RPM │  │  ⌀ 58.8 RPM │      │
│  │  PWM: 118.3  │  │  PWM: 103.8  │      │
│  │  Dir: FWD    │  │  Dir: FWD    │      │
│  │  Err: 1.5    │  │  Err: 0.9    │      │
│  └─────────────┘  └─────────────┘      │
│                                          │
│  ┌─────────────────────────────────────┐│
│  │        RPM Chart (last 10s)         ││
│  │  ╭────────────────────────────────╮ ││
│  │  │    ╭──╮                        │ ││
│  │  │───╯  ╰─────────────────────── │ ││
│  │  │         ╭──╮                   │ ││
│  │  │────────╯  ╰───────────────────│ ││
│  │  ╰────────────────────────────────╯ ││
│  └─────────────────────────────────────┘│
│                                          │
│  Target: [60] RPM   [Set] [Stop] [PID]  │
└─────────────────────────────────────────┘
```

**Features**:
- Real-time RPM display with circular gauges
- Motor status cards (RPM, PWM, direction, error)
- Line chart showing RPM history (last 10 seconds)
- Quick controls: Set target RPM, Stop, Enable PID

### 2. Control Screen

**Layout**: Joystick control + direct PWM sliders

```
┌─────────────────────────────────────────┐
│  Motor Control                           │
├─────────────────────────────────────────┤
│                                          │
│  ┌─────────────────────────────────────┐│
│  │           Virtual Joystick           ││
│  │              ┌─────┐                 ││
│  │              │  ●  │                 ││
│  │              └─────┘                 ││
│  │      (drag to set direction/speed)   ││
│  └─────────────────────────────────────┘│
│                                          │
│  FL: [██░░░░░░] 120/255   [Individual]  │
│  FR: [██░░░░░░] 120/255   [Individual]  │
│                                          │
│  Mode: [PID] [Direct] [Stop All]        │
└─────────────────────────────────────────┘
```

**Features**:
- Virtual joystick for omnidirectional control (when 4 motors)
- Individual motor PWM sliders
- Mode selection (PID, Direct PWM, Stop)
- Emergency stop button (prominent)

### 3. PID Tuning Screen

**Layout**: Gain sliders + step test controls

```
┌─────────────────────────────────────────┐
│  PID Tuning                              │
├─────────────────────────────────────────┤
│  Kp: [━━━━━━━━●━━] 1.00                 │
│  Ki: [━━●━━━━━━━━] 0.05                 │
│  Kd: [●━━━━━━━━━] 0.01                  │
│  OS: [━━━━━━━━━●] 10.0                  │
│                                          │
│  [Apply] [Reset] [Step Test]            │
│                                          │
│  ┌─────────────────────────────────────┐│
│  │     Step Response (last test)        ││
│  │  Overshoot: 12.3%                    ││
│  │  Rise time: 0.45s                    ││
│  │  Settling: 1.2s                      ││
│  │  SS Error: 0.5 RPM                   ││
│  └─────────────────────────────────────┘│
└─────────────────────────────────────────┘
```

**Features**:
- Sliders for Kp, Ki, Kd, Output Scale
- Apply button to send gains to ESP32
- Step test with configurable target and duration
- Step response metrics display

### 4. Calibration Screen

**Layout**: Calibration wizard with progress

```
┌─────────────────────────────────────────┐
│  Calibration                             │
├─────────────────────────────────────────┤
│  This wizard measures L298N channel      │
│  imbalance and computes output scales.   │
│                                          │
│  Motors: [FL, FR]  PWM: [80,120,160]    │
│                                          │
│  [Start Calibration]                     │
│                                          │
│  Status: Testing PWM 120...              │
│  Progress: [████░░░░░░] 2/5             │
│                                          │
│  ┌─────────────────────────────────────┐│
│  │  Results (after completion)          ││
│  │  FL: os0.869                         ││
│  │  FR: os1.000                         ││
│  │  Imbalance: 13.1%                    ││
│  │                                      ││
│  │  [Apply Scales]                      ││
│  └─────────────────────────────────────┘│
└─────────────────────────────────────────┘
```

**Features**:
- Configure motors and PWM levels
- Run calibration sequence
- Display results with recommended scales
- Apply scales to ESP32

---

## Dependencies (pubspec.yaml)

```yaml
name: rocbot_controller
description: RocBot ESP32 Motor Controller App
version: 0.1.0

environment:
  sdk: '>=3.0.0 <4.0.0'

dependencies:
  flutter:
    sdk: flutter
  
  # State management
  flutter_riverpod: ^2.4.0
  
  # Networking
  dart_socket: ^1.0.0  # For TCP client
  
  # Data visualization
  fl_chart: ^0.65.0  # Real-time charts
  
  # UI components
  google_fonts: ^6.1.0  # Monospace fonts
  flutter_svg: ^2.0.0  # SVG icons
  
  # Storage
  shared_preferences: ^2.2.0  # Settings storage
  path_provider: ^2.1.0  # File paths
  
  # Utilities
  intl: ^0.19.0  # Number formatting
  json_annotation: ^4.8.0  # JSON serialization

dev_dependencies:
  flutter_test:
    sdk: flutter
  json_serializable: ^6.7.0
  build_runner: ^2.4.0
  flutter_lints: ^3.0.0
```

---

## Implementation Plan

### Phase 1: Foundation (Week 1)

- [ ] Create Flutter project structure
- [ ] Set up theme (cyberpunk neon colors, monospace fonts)
- [ ] Implement TCP client for ESP32 communication
- [ ] Create basic JSON protocol parser/builder
- [ ] Implement connection screen (manual IP entry)
- [ ] Create MotorState and ControllerState models
- [ ] Basic home screen with motor status display

**Deliverable**: App can connect to ESP32 and display motor state

### Phase 2: Dashboard (Week 2)

- [ ] Implement motor status cards with real-time updates
- [ ] Add RPM gauge widgets (circular progress indicators)
- [ ] Implement line chart for RPM history (fl_chart)
- [ ] Add quick controls (set RPM, stop, enable PID)
- [ ] Implement state management with Riverpod
- [ ] Add connection persistence (save last used IP)

**Deliverable**: Full dashboard with real-time charts and controls

### Phase 3: Control Features (Week 3)

- [ ] Implement PID tuning screen with sliders
- [ ] Add step test functionality
- [ ] Implement calibration wizard
- [ ] Add direct PWM control screen
- [ ] Implement virtual joystick (for future 4-motor)

**Deliverable**: Complete control features

### Phase 4: Polish & Android (Week 4)

- [ ] Android-specific setup (permissions, networking)
- [ ] Linux-specific setup (desktop window sizing)
- [ ] Add mDNS discovery for ESP32 auto-discovery
- [ ] Implement offline mode (view last logged data)
- [ ] Add data logging to local files
- [ ] Create app icons and splash screen
- [ ] Write comprehensive documentation

**Deliverable**: Production-ready app for Android and Linux

---

## ESP32 Firmware Changes Required

### New File: `main_controller.cpp`

A new firmware variant that:
1. Runs TCP server on port 8080
2. Parses JSON commands
3. Streams state updates in JSON format
4. Maintains existing motor control logic

### platformio.ini Addition

```ini
[env:controller]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200

; Build only main_controller.cpp
src_filter = +<main_controller.cpp> +<*.cpp> -<main.cpp> -<main_debug.cpp> -<main_microros.cpp>

; ArduinoJson for JSON parsing
lib_deps = bblanchon/ArduinoJson@^7.0.0
```

### Testing

```bash
# Build and flash
pio run -e controller -t upload

# Monitor serial
pio device monitor

# Test with netcat
echo '{"cmd":"get_state"}' | nc 192.168.1.100 8080
```

---

## Network Discovery

### mDNS (Recommended)

The ESP32 advertises itself via mDNS (Bonjour/Avahi):

**Service Name**: `_rocbot._tcp.local.`  
**Port**: 8080  
**TXT Records**:
- `version=1.0`
- `motors=FL,FR` (or `FL,FR,BL,BR` when expanded)

The Flutter app scans for `_rocbot._tcp.local.` services on the local network and presents a list of discovered robots.

### Fallback: Manual IP Entry

If mDNS is not available (some Android networks), users can manually enter the ESP32's IP address.

---

## Security Considerations

### Local Network Only

- App only communicates on local network (no internet)
- No authentication required (trusted environment)
- ESP32 only accepts one connection at a time

### Future Enhancements

- Add simple password authentication
- Encrypt communication with TLS
- Add connection timeout and reconnection logic

---

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| State update rate | 10 Hz | 100ms between updates |
| Chart refresh rate | 30 FPS | Smooth animation |
| Command latency | < 50ms | Round-trip to ESP32 |
| App startup | < 2s | Cold start to ready |
| Memory usage | < 100 MB | On Android |

---

## Testing Strategy

### Unit Tests

- Protocol parser/builder
- Motor state model
- Connection state management

### Widget Tests

- Motor card rendering
- Chart widget
- PID slider interaction

### Integration Tests

- Connect to ESP32 simulator
- Send commands and verify responses
- Stream data and verify chart updates

### Manual Testing

- Test on physical Android device
- Test on Linux desktop
- Test with actual ESP32 hardware

---

## Future Enhancements

### Phase 2 Features (After Initial Release)

1. **Bluetooth Support**: Connect via Bluetooth Classic or BLE
2. **ROS 2 Integration**: Optional ROS 2 transport (via micro-ROS agent)
3. **Multi-Robot**: Control multiple robots from one app
4. **Voice Commands**: "Set speed to 60 RPM"
5. **Camera Feed**: Display video from Luxonis OAK-D
6. **Map View**: 2D map with robot position (when LiDAR added)
7. **Autonomy Controls**: Start/stop autonomous navigation

### Platform Expansion

1. **iOS**: Same Flutter code, add iOS-specific setup
2. **macOS**: Desktop app for Mac
3. **Windows**: Desktop app for Windows
4. **Web**: PWA version (already planned in migration plan)

---

## References

- [Flutter Documentation](https://docs.flutter.dev/)
- [Riverpod State Management](https://riverpod.dev/)
- [fl_chart Documentation](https://pub.dev/packages/fl_chart)
- [ArduinoJson Documentation](https://arduinojson.org/)
- [mDNS in Flutter](https://pub.dev/packages/nsd_android)

---

## Summary

The RocBot Controller App provides a native, high-performance interface for controlling the robot from Android phones and Linux desktops. By using Flutter with TCP/JSON communication, we achieve:

1. **True native experience** on both platforms
2. **Simple, debuggable protocol** (JSON over TCP)
3. **No ROS 2 dependency** for basic control
4. **Excellent performance** for real-time visualization
5. **Easy extensibility** for future features

The app complements the existing PID tuner web dashboard and aligns with Phase 2 and Phase 5 of the RocBot migration plan.
