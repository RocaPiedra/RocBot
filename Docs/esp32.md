# ESP32 Port

## Overview

The motor control firmware has been ported to the ESP32 as a PlatformIO project. This enables WiFi-based remote control, which will be the foundation for the Jetson integration.

## Project Structure

```
ESP32WiFiConnect/
├── platformio.ini           # PlatformIO configuration
├── include/                 # Standard PlatformIO include dir
│   └── ssid.hpp             # WiFi credentials (gitignored)
├── src/
│   ├── main.cpp             # ESP32 main sketch
│   ├── MotorController.cpp  # Same as AVR version
│   ├── MotorController.hpp
│   ├── PIDClass.cpp         # Same as AVR version
│   ├── PIDClass.hpp
│   ├── AuxFunctions.cpp     # Same as AVR version
│   └── AuxFunctions.hpp
├── lib/                     # PlatformIO library dir (empty)
├── test/                    # PlatformIO test dir (empty)
└── RocBot.code-workspace    # VS Code multi-root workspace
```

## Differences from AVR Version

| Aspect | AVR (Arduino Uno/Mega) | ESP32 |
|--------|----------------------|-------|
| Framework | Arduino IDE | PlatformIO + Arduino |
| PWM | TCCR1B register config | Automatic (ESP32 LEDC) |
| Atomic blocks | `ATOMIC_BLOCK()` | Not needed (single core) |
| WiFi | N/A | Planned (commented out) |
| Speed | 16 MHz | 240 MHz |

## WiFi (In Progress)

The WiFi connection code is **currently commented out** in `main.cpp`. When enabled:

- Uses `WiFiMulti` for connection management
- Credentials stored in `include/ssid.hpp` (gitignored for security)
- Will accept serial commands over TCP

## Pin Assignments

GPIO numbers on ESP32 (4-motor omnidirectional, see `AGENTS.md`):

| Motor | PWM | ENCA | ENCB | IN1 | IN2 |
|-------|-----|------|------|-----|-----|
| FR    | 14  | 22   | 23   | 27  | 26  |
| FL    | 32  | 35   | 34   | 33  | 25  |
| RR    | 18  | 36   | 39   | 19  | 21  |
| RL    | 13  | 16   | 17   | 4   | 5   |

## Build & Deploy

```bash
cd ESP32/ESP32WiFiConnect
platformio run -t upload
platformio device monitor
```

## Next Steps

- Enable WiFi and implement TCP command interface
- Add OTA (over-the-air) firmware updates
- Implement ROS 2 micro-ROS node for direct Jetson integration
- Add mecanum/omni kinematics (per-wheel PID on ESP32, kinematics on Jetson/ROS 2)
