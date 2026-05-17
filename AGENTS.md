# RocBot Agent Guidance

**Important**: The code in `@Motors/` was only for initial development. The current active project is in `@ESP32/ESP32WiFiConnect/`.

## Active Project: ESP32 Motor Controller

**Build system**: PlatformIO (not Arduino IDE)

```bash
cd ESP32/ESP32WiFiConnect
pio run -t upload          # Build and flash (uses main.cpp)
pio run -e debug -t upload # Build and flash debug version
pio device monitor         # Serial monitor (115200 baud)
```

## Key Files

- `ESP32/ESP32WiFiConnect/src/main.cpp` — Main motor control loop
- `ESP32/ESP32WiFiConnect/src/main_debug.cpp` — Debug version with tuning commands
- `ESP32/ESP32WiFiConnect/pid_tuner.py` — Python PID auto-tuner
- `ESP32/ESP32WiFiConnect/include/MotorController.hpp` — Motor class
- `ESP32/ESP32WiFiConnect/include/PIDClass.hpp` — PID controller
- `ESP32/ESP32WiFiConnect/include/ssid.hpp` — WiFi credentials (gitignored)

## Motor Pin Assignments (ESP32)

| Motor | PWM | ENCA | ENCB | IN1 | IN2 |
|-------|-----|------|------|-----|-----|
| FR    | 14  | 22   | 23   | 27  | 26  |
| FL    | 32  | 35   | 34   | 33  | 25  |

## Hardware Configuration

- **Motor Driver**: L298N with dual isolated channels (ONE driver for both motors)
- **Encoder Direction**: INVERTED - ENCB LOW = forward, ENCB HIGH = reverse (fixed in code)

## Known Issues

1. **PID tuning required** - Current gains (Kp=0.8, Ki=1.0, Kd=0.1) are too aggressive
2. **FR L298N channel outputs ~14% more power** than FL channel
3. **Encoder direction is inverted** - fixed in MotorController.cpp

## Serial Protocol (Standard Mode)

- Send `g` → receives motor state
- Send integer (e.g. `60`) → sets target RPM
- Response format: `&FL;target:60;rpm:123.4;rpm_filt:122.1;pwr:150;pwr_filt:148.2`

## Code Constants

- MAXCPR: 330
- MAXRPM: 330
- REFRESHRATE: 5ms
- IIR filter: y[n] = 0.854*y[n-1] + 0.0728*x[n] + 0.0728*x[n-1]

## Debug Commands (main_debug.cpp)

| Command | Action |
|---------|--------|
| `d<0-255>` | Direct forward PWM test |
| `D<0-255>` | Direct reverse PWM test |
| `e` | Encoder debug info |
| `p` | Enable PID mode |
| `s` | Stop motors |
| `g` | Get motor state + PID params |
| `<num>` | Set target RPM (e.g., 60 or -60) |

## PID Tuning Commands (main_debug.cpp)

| Command | Action |
|---------|--------|
| `kp<value>` | Set Kp (e.g., kp0.1, kp1.5) |
| `ki<value>` | Set Ki (e.g., ki0.05) |
| `kd<value>` | Set Kd (e.g., kd0.01) |
| `step<value>` | Run step test to target |
| `step30` | Run step test to 30 RPM |

## Python PID Tuner

```bash
# Install dependencies
pip install pyserial matplotlib numpy scipy

# Run tuner
cd ESP32/ESP32WiFiConnect
python pid_tuner.py --port /dev/ttyUSB0 --motor FL --test step --target 30
```

## Bug Fixes Applied

1. **PID error calculation**: Changed from `e = rpm - target` to `e = target - rpm`
2. **Encoder direction**: Changed from `if(b > 0)` to `if(b == 0)` for ENCB reading

## Initial Tuning (Try These First)

Conservative gains to start with:
```bash
kp0.1
ki0
kd0
p
30
```

If too slow, try:
```bash
kp0.3
ki0.05
kd0.01
p
30
```