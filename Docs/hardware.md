# Hardware Reference

## Parts List

| Component | Model | Quantity | Purpose |
|-----------|-------|----------|---------|
| Arduino Uno | AVR | 1 | Early development, I2C experiments |
| Arduino Mega 2560 Pro Mini | AVR | 1 | IMU + motor control (current) |
| ESP32 | ESP-WROOM-32 | 1 | WiFi-enabled motor control |
| Motor Driver | L298N | 2 | Dual H-bridge, drives 2 motors each |
| DC Motor w/ Encoder | N/A | 4 | Drive motors with quadrature encoders |
| IMU | GY-BNO080 (BNO080) | 1 | 9-DOF absolute orientation sensor |

## Motor Specifications

From `Motors/MotorParameters.xml`:

| Parameter | Value |
|-----------|-------|
| CPR (encoder) | 1320 counts/revolution |
| Max angular speed | 330 RPM (no load) |
| Rated voltage | 12V |
| Power | 7.2W |
| Wheel diameter | 96mm |
| Effective CPR (gearbox) | 330 (~4:1 reduction inferred) |

## L298N Motor Driver Connections

Each L298N drives two motors on its fixed terminal strip
`ENA – IN1 – IN2 – IN3 – IN4 – ENB`:

| L298N Terminal | Connection |
|----------------|-----------|
| VCC (12V) | Motor power supply (14.8V battery direct) |
| GND | Shared ground with ESP32 |
| 5V | Onboard regulator output (5V jumper ON) — powers logic, do NOT backfeed external 5V |
| ENA | PWM pin (speed control, channel A motor) |
| IN1 | Direction pin 1 (channel A motor) |
| IN2 | Direction pin 2 (channel A motor) |
| IN3 | Direction pin 1 (channel B motor) |
| IN4 | Direction pin 2 (channel B motor) |
| ENB | PWM pin (speed control, channel B motor) |
| OUT1/OUT2 | Channel A motor terminals |
| OUT3/OUT4 | Channel B motor terminals |

### RocBot Channel Mapping (verified against wiring)

| Driver | Channel | Motor | ENA/ENB (PWM) | IN1/IN3 | IN2/IN4 | OUT |
|--------|---------|-------|---------------|---------|---------|-----|
| L298N #1 (front) | A | FR | ENA = 14 | IN1 = 27 | IN2 = 26 | OUT1/OUT2 → FR |
| L298N #1 (front) | B | FL | ENB = 32 | IN3 = 25 ⚠️ | IN4 = 33 ⚠️ | OUT3/OUT4 → FL |
| L298N #2 (rear) | A | RR | ENA = 18 | IN1 = 19 | IN2 = 21 | OUT1/OUT2 → RR |
| L298N #2 (rear) | B | RL | ENB = 13 | IN3 = 4 | IN4 = 5 | OUT3/OUT4 → RL |

⚠️ FL is **cross-wired** (firmware `In1`=33 → terminal IN4,
`In2`=25 → terminal IN3) to compensate the mirror-mounted
motors. RL is wired **straight**, pending test.

> ✅ **Front verified**: `D100` drives both front wheels forward
> together. ❌ **Rear unverified**: `D100` may spin the rear
> wheels in opposite travel directions (same mirror reason) —
> if so, cross RL like FL (IN3 = 5, IN4 = 4).

### L298N Jumpers (as built and verified)

| Jumper | State | Why |
|--------|-------|-----|
| ENA | **OFF (removed)** ✅ | ON forces full speed, PWM ignored |
| ENB | **OFF (removed)** ✅ | Same as ENA |
| 5V | **ON (kept)** ✅ | Onboard regulator powers logic from VCC — proven in testing |

> ⚠️ With the 14.8V pack the onboard regulator runs hot
> (`(14.8 − 5) × I₅ᵥ` wasted as heat): keep the 5V load to
> logic + ESP32 only, and never connect an external 5V supply
> to the 5V pin while this jumper is on. On brownouts under
> WiFi/4-motor load, move to the external buck in
> [Power Supply](power_supply.md) and remove the 5V jumper.

Full per-terminal wiring with ESP32 GPIOs: see [Electric Components](electric_components.md).

### Direction Logic

| IN1 | IN2 | Motor A |
|-----|-----|---------|
| LOW | LOW | Stop |
| HIGH | LOW | Forward |
| LOW | HIGH | Reverse |
| HIGH | HIGH | Brake |

## IMU Wiring

| IMU (GY-BNO080) | Arduino Mega 2560 Pro Mini |
|-----------------|---------------------------|
| SCL | D20 (SCL) |
| SDA | D21 (SDA) |
| VCC | 3.3V |
| GND | GND |

I2C address: **0x4B** (75 decimal)

## Pin Assignments (ESP32 — 4-motor omnidirectional)

| Motor | PWM | ENCA | ENCB | IN1 | IN2 |
|-------|-----|------|------|-----|-----|
| FR | 14 | 22 | 23 | 27 | 26 |
| FL | 32 | 35 | 34 | 33 | 25 |
| RR | 18 | 36 | 39 | 19 | 21 |
| RL | 13 | 16 | 17 | 4 | 5 |

L298N #1 drives FL+FR, L298N #2 drives RL+RR. See [Electric Components](electric_components.md) for full wiring.

## PWM Configuration (AVR)

The AVR version configures Timer1 (`TCCR1B`) for a different PWM frequency to reduce audible motor whine. On ESP32 this is handled automatically by the LEDC peripheral.

## Encoder Connection

Each motor encoder has two channels (A and B):
- **Channel A** → ENCA pin (interrupt-enabled, RISING edge)
- **Channel B** → ENCB pin (direction detection, level read on interrupt)

## Reference Diagrams

Hardware reference images are in `Diagrams/`:
- `arduino-nano-pinout.png`
- `ESP32-pinout-diagram-1-1024x737.avif`
- `L298N_conexiones.jpg`
- `L298N-Motor-Driver-Module-Pinout.png`
- `MotorWithEncoder.png`
