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

Each L298N drives two motors:

| L298N Terminal | Connection |
|----------------|-----------|
| VCC (12V) | Motor power supply (12V) |
| GND | Shared ground with Arduino/ESP32 |
| 5V | Optional — output to power logic (not recommended) |
| ENA | PWM pin (speed control, motor A) |
| IN1 | Direction pin 1 (motor A) |
| IN2 | Direction pin 2 (motor A) |
| ENB | PWM pin (speed control, motor B) |
| IN3 | Direction pin 1 (motor B) |
| IN4 | Direction pin 2 (motor B) |
| OUT1/OUT2 | Motor A terminals |
| OUT3/OUT4 | Motor B terminals |

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

## Pin Assignments (MotorControlPID)

| Motor | PWM | ENCA | ENCB | IN1 | IN2 |
|-------|-----|------|------|-----|-----|
| FR | 14 | 12 | 13 | 27 | 26 |
| FL | 32 | 35 | 34 | 33 | 25 |

## PWM Configuration (AVR)

The AVR version configures Timer1 (`TCCR1B`) for a different PWM frequency to reduce audible motor whine. On ESP32 this is handled automatically by the LEDC peripheral.

## Encoder Connection

Each motor encoder has two channels (A and B):
- **Channel A** → ENCA pin (interrupt-enabled, RISING edge)
- **Channel B** → ENCB pin (direction detection, level read on interrupt)

## Reference Diagrams

Hardware reference images are in `Diagrams/`:
- `arduino-nano-pinout.png`
- `L298N_conexiones.jpg`
- `MotorWithEncoder.png`
