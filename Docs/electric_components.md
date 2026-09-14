# Electric Components

This document describes the core electric components of the RocBot robot platform — their specifications, wiring, and how they interact in the system.

## Overview

| Component | Model / Part Number | Quantity | Role |
|-----------|---------------------|----------|------|
| Battery | Zeee 6500 mAh 80C 14.8V 96.2 Wh | 1 | Power supply for motors and logic |
| DC Motor | JGB-520 12V 330 RPM (with encoder) | 4 | Drive motors (one per wheel) |
| Motor Driver | L298N Dual H-Bridge | 2 | PWM speed + direction control |
| Processor | ESP32-WROOM-32D | 1 | Real-time motor control + WiFi |

---

## Battery: Zeee 6500 mAh 80C 14.8V 96.2 Wh

### Specifications

| Parameter | Value |
|-----------|-------|
| Capacity | 6500 mAh (6.5 Ah) |
| Nominal voltage | 14.8V (4S LiPo) |
| Max continuous current | 520 A (80C × 6.5 Ah) |
| Energy | 96.2 Wh |
| Cell chemistry | Lithium Polymer (LiPo) |
| Discharge rate | 80C |
| Charge connector | JST-XH balance tap |
| Main discharge connector | XT60 or XT90 (depending on pack) |

### Why this battery?

The Zeee 6500 mAh 4S LiPo was chosen for its high burst current capability and capacity. The four drive motors can draw significant current during acceleration, and the 80C rating provides a ~520 A peak headroom. The 14.8V nominal voltage matches the 12V motors well (the motors run faster and stronger at 14.8V, and the L298N drivers handle up to ~35V). The 96.2 Wh capacity provides substantial runtime for a robot of this size.

> ⚠️ **LiPo Safety**: Always use a LiPo-compatible charger, balance-charge regularly, and store at ~3.8V per cell (storage voltage). Never discharge below 3.0V per cell.

---

## DC Motors: JGB-520 12V 330 RPM

### Specifications

| Parameter | Value |
|-----------|-------|
| Model | JGB-520 |
| Rated voltage | 12V DC |
| No-load speed | 330 RPM |
| Rated current | ~0.6 A (typical) |
| Stall current | ~2.5 A (estimated) |
| Power | ~7.2 W |
| Encoder | Quadrature (2-channel, A + B) |
| Encoder CPR (raw) | 1320 counts/revolution |
| Encoder CPR (gearbox output) | 330 counts/revolution |
| Gear ratio | ~4:1 (implied from CPR) |
| Wheel diameter | 96 mm |
| Torque | Medium — suitable for 4WD indoor robot |

### Wiring

Each motor has a 6-pin JST connector:

| Pin | Function | Wire color (typical) |
|-----|----------|---------------------|
| M+ | Motor positive | Red |
| M− | Motor negative | Black |
| GND | Encoder ground | Black |
| VCC | Encoder power (+3.3V / +5V) | Red |
| A | Encoder channel A | Green |
| B | Encoder channel B | Yellow |

### Encoder Notes

The encoder produces quadrature pulses. In the firmware, we use **RISING edge on channel A** to count steps, and read the **level of channel B** to determine direction. The raw encoder output is 1320 CPR (counts per revolution), but because the encoder is mounted on the motor shaft before the gearbox, the effective CPR at the wheel is **330 counts/revolution** (matching the 4:1 gear reduction).

> ⚠️ **Direction Inversion**: On the RocBot hardware, the ENCB signal reads LOW when the motor is spinning forward. This was inverted in the `MotorController` code (`if(b == 0)` instead of `if(b > 0)`).

---

## Motor Controllers: L298N Dual H-Bridge

### Specifications

| Parameter | Value |
|-----------|-------|
| Chip | L298N (ST Microelectronics) |
| Channels | 2 independent H-bridges |
| Logic voltage | 5V (from ESP32 or external 5V regulator) |
| Motor voltage | 5V – 35V |
| Continuous current | 2 A per channel |
| Peak current | 3 A per channel |
| PWM control | Yes (ENA / ENB pins) |
| Heat dissipation | Large heatsink required under load |

### Wiring

Each L298N drives **two motors** on its fixed terminal strip
`ENA – IN1 – IN2 – IN3 – IN4 – ENB`: channel A (`ENA/IN1/IN2`,
`OUT1/OUT2`) drives one motor, channel B (`IN3/IN4/ENB`,
`OUT3/OUT4`) drives the other. L298N #1 drives the front motors
(FR on channel A, FL on channel B), L298N #2 drives the rear
motors (RR on channel A, RL on channel B).

L298N #1 (front):

| Terminal | Connection | Notes |
|----------|-----------|-------|
| **VCC** (12V) | Battery positive (+14.8V) | Motor power supply |
| **GND** | Battery ground + ESP32 ground | Common ground reference |
| **5V** | Onboard 5V output (5V jumper ON, verified) | Powers logic — do NOT feed external 5V here, see Jumpers below |
| **ENA** | ESP32 PWM pin GPIO 14 | Channel A speed (FR) |
| **IN1** | ESP32 GPIO 27 | Channel A direction 1 (FR) |
| **IN2** | ESP32 GPIO 26 | Channel A direction 2 (FR) |
| **IN3** | ESP32 GPIO 25 | Channel B direction 1 (FL — ⚠️ crossed, see note) |
| **IN4** | ESP32 GPIO 33 | Channel B direction 2 (FL — ⚠️ crossed, see note) |
| **ENB** | ESP32 PWM pin GPIO 32 | Channel B speed (FL) |
| **OUT1 / OUT2** | FR motor terminals | Channel A motor |
| **OUT3 / OUT4** | FL motor terminals | Channel B motor |

> ℹ️ **FL channel is cross-wired on purpose**: the firmware's
> `In1Pin` (GPIO 33) lands on terminal **IN4**, and `In2Pin`
> (GPIO 25) on terminal **IN3**. The left/right motors are
> mirror-mounted, so FL needs opposite electrical polarity to
> travel the same direction as FR. Verified: `D100` drives both
> front wheels forward together.

L298N #2 (rear):

| Terminal | Connection | Notes |
|----------|-----------|-------|
| **VCC** (12V) | Battery positive (+14.8V) | Motor power supply, thick wire |
| **GND** | Battery ground + ESP32 ground | Common ground reference |
| **5V** | Onboard 5V output (5V jumper ON, verified) | Same as #1 — do NOT feed external 5V here |
| **ENA** | ESP32 PWM pin GPIO 18 | Channel A speed (RR) |
| **IN1** | ESP32 GPIO 19 | Channel A direction 1 (RR) |
| **IN2** | ESP32 GPIO 21 | Channel A direction 2 (RR) |
| **IN3** | ESP32 GPIO 4 | Channel B direction 1 (RL) |
| **IN4** | ESP32 GPIO 5 | Channel B direction 2 (RL, strapping pin — ESP32-driven output, OK) |
| **ENB** | ESP32 PWM pin GPIO 13 | Channel B speed (RL) |
| **OUT1 / OUT2** | RR motor terminals | Channel A motor |
| **OUT3 / OUT4** | RL motor terminals | Channel B motor |

> ℹ️ **RL channel is wired straight** (terminal IN3 = GPIO 4,
> IN4 = GPIO 5, matching firmware order) — unlike FL. Since the
> rear motors are also mirror-mounted, expect a global command
> to spin them in *opposite* travel directions. Verify with
> `D100`: if the rear wheels fight each other, cross RL the
> same way as FL (terminal IN3 = GPIO 5, IN4 = GPIO 4).

> ⚠️ **Forward polarity UNVERIFIED**: OUT wiring follows the
> standard channel layout, but confirm with `d100` (expect all
> wheels forward) and `D100` (all reverse) after flashing. If a
> wheel spins backwards, swap its OUT1/OUT2 (or OUT3/OUT4) wires.

### Jumpers (both L298Ns — as built and verified)

| Jumper | State | Why |
|--------|-------|-----|
| **ENA** | **OFF (removed)** ✅ | ON hard-wires enable HIGH → motor stuck at full speed, PWM ignored |
| **ENB** | **OFF (removed)** ✅ | Same as ENA |
| **5V / VCC-select** | **ON (kept)** ✅ | Onboard regulator powers the logic from VCC. This has worked in testing — but note the 14.8V caveat below |

> ⚠️ **5V jumper ON + 14.8V pack**: the onboard linear regulator
> burns `(14.8 − 5) × I₅ᵥ` as heat, so keep the 5V load light
> (logic + ESP32 only, no servos/sensors strips). **Never connect
> an external 5V supply to the 5V pin while this jumper is on**
> — the two regulators would fight. If you see ESP32 brownouts
> (resets during WiFi TX or 4-motor load) or a scorching
> regulator, switch to the external buck in
> [Power Supply](power_supply.md) (Option 2) and remove the 5V
> jumper.

### Direction Truth Table

| IN1 | IN2 | Channel A behavior |
|-----|-----|-----------------|
| LOW | LOW | Stop (coast) |
| HIGH | LOW | Forward |
| LOW | HIGH | Reverse |
| HIGH | HIGH | Brake (short circuit) |

Channel B behaves identically with `IN3/IN4` in place of `IN1/IN2`.

> 💡 **Known Issue**: The L298N is not a modern driver — it has ~1.5V–2V voltage drop per channel, which means the motors only see ~12V–13V from a 14.8V battery. This results in lower top speed and wasted heat. For future upgrades, consider the **TB6612FNG** or **DRV8833** (MOSFET-based, lower drop, higher efficiency).

---

## Processor: ESP32-WROOM-32D

### Specifications

| Parameter | Value |
|-----------|-------|
| Module | ESP32-WROOM-32D |
| SoC | ESP32-D0WD (Dual-core Xtensa LX6) |
| CPU | 2 × 240 MHz |
| RAM | 520 KB SRAM |
| Flash | 4 MB (external QSPI) |
| WiFi | 802.11 b/g/n 2.4 GHz |
| Bluetooth | BLE 4.2 + Classic |
| GPIO | 34 programmable GPIOs |
| ADC | 12-bit SAR ADC (18 channels) |
| DAC | 2 × 8-bit |
| PWM | 16 channels (LEDC peripheral) |
| Timers | 4 × 64-bit general-purpose |
| UART | 3 × UART |
| SPI | 4 × SPI |
| I2C | 2 × I2C |
| Operating voltage | 3.3V logic |
| Input voltage (VIN) | 5V – 12V via onboard regulator |

### Why ESP32?

The ESP32 was chosen as the main motor controller because it provides:
- **Sufficient GPIOs** for 4 motors × 3 pins (PWM + 2 direction) + 4 encoder interrupts = 20 pins minimum
- **Hardware interrupts** on almost all GPIOs (critical for encoder counting)
- **WiFi built-in** — eliminates the need for a separate wireless module or USB tether
- **Fast CPU** — 240 MHz handles the 5ms PID control loop easily alongside micro-ROS communication
- **PlatformIO support** — robust C++ development with library management

### Pin Assignments (Current — 4-motor omnidirectional)

| Motor | Function | GPIO |
|-------|----------|------|
| **FL** | PWM | 32 |
| **FL** | ENCA (interrupt) | 35 |
| **FL** | ENCB (direction) | 34 |
| **FL** | IN1 | 33 |
| **FL** | IN2 | 25 |
| **FR** | PWM | 14 |
| **FR** | ENCA (interrupt) | 22 |
| **FR** | ENCB (direction) | 23 |
| **FR** | IN1 | 27 |
| **FR** | IN2 | 26 |
| **RR** | PWM | 18 |
| **RR** | ENCA (interrupt) | 36 |
| **RR** | ENCB (direction) | 39 |
| **RR** | IN1 | 19 |
| **RR** | IN2 | 21 |
| **RL** | PWM | 13 |
| **RL** | ENCA (interrupt) | 16 |
| **RL** | ENCB (direction) | 17 |
| **RL** | IN1 | 4 |
| **RL** | IN2 | 5 |

> 📝 **Notes**:
> - `36, 39` are input-only — ideal for encoders.
> - `16, 17` are UART2 pins, free and safe for encoders.
> - `5` is a strapping pin — OK as ESP32-driven output to L298N (high-impedance input).
> - Avoid `1, 3` (USB serial), `0` (BOOT), `6-11` (flash), `12` (strapping).

---

## Power Budget

| Component | Voltage | Current (typical) | Current (max) |
|-----------|---------|-------------------|---------------|
| 4 × Motors | 14.8V | 2.4 A total | ~10 A total (stall) |
| ESP32 | 3.3V | 80 mA | 240 mA (WiFi TX) |
| L298N (logic) | 5V | 20 mA | 20 mA |
| L298N (losses) | — | — | ~2–3 W heat per driver |

**Estimated runtime**:
- Cruising (light load): ~4 hours
- Heavy maneuvering / PID tuning: ~1–2 hours

---

## Wiring Diagram (Conceptual)

```
┌─────────────────────────────────────────────────────────────┐
│                        BATTERY (14.8V)                       │
│                    Zeee 6500 mAh 80C LiPo                    │
└─────────────┬─────────────────────────────┬─────────────────┘
              │                             │
              ▼                             ▼
     ┌──────────────┐              ┌──────────────┐
     │   L298N #1   │              │   L298N #2   │
     │   (FL + FR)  │              │   (RL + RR)  │
     └──────┬───────┘              └──────┬───────┘
            │                             │
     ┌──────┴──────┐              ┌──────┴──────┐
     │  FL / FR    │              │  RL / RR    │
     │ JGB-520     │              │ JGB-520     │
     │ + Encoders  │              │ + Encoders  │
     └──────┬──────┘              └──────┬──────┘
            │                             │
     ┌──────┴─────────────────────────────┴──────┐
     │                 ESP32                     │
     │  FL: 32/35/34/33/25  FR: 14/22/23/27/26   │
     │  RL: 13/16/17/4/5    RR: 18/36/39/19/21   │
     └───────────────────┬───────────────────────┘
                         ▼
     ┌─────────────────────────────────────────────┐
     │         WiFi (2.4 GHz) / micro-ROS          │
     │              ↕ Laptop / Jetson                │
     └─────────────────────────────────────────────┘
```

---

## Future Upgrades

| Current | Recommended Upgrade | Reason |
|---------|---------------------|--------|
| L298N | TB6612FNG or DRV8833 | Lower voltage drop, higher efficiency, less heat |
| ESP32 (single) | ESP32-S3 or dual-ESP32 | More RAM for micro-ROS, better isolation of motor control vs. WiFi stack |
| Battery monitoring | INA219 current/voltage sensor | Real-time power draw monitoring, low-battery safety |
| 5V regulator | LM2596 buck module | See [Power Supply](power_supply.md) for wiring details |

---

## Related Documents

- [Power Supply](power_supply.md) — How to drop 14.8V to 5V and 3.3V safely for your breadboard
- [Hardware Reference](hardware.md) — Pin assignments, wiring tables, I2C connections
- [Motor Control](motor_control.md) — PWM configuration, encoder reading, ISR details
- [ESP32 Port](esp32.md) — PlatformIO setup, WiFi transport, micro-ROS integration
- [PID Controller](pid_controller.md) — PID tuning, IIR filtering, output scaling
