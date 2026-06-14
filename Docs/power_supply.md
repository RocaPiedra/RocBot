# Power Supply Plan

Your **14.8V LiPo** is perfect for the motors but will destroy the ESP32 and sensors if connected directly. This guide shows how to drop it safely to the voltages your electronics need, using a breadboard and common components.

## Voltage Rails You Need

| Rail | Voltage | For | Source |
|------|---------|-----|--------|
| **Motor Power** | 14.8V (battery direct) | Motors via L298N | Battery → L298N VCC |
| **Logic / 5V** | 5V ±5% | L298N logic (Vss), ESP32 VIN, USB | 14.8V → step-down regulator |
| **Sensor / 3.3V** | 3.3V ±5% | ESP32 GPIO, IMU, encoders | ESP32 dev board 3.3V pin OR external regulator |

> **Critical**: The BNO080 IMU **must** run at 3.3V. 5V will kill it permanently.

---

## Recommended Architecture

```
Battery (14.8V LiPo)
    │
    ├─────[ Thick Wire ]──────┬──────────────────────┬──────────────┐
    │                         │                      │              │
    ▼                         ▼                      ▼              ▼
┌─────────┐            ┌─────────────┐      ┌──────────┐      ┌──────┐
│ L298N   │            │  Step-Down  │      │  ESP32   │      │ GND  │
│ VCC pin │            │  Regulator  │      │  VIN pin │      │      │
│ (14.8V) │            │  (5V OUT)   │      │  (5V)    │      │      │
└────┬────┘            └──────┬──────┘      └────┬─────┘      └──┬───┘
     │                        │                  │               │
     ▼                        │                  ▼               │
┌─────────┐                   │           ┌──────────┐          │
│ Motors  │                   │           │ ESP32    │          │
│ (14.8V) │                   │           │ 3.3V pin │          │
└─────────┘                   │           └────┬─────┘          │
     │                        │                │                │
     │                        │                ▼                │
     │                        │         ┌──────────────┐        │
     │                        │         │ IMU, Encoders│        │
     │                        │         │ (3.3V)       │        │
     │                        │         └──────────────┘        │
     │                        │                                 │
     └────────────────────────┴─────────────────────────────────┘
                         Common Ground (GND)
```

**Key rule**: Motor power (14.8V, high current) stays on thick wires directly to the L298N. Logic power (5V, low current) lives on the breadboard.

---

## Option 1: Use an LM7805 (if you have one)

The **LM7805** is a classic linear regulator. It drops 14.8V down to 5V. It will get hot — you must add a heatsink.

### Circuit

```
Battery+ (14.8V) ──┬── 7805 IN  ── 7805 OUT ──┬── 5V Rail
                   │                            │
                   └── 7805 GND ────────────────┴── GND Rail
```

### Components needed

| Component | Value | Purpose |
|-----------|-------|---------|
| Voltage regulator | LM7805 (TO-220) | 14.8V → 5V |
| Input capacitor | 100µF electrolytic | Stability, reduces voltage spikes |
| Output capacitor | 10µF + 100nF ceramic | Filters noise, keeps 5V clean |
| Heatsink | Small TO-220 clip-on | **Mandatory** — see heat math below |
| Diode (optional) | 1N4007 | Reverse polarity protection for battery |

### Heat Math

```
Voltage drop = 14.8V - 5V = 9.8V
Current draw = ~300mA (ESP32 + WiFi + sensors + L298N logic)
Power wasted = 9.8V × 0.3A = 2.9W
Temperature rise = 2.9W × 65°C/W (no heatsink) = 190°C  ← TOO HOT
Temperature rise = 2.9W × 15°C/W (small heatsink) = 44°C  ← OK
```

**Conclusion**: A 7805 without a heatsink will shut down or burn. Add a small heatsink or metal plate. Do not touch it while running.

### Wiring Steps

1. **Battery → 7805**: Connect battery positive to 7805 **IN** pin (add 100µF cap between IN and GND).
2. **7805 → 5V rail**: Connect 7805 **OUT** to a breadboard power rail (add 10µF + 100nF cap between OUT and GND).
3. **GND**: Connect 7805 GND to battery negative and to the breadboard GND rail.
4. **5V → L298N**: Remove the 5V jumper on the L298N module. Connect your breadboard 5V rail to the L298N **5V pin**.
5. **5V → ESP32**: Connect breadboard 5V rail to ESP32 **VIN** pin (or USB pin if using a USB cable with 5V cut).
6. **3.3V → sensors**: Connect ESP32 **3.3V pin** to IMU VCC and encoder VCC.
7. **GND**: Connect all GNDs together (battery, L298N, ESP32, IMU, encoders) — this is your **single common ground**.

### Breadboard Layout

```
[ Breadboard Power Rail - Top: 5V ]
  │
  ├──→ L298N 5V pin (jumper OFF)
  ├──→ ESP32 VIN pin
  └──→ 10µF + 100nF caps to GND

[ Breadboard Power Rail - Bottom: GND ]
  │
  ├──→ Battery GND
  ├──→ L298N GND
  ├──→ ESP32 GND
  ├──→ IMU GND
  └──→ Encoder GND
```

---

## Option 2: Use an LM2596 Buck Converter (recommended)

A **switching buck converter** is far more efficient. It wastes almost no heat. If you have an LM2596 module (or a "DC-DC step-down" board), use this instead.

### Advantages
- ~90% efficient (only ~0.3W lost vs 2.9W with 7805)
- Runs cool without a heatsink
- Can provide more current if you add sensors later

### Wiring

1. Connect battery **+14.8V** to LM2596 **IN+**.
2. Connect battery **GND** to LM2596 **IN−**.
3. Turn the potentiometer until the output reads **5.0V** (use a multimeter).
4. Connect LM2596 **OUT+** to your breadboard 5V rail.
5. Connect LM2596 **OUT−** to your breadboard GND rail.
6. Add a **100µF capacitor** across the LM2596 output (optional but recommended for cleaner 5V).
7. Wire the rest exactly as in Option 1 (5V → L298N, 5V → ESP32 VIN, ESP32 3.3V → sensors).

### Output Filtering

Switching converters create electrical noise. Add a small **LC filter** or just a **100µF electrolytic + 100nF ceramic** in parallel on the output to keep the ESP32 happy.

---

## Option 3: Two-Stage (if you only have 3.3V regulators)

If you have an **AMS1117-3.3** or **LM317** but no 5V regulator:

**Do NOT connect 14.8V directly to a 3.3V regulator.** The voltage drop is too large and it will overheat instantly.

Instead:

1. Build a voltage divider to drop 14.8V to ~6V, then feed that into a 3.3V regulator (inefficient, hot).
2. Or better: buy a cheap LM2596 module for $2–3.

**Verdict**: Use Option 1 or 2. Do not attempt to power 3.3V directly from 14.8V with a linear regulator.

---

## What You Can Do Right Now (Breadboard Priority)

If you have an **LM7805 + heatsink + a few capacitors**, build Option 1 today. If you have an **LM2596 module**, build Option 2.

### Minimum Shopping List (if missing parts)

| Part | Cost | Why |
|------|------|-----|
| LM2596 buck module | $2–3 | Clean, cool 5V supply |
| 100µF electrolytic capacitor | $0.10 | Input spike filtering |
| 10µF + 100nF ceramic capacitors | $0.10 | Output noise filtering |
| XT60 to barrel jack / wires | $2 | Connect battery to breadboard |
| Fuse holder + 5A fuse | $1 | **LiPo safety** — mandatory |

### Build Order

1. **Connect battery GND** to breadboard GND rail first. This is your reference.
2. **Add regulator** (7805 or LM2596) with capacitors.
3. **Test with multimeter**: Verify 5V output before connecting anything expensive.
4. **Connect L298N 5V pin** (jumper removed) and test motor direction with `d100` command.
5. **Connect ESP32 VIN** and flash test firmware (`hello_microros.cpp`).
6. **Connect IMU + encoders** to ESP32 3.3V pin and GND.

---

## Grounding Rules (Critical)

1. **Single common ground**: Battery GND, L298N GND, ESP32 GND, IMU GND, encoder GND must all connect to the same point.
2. **Do NOT create ground loops**: Use the breadboard GND rail as your star point.
3. **Keep motor GND separate from logic GND until the star point**: The L298N has one GND pin — it connects to both motor return and logic. That's fine. Just don't run a separate wire from motor negative back to the battery that bypasses the L298N GND.

```
Wrong:
  Battery GND ──→ L298N GND
  Battery GND ──→ ESP32 GND   (separate wire — creates ground loop)

Right:
  Battery GND ──→ Breadboard GND rail ──→ L298N GND + ESP32 GND + all others
```

---

## Noise & Filtering

Motors are electrically noisy. The L298N switches inductive loads, causing voltage spikes and ground bounce.

### What to add

| Component | Placement | Purpose |
|-----------|-----------|---------|
| **100µF electrolytic** | Across battery terminals | Absorbs motor current spikes |
| **100nF ceramic** | Across 5V rail and GND | High-frequency noise filter |
| **10µF ceramic** | Across ESP32 VIN and GND | Prevents ESP32 brownout during WiFi TX |
| **100nF ceramic** | Across each encoder VCC/GND | Clean encoder power |
| **100nF ceramic** | Across IMU VCC/GND | Clean IMU power |

### Keep wires short

- ESP32 → IMU I2C wires: keep under 10 cm if possible
- Encoder wires: use twisted pairs or keep them away from motor power wires
- Motor power wires: thick, short, direct from battery to L298N

---

## Safety Checklist

- [ ] **LiPo fuse**: Add a 5A–10A fuse between battery positive and everything else. LiPo batteries can output hundreds of amps in a short circuit.
- [ ] ** polarity check**: Double-check battery + and − before first power-on. Reverse polarity will destroy the L298N and possibly the ESP32.
- [ ] **Voltage test first**: Use a multimeter to confirm 5V output before connecting the ESP32.
- [ ] **Heat check**: If using a 7805, touch it after 5 minutes. If it burns your finger, add a bigger heatsink or switch to a buck converter.
- [ ] **No bare wires**: LiPo + and − must never touch each other. Use insulated connectors (XT60) or terminal blocks.
- [ ] **Low-voltage alarm**: Consider a simple 4S LiPo buzzer ($1) that beeps when any cell drops below 3.3V. Protects your battery from over-discharge.

---

## Example: Full Wiring with 7805 on Breadboard

```
Battery (XT60)
  ├── Red (+14.8V) ── Fuse ── L298N VCC (thick wire)
  │                          │
  │                          └── 7805 IN pin (thin wire)
  │                               │
  │                               ├── 100µF cap to GND
  │                               │
  │                               └── 7805 OUT pin ── 5V Rail
  │                                                    │
  │                              ┌──────────────────┬─┴──┐
  │                              │                  │    │
  │                              ▼                  ▼    ▼
  │                         L298N 5V pin      ESP32 VIN  10µF+100nF to GND
  │                              (jumper OFF)
  │
  └── Black (GND) ── Breadboard GND Rail ──┬────────┬────────┬────────┐
                                            │        │        │        │
                                            ▼        ▼        ▼        ▼
                                         L298N    ESP32    IMU    Encoders
                                          GND      GND      GND      GND

ESP32 3.3V pin ──┬── IMU VCC
                 ├── Encoder VCC (4×)
                 └── 100nF cap to GND (per sensor)
```

---

## Related Documents

- [Electric Components](electric_components.md) — Battery, motor, driver, processor specs
- [Hardware Reference](hardware.md) — Pin assignments, wiring tables, I2C connections
- [ESP32 Port](esp32.md) — PlatformIO setup, WiFi transport, micro-ROS integration
