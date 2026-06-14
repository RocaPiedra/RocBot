# PID Tuner — Web Dashboard, Calibration & Auto-Tuning

A real-time web dashboard for tuning the RocBot motor controller. Built with NiceGUI, featuring live charts, L298N channel imbalance calibration, and pluggable PID auto-tuning via the strategy pattern.

**Location**: `ESP32/ESP32WiFiConnect/tools/rocbot_tuner/`
**Run**: `/usr/bin/python3 -m tools.rocbot_tuner.app` → http://localhost:8080

---

## Architecture

```
tools/rocbot_tuner/
├── app.py                 # NiceGUI web app (3 tabs)
├── calibration.py         # L298N channel calibration
├── models.py              # MotorState, ControllerState dataclasses
├── parser.py              # Serial debug line parser
├── metrics.py             # Step response analysis
├── logger.py              # CSV logging
├── tuning/                # Pluggable tuning strategies
│   ├── __init__.py        # Registry + discovery
│   ├── base.py            # Abstract base class
│   ├── ziegler_nichols.py # Step response method
│   ├── relay.py           # Relay feedback method
│   └── cohen_coon.py      # Dead-time optimized method
├── transport/
│   ├── base.py            # Transport ABC
│   ├── serial.py          # USB serial to ESP32
│   └── ros2.py            # rclpy over micro-ROS agent
└── logs/                  # CSV output
```

### Transport Layer

The tuner supports two communication backends:

| Transport | Protocol | When to Use |
|-----------|----------|-------------|
| `SerialTransport` | USB serial (115200 baud) | Debug firmware (`main_debug.cpp`) — PID tuning, direct PWM |
| `Ros2Transport` | WiFi/UDP via micro-ROS agent | micro-ROS firmware (`main_microros.cpp`) — ROS 2 topic control |

Both implement the `Transport` ABC with `connect()`, `disconnect()`, `send_command()`, and `read_loop()`. Switching between them is a one-line change in the UI.

---

## Dashboard Tab

The default view when the app starts. Shows:

### Motor Status Cards

Per-motor live readouts:

| Field | Description |
|-------|-------------|
| RPM | Current raw RPM |
| F (Filtered) | IIR-filtered RPM |
| PWM | Current motor power (filtered output) |
| Dir | Direction: FWD, REV, or STP |
| Error | Target − measured RPM |

### Charts

Three synchronized ECharts with dark cyberpunk theme:

- **RPM Chart** — Dashed line for target, solid for measured RPM. One colour per motor (FL=cyan, FR=magenta, BL=green, BR=orange)
- **PWM Chart** — Filtered motor power output
- **Error Chart** — Tracking error (target − measured)

500-point rolling window with auto-scaling.

### Step Response Metrics

After running a step test (via the drawer), displays:

- **Peak RPM & time** — Maximum overshoot point
- **Overshoot %** — Percentage above target
- **Rise time** — 10% to 90% of target
- **Settling time** — Time to stay within 2% band
- **Steady-state error** — Final RPM offset

---

## Calibration Tab

### What It Solves

The L298N dual-H-bridge has two independent channels that are **not perfectly matched**. Semiconductor tolerances, PCB trace asymmetry, and thermal effects mean one channel consistently delivers more power than the other at the same PWM duty cycle. In RocBot, the FR channel outputs **~14% more power** than FL.

This tab measures the imbalance and computes `os<scale>` (output scale) values to compensate.

### How It Works

1. **Configure** — Select motors, PWM test levels, and direction
2. **Run** — The sequence executes:
   - Stop motors
   - For each PWM level (e.g. 80, 120, 160, 200):
     - Apply direct PWM (`d<pwm>`)
     - Wait for steady state
     - Record RPM samples for 2 seconds
   - Stop motors
3. **Compute** — For each motor, calculates:
   - `avg_eff = avg(RPM/PWM)` across all levels
   - Reference motor = highest efficiency (strongest channel)
   - `scale_motor = eff_motor / eff_reference`
4. **Apply** — Sends `os<scale>` commands to the ESP32 for each motor

### Results Example

```
**Calibration Results**
Reference motor: `FR`

**FL:**
- Avg efficiency: `0.821 RPM/PWM`
- PWM 80: `65.2` RPM avg (50 samples)
- PWM 120: `98.1` RPM avg (50 samples)
- PWM 160: `131.5` RPM avg (50 samples)

**FR:**
- Avg efficiency: `0.945 RPM/PWM`
- PWM 80: `75.5` RPM avg (50 samples)
- PWM 120: `113.8` RPM avg (50 samples)
- PWM 160: `151.2` RPM avg (50 samples)

**Recommended Output Scales:**
- `FL` → `os0.869`
- `FR` → `os1.000`

**Channel imbalance:** `13.1%`
```

The recommended scales can be applied directly to the ESP32 via the "Apply Scales" button.

---

## Auto-Tuning Tab

### Strategy Pattern

The tuning system uses the **Strategy pattern** — each tuning algorithm is a separate class inheriting from `TuningMethod`. The UI auto-discovers registered methods via the `METHODS` registry dict.

```python
# tuning/__init__.py
METHODS: dict[str, type[TuningMethod]] = {
    "ziegler_nichols": ZieglerNicholsStepResponse,
    "relay": RelayTuning,
    "cohen_coon": CohenCoonTuning,
}
```

To add a new method:
1. Subclass `TuningMethod` in a new file
2. Implement `method_id()`, `method_name()`, `parameters()`, and `run()`
3. Register in `METHODS` — the UI picks it up automatically

### Method 1: Ziegler-Nichols Step Response

**How it works:**

1. Stop motors, capture 0.5s baseline
2. Apply an open-loop PWM step (configurable, default d80)
3. Record the process reaction curve for 4 seconds
4. Find the **inflection point** (maximum slope) and fit a tangent line
5. Extract three process parameters:

| Parameter | How | Meaning |
|-----------|-----|---------|
| K | ΔRPM / ΔPWM | Process gain |
| L | Tangent x-axis intercept | Dead time (s) |
| T | Time from baseline to final value along tangent | Time constant (s) |

6. Apply Ziegler-Nichols tuning rules:

| Type | Kp | Ki | Kd |
|------|----|----|----|
| P | 1/a | — | — |
| PI | 0.9/a | Kp/(3.33·L) | — |
| PID | 1.2/a | Kp/(2·L) | Kp·0.5·L |

Where `a = K·L/T` (normalized gain).

**Best for**: Systems with an S-shaped step response (most DC motors).

### Method 2: Relay Feedback (Åström-Hägglund)

**How it works:**

1. Replace the PID controller with a **relay** (bang-bang) controller
2. The relay switches ON when error > +hysteresis, OFF when error < −hysteresis
3. This causes the system to oscillate at its **ultimate frequency**
4. Measure the oscillation:

| Parameter | How | Meaning |
|-----------|-----|---------|
| a | Average peak-to-peak amplitude | Oscillation amplitude (RPM) |
| Tu | Average period between zero crossings | Ultimate period (s) |
| Ku | 4·d / (π·a) | Ultimate gain |

5. Apply ZN-like rules from Ku and Tu:

| Type | Kp | Ki | Kd |
|------|----|----|----|
| P | 0.5·Ku | — | — |
| PI | 0.45·Ku | Kp/(0.83·Tu) | — |
| PID | 0.60·Ku | Kp/(0.5·Tu) | Kp·0.125·Tu |

**Best for**: Noisy systems, or when open-loop step testing is impractical. More robust than ZN for systems with friction or backlash.

### Method 3: Cohen-Coon

**How it works:**

Uses the same step response data as ZN but applies different formulas optimized for disturbance rejection:

| Parameter | P | PI | PID |
|-----------|----|----|-----|
| Kp | (1/K)·(1+τ/3)/(1+τ) | (0.9/K)·(1+0.92·τ)/(1+τ) | (1.35/K)·(1+0.18·τ)/(1+0.61·τ) |
| Ki | — | Kp / (3.33·L·(1+0.3·τ)/(1+2.2·τ)) | Kp / (2.5·L·(1+0.3·τ)/(1+1.3·τ)) |
| Kd | — | — | Kp·(0.37·L)·(1−0.19·τ)/(1−0.41·τ) |

Where `τ = L/T` (delay-to-time-constant ratio).

**Best for**: Systems where `τ > 0.1` (significant dead time relative to time constant). Generally produces more aggressive gains than ZN.

### Phase Progress

While a tuning method runs, the UI shows a live progress timeline:

```
[OK] Stopping motors
[>] Capturing baseline
[OK] Applying step d80
[>] Collecting reaction curve
[OK] Computing gains
```

Each phase has status: `pending → running → complete → error`.

### Applying Results

After tuning completes, the "Apply Gains" button sends the recommended Kp, Ki, Kd, and output scale to the ESP32 in one click.

---

## Running the Tuner

### Dependencies

```bash
# Install on system Python (not Hermes venv)
pip install nicegui pyserial numpy
```

### Start

```bash
cd ~/personal/projects/RocBot/ESP32/ESP32WiFiConnect
/usr/bin/python3 -m tools.rocbot_tuner.app

# Optional: specify serial port and web port
/usr/bin/python3 -m tools.rocbot_tuner.app --port /dev/ttyUSB0 --port-web 9090
```

Open http://localhost:8080 in a browser.

### First-Time Workflow

1. Connect the ESP32 via USB
2. Flash the debug firmware: `pio run -e debug -t upload`
3. In the tuner: select **Serial (USB)** as transport, click **Connect**
4. Click **PID** mode, set a target RPM, click **Set Target** — verify motors spin
5. Open **Calibration** tab → **Run Calibration** → **Apply Scales**
6. Open **Auto-Tuning** tab → select method → **Run Tuning** → **Apply Gains**
7. Back in **Dashboard**: run a **Step Test** to verify the tuned response

---

## Known Issues

- **Serial permission**: `sudo usermod -aG dialout $USER` then logout/login
- **NiceGUI reload**: If the app crashes on file change, set `reload=False` (already default)
- **ROS2 transport**: Requires `rclpy` (ROS 2 Humble installed). Falls back to serial on plain Ubuntu.
- **Large datasets**: The 500-point rolling window prevents memory issues. Step test buffers use 5000 points for accurate metrics.
