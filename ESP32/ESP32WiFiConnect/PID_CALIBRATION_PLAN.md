# RocBot PID Calibration Plan

## Current Problem

The PID controller oscillates wildly:
- Target: 50 RPM → Actual oscillates 0 to 290+ RPM
- PWM always at max (255) - no speed control
- Direction flips constantly
- Noisy, jittery response

**Root cause**: Current gains (Kp=0.8, Ki=1.0, Kd=0.1) are way too aggressive.

---

## Recommended Tuning Approach

### Phase 1: Manual Tuning (Start Here)

Try these conservative gains first:

| Test | Kp | Ki | Kd | Notes |
|------|-----|-----|-----|-------|
| 1 | 0.1 | 0.0 | 0.0 | P-only, should be stable |
| 2 | 0.2 | 0.0 | 0.0 | Higher P |
| 3 | 0.1 | 0.05 | 0.0 | Add some I |
| 4 | 0.15 | 0.05 | 0.01 | Add some D |

**Test command:**
```bash
# In serial monitor:
kp0.1    # Set Kp
ki0      # Set Ki
kd0      # Set Kd
p        # Enable PID
30       # Target 30 RPM
```

### Phase 2: Step Response Testing

Use the Python tuner to run systematic step tests:

```bash
python pid_tuner.py --port /dev/ttyUSB0 --motor FL --test step --target 30
```

This records:
- Rise time
- Overshoot percentage
- Settling time
- Steady-state error

### Phase 3: Auto-Tuning

Implement Ziegler-Nichols or relay auto-tuning:

1. **Ziegler-Nichols**:
   - Increase Kp until system oscillates continuously
   - Record that Kp (Ku) and oscillation period (Tu)
   - Calculate PID: Kp=0.6Ku, Ki=2Ku/Tu, Kd=Ku*Tu/8

2. **Relay (Åström)**:
   - Use on-off control to create oscillation
   - Measure amplitude and period
   - Calculate PID from those values

### Phase 4: Advanced Optimization

If basic tuning doesn't work well:
- **Gain scheduling**: Different PID for different speed ranges
- **Cascaded PID**: Inner loop (torque) + outer loop (velocity)
- **Feedforward**: Add voltage based on expected dynamics
- **LQR**: State-space optimal control

---

## Python Tuner Usage

### Quick Test
```bash
python pid_tuner.py --port /dev/ttyUSB0
```

### Step Response
```bash
python pid_tuner.py --test step --target 30
```

### Parameter Sweep
```bash
python pid_tuner.py --test sweep
```

### Output
- Real-time plot of Target vs Actual RPM
- Metrics: overshoot, settling time, rise time, steady-state error
- CSV log saved to `logs/` directory

---

## ESP32 Tuning Commands

| Command | Action |
|---------|--------|
| `kp0.5` | Set Kp to 0.5 |
| `ki0.1` | Set Ki to 0.1 |
| `kd0.05` | Set Kd to 0.05 |
| `kp` (no value) | Show current Kp |
| `p` | Enable PID mode |
| `s` | Stop |
| `30` | Set target to 30 RPM |
| `step30` | Run step test to 30 RPM |

---

## Expected Results After Tuning

Good PID should have:
- **< 10% overshoot** - shouldn't shoot way past target
- **< 2s settling** - stabilizes quickly
- **< 5 RPM steady-state error** - gets close to target
- **Smooth response** - no jittering

---

## Log Files

Python tuner saves:
- `logs/pid_tuning_YYYYMMDD_HHMMSS.csv`
- Contains: time, target, rpm, error, pwm

Use these for offline analysis and comparing different tuning runs.

---

## Next Steps

1. Try conservative gains first (Phase 1)
2. If that works, run systematic step tests (Phase 2)
3. Implement auto-tuning if needed (Phase 3)
4. Consider advanced algorithms for fine-tuning (Phase 4)