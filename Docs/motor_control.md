# Motor Control Architecture

## Overview

The motor control subsystem has evolved through four iterations, from a simple single-motor PID calibration sketch to a dual-motor object-oriented controller running at 5ms.

## Evolution

### 1. `motor_calibration/` — Initial Calibration

- Single motor, 100ms refresh rate
- Direct PID computation in the main loop
- Encoder via RISING edge interrupt on channel A
- Channel B used for direction detection
- Auto-toggle between 60 and 180 RPM every 5s

### 2. `pid_roc/` — Basic PID

- 50ms refresh rate (doubled speed)
- Motor ID prefix (`&FL;`) for multi-motor serial parsing
- Structured `serialInput()`, `getMotorDirection()`, `RPMtoMpS()` functions
- Stub for sinusoidal input signal

### 3. `pid_roc_filtered/` — Low-Pass Filtering

- Introduced second-order IIR filters on RPM and power
- 10ms refresh rate
- Multi-step input signal: 60→180→240→-60→-180→-240→0

### 4. `MotorControlPID/` — Object-Oriented (Latest)

- 5ms refresh rate (fastest yet)
- C++ classes: `MotorController`, `PIDClass`, `AuxFunctions`
- Dual motor (FL, FR) with independent control
- Timer1 PWM frequency modification for quieter motor operation

## Pin Assignments (MotorControlPID)

| Motor | PWM | ENCA | ENCB | IN1 | IN2 |
|-------|-----|------|------|-----|-----|
| FR    | 14  | 12   | 13   | 27  | 26  |
| FL    | 32  | 35   | 34   | 33  | 25  |

## Control Loop (5ms)

```
loop():
  readSerialInput()          // 'g' = report state, int = set target RPM
  
  for each motor:
    deltaT = micros() - prevT
    
    if deltaT >= 5ms:
      ATOMIC_BLOCK:
        updateRPM()          // Convert pulses → RPM, apply filter
        resetPulses()
      
      controlMotor(target, deltaT)
        PID.CalculateOutput()   // e → PID → raw output
        PID.FilterOutput()      // Absolute → clamp → IIR filter
        SetDirection()          // IN1/IN2 pins
        SetSpeed()              // analogWrite(PWM)
```

## Encoder Reading

- **Type**: Quadrature encoder, single-channel interrupt
- **RISING edge** on channel A triggers interrupt
- **Channel B** level sampled on A's rising edge determines direction
- **CPR**: 1320 counts/revolution (motor shaft)
- **MAXCPR**: 330 counts/revolution (gearbox output, ~4:1 reduction)

## RPM Calculation

```
CurrentRPM = (pulses / max_cpr) * (1000 / deltaTms) * 60
```

Followed by IIR filtering:
```
FilteredRPM = 0.854 * FilteredRPM + 0.0728 * CurrentRPM + 0.0728 * previous_rpm
```

## Motor Direction

L298N driver uses two direction pins (IN1, IN2) with a PWM pin:
- Forward: IN1=HIGH, IN2=LOW
- Reverse: IN1=LOW, IN2=HIGH
- Stop: both LOW

## Signal Generation

The `squaredInputSignal()` function generates test profiles by cycling through target RPM values. In the latest version it toggles between 0 and 120 RPM every 2 seconds.
