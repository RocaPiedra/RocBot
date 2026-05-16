# PID Controller

## Overview

A digital PID controller with integrated low-pass filtering implemented as a reusable C++ class (`PIDClass`).

## PID Gains (Calibrated)

| Gain | Value | Notes |
|------|-------|-------|
| Kp   | 0.8   | Proportional — stable, no overshoot |
| Ki   | 1.0   | Integral — eliminates steady-state error |
| Kd   | 0.1   | Derivative — minimal damping needed |

**Calibration date**: January 22nd  
**Character**: Slow but stable, little jitter, no offset error.

## Algorithm

### `CalculateOutput(measured, target, deltaTs)`

```
e = measured - target              // error
dedt = (e - e_prev) / deltaTs      // derivative
eint += e * deltaTs                 // integral (accumulated)

e_prev = e

output = Kp*e + Kd*dedt + Ki*eint

return output                       // raw PID output
```

### Integral Windup

Integral windup is controlled by the control loop refresh rate (5ms). If the actuator saturates, the integral term continues accumulating. This is a known area for improvement — a clamping or anti-windup scheme should be added.

## Low-Pass Filtering

A second-order IIR filter is applied independently to:
1. **RPM measurement** (in `MotorController::updateRPM()`)
2. **Motor power output** (in `PIDClass::FilterOutput()`)

### Filter Coefficients

```
y[n] = 0.854 * y[n-1] + 0.0728 * x[n] + 0.0728 * x[n-1]
```

Coefficients sum to ~0.9996 (approximately unity DC gain).  
This corresponds to a cutoff frequency of approximately 10-15 Hz at a 5ms sample rate.

### `FilterOutput(rawOutput)`

```
pwr = abs(rawOutput)                  // absolute value
pwr = constrain(pwr, 0, 255)          // clamp to PWM range

pwr_filt = 0.854 * pwr_filt
         + 0.0728 * pwr
         + 0.0728 * pwr_prev

pwr_prev = pwr

return pwr_filt                        // filtered PWM value
```

The filtered power (`pwr_filt`) is what actually drives the motor via `analogWrite()`.

## Class Interface

```
PIDClass(kp, ki, kd)

  CalculateOutput(measured, target, deltaTs) → float
  FilterOutput(output)                     → float
  GetPower()                               → float  (raw)
  GetPowerFiltered()                       → float  (filtered)
```

## Performance Notes

- Filtering reduces jitter significantly but adds phase lag
- The trade-off is acceptable: stable speed control at the cost of slower response to setpoint changes
- Future improvement: implement conditional integration (anti-windup) to improve step response
