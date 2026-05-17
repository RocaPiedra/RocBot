# RocBot Motor Debug Findings

## Test Date: May 2025

---

## Hardware Configuration

**IMPORTANT**: Using ONE L298N driver with two isolated channels for both motors.

| L298N Channel | Motor | Notes |
|---------------|-------|-------|
| Channel A (OUT1/OUT2) | FR | Outputs ~14% more power than Channel B |
| Channel B (OUT3/OUT4) | FL | Slightly weaker |

---

## 🔧 Bug Fixes Applied

### Bug 1: PID Error Calculation (FIXED)
**File**: `src/PIDClass.cpp`

```cpp
// BEFORE (WRONG):
e = rpm - target_rpm;

// AFTER (CORRECT):
e = target_rpm - rpm;
```

**Effect**: Without fix, setting target=60 caused motor to go REVERSE instead of forward.

---

### Bug 2: Encoder Direction Inverted (FIXED)
**File**: `src/MotorController.cpp`

```cpp
// BEFORE (WRONG - default logic):
if(b > 0){
  pulses++;  // ENCB HIGH = forward
}else{
  pulses--;  // ENCB LOW = reverse
}

// AFTER (CORRECT - your encoder is inverted):
if(b == 0){
  pulses++;  // ENCB LOW = forward
}else{
  pulses--;  // ENCB HIGH = reverse
}
```

**Effect**: Without fix, going forward showed negative RPM, PID went crazy trying to correct.

---

## Test Results Summary

### L298N Channel Difference
| Test | FR Interrupts | FL Interrupts | Difference |
|------|---------------|---------------|------------|
| Forward (d100) | ~100,000 | ~80,000 | **14%** |
| Reverse (D100) | ~6,500 | ~2,300 | **65%** |

**Conclusion**: Channel A (FR) is significantly more powerful than Channel B (FL).

---

### Encoder Direction Bug
**Symptom**: RPM showed -400 even with target=0 and direction showing FWD

**Root Cause**: 
- Motor physically going "forward" (IN1=HIGH, IN2=LOW)
- But encoder counting DOWN (ENCB logic inverted)
- RPM = negative when it should be positive

**Fix**: Changed ENCB reading from `if(b > 0)` to `if(b == 0)`

---

### Direct PWM Non-Linearity
**Observation**: d1-d4 = no movement, d20+ starts moving, not proportional

**Likely Cause**: 
- L298N has a voltage drop (~2V)
- At low PWM, voltage too low to overcome friction
- This is normal for PWM-controlled motors

---

## Pin Assignments (Verified)

| Motor | PWM | ENCA | ENCB | IN1 | IN2 |
|-------|-----|------|------|-----|-----|
| FR    | 14  | 22   | 23   | 27  | 26  |
| FL    | 32  | 35   | 34   | 33  | 25  |

## L298N Direction Logic

| IN1 | IN2 | Motor Action |
|-----|-----|--------------|
| HIGH | LOW | Forward ✅ |
| LOW | HIGH | Reverse |
| LOW | LOW | Stop (coast) |
| HIGH | HIGH | Stop (brake) |

---

## Debug Commands

| Command | Action |
|---------|--------|
| `d100` | Direct forward PWM 100 |
| `D100` | Direct reverse PWM 100 |
| `e` | Encoder debug info |
| `p` | Enable PID mode |
| `s` | Stop |
| `g` | Get motor state |
| `60` | Target RPM 60 |
| `-60` | Target RPM -60 |

---

## Next Steps After Fixes

1. Flash updated firmware with both fixes
2. Test: `p` then `60` - should go forward with positive RPM
3. Test: `0` then `-60` - should go reverse with negative RPM
4. Calibrate FR/FL PWM difference if needed