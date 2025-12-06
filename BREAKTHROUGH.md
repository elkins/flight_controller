# Cascaded Controller Breakthrough - Dec 5, 2024

## TL;DR
**The cascaded controller is now flying stably!** Fixed critical motor mixing sign error that was causing all previous crashes.

## The Problem
For weeks, the cascaded controller would immediately flip and crash during takeoff, despite having:
- Correct PID gains (copied from gym-pybullet-drones)
- Proper quaternion math
- Validated physics simulation
- Working manual flight mode

## The Root Cause
**Motor mixing matrix had wrong signs for PyBullet's body frame convention.**

PyBullet uses a different coordinate system than expected. The mixing coefficients needed to be inverted:

```python
# BEFORE (crashed):
self.motor_mix[i, 1] = -np.sin(angles[i])   # Roll
self.motor_mix[i, 2] = np.cos(angles[i])    # Pitch
self.motor_mix[i, 3] = (-1) ** (i + 1)      # Yaw

# AFTER (flies!):
self.motor_mix[i, 1] = np.sin(angles[i])    # Roll - INVERTED
self.motor_mix[i, 2] = -np.cos(angles[i])   # Pitch - INVERTED
self.motor_mix[i, 3] = (-1) ** i            # Yaw - INVERTED
```

## Discovery Process

### 1. Ruled Out Physics Issues
Created `test_passive_stability.py` - applied equal thrust to all motors with NO control.
- **Result**: Drone stayed perfectly level
- **Conclusion**: PyBullet physics is correct, problem is in control

### 2. Isolated Rate Controller
Created `test_rate_controller_tuning.py` - step response test:
- Command +10°/s roll rate
- **Expected**: Drone rolls right (positive)
- **Actual**: Drone rolled LEFT (negative) and diverged
- **Diagnosis**: Control was fighting in the wrong direction!

### 3. Confirmed with Debug Logging
Created `test_rate_debug.py` with detailed PID logging:
```
Setpoint: +10.0°/s (want right)
Actual:   -4.7°/s  (going left!)
Error:    +14.6°/s (correct)
Torque:   +0.000255 (correct)
But rate gets MORE negative → WRONG SIGN
```

### 4. Empirical Fix
Inverted roll torque sign in test → **drone rolled in correct direction!**

Applied same fix to all three axes (roll, pitch, yaw) in cascaded controller.

## Results

### Before Fix
```
t=4.8s: Att=[106.0°, -40.8°, -53.2°]  # Flipped!
t=6.0s: Att=[-174.4°, -6.1°, -106.2°] # Crashed
```

### After Fix
```
t=3.6s  [TAKEOFF]: Att=[0.0°, 0.0°, 0.0°]   # Level
t=10.8s [HOVER]:   Att=[0.0°, 0.0°, 0.0°]   # Stable!
t=18.1s [HOVER]:   Att=[0.0°, 0.0°, 0.0°]   # Perfect
```

**30-second flight test:**
- ✓ Smooth takeoff: 0.05m → 3.97m altitude
- ✓ Attitude: Within ±0.1° throughout entire flight
- ✓ Zero yaw drift
- ✓ Hover thrust estimate: 0.295 (close to expected 0.265)
- ✓ Loop rate: 83 Hz sustained

## Why This Was Hard to Find

1. **Sign errors are subtle**: The math looked correct in isolation
2. **Multiple coordinate frames**: World, body, PyBullet's internal representation
3. **Cascaded complexity**: 4 nested control loops amplified the error
4. **Looked like gain tuning**: Initial attempts focused on adjusting PID gains
5. **Manual control worked**: Simple rate-only control masked the issue

## Key Learnings

### 1. Test Components in Isolation
Don't trust cascaded systems when debugging. Test each loop independently:
- Passive (no control) → validates physics
- Rate only → validates innermost loop
- Attitude → validates next layer
- Then full cascade

### 2. Empirical Testing Beats Theory
When theory says it should work but doesn't → test empirically:
- Step response tests
- Sign inversion experiments
- Direct observation of behavior

### 3. Coordinate Frame Validation is Critical
Never assume coordinate frame conventions. Always validate:
- Test each axis independently
- Verify positive control → positive response
- Check all rotation directions

## Files Changed

**Core Fix:**
- `src/control/cascaded_controller.py` (lines 400-402)

**Diagnostic Tools Added:**
- `examples/test_rate_controller_tuning.py` - Step response testing
- `examples/test_rate_debug.py` - Detailed PID analysis
- `examples/test_passive_stability.py` - Physics validation
- `examples/demo_for_recording.py` - Working demo

**Documentation:**
- `docs/ADAPTIVE_HOVER_THRUST.md` - Hover thrust estimation

## Current Controller State

**Rate Controller Gains (Conservative):**
```python
roll/pitch: kp=0.01, ki=0.002, kd=0.0002
yaw:        kp=0.015, ki=0.002, kd=0.0
```

**Performance:**
- Stable hover: ✓
- Gentle maneuvers: ✓
- Aggressive figure-8: ⚠️ (crashes, needs higher gains)

**Next Steps:**
1. Gradually increase rate gains for better performance
2. Test more aggressive maneuvers
3. Tune velocity controller for faster response
4. Add trajectory tracking

## Commit
```
96ed921 - Fix cascaded controller motor mixing - now flies stably!
```

## The Bottom Line

After weeks of instability, one 3-line change fixed everything:
```diff
- self.motor_mix[i, 1] = -np.sin(angles[i])
+ self.motor_mix[i, 1] = np.sin(angles[i])
```

**The cascaded controller is now production-ready for gentle flight operations.**
