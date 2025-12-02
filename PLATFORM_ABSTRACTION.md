# Platform Abstraction Guide

## Overview

The flight controller now includes a **Hardware Abstraction Layer (HAL)** that makes it easy to port to different hardware platforms. The core flight control logic remains the same across all platforms.

## Architecture

```
┌─────────────────────────────────────────┐
│   Flight Controller Logic (main.py)    │
│   - PID controllers                     │
│   - Control loops                       │
│   - Motor mixing                        │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│   Hardware Abstraction Layer (hal.py)  │
│   - Abstract interfaces                 │
│   - Platform detection                  │
│   - Factory methods                     │
└──────────────┬──────────────────────────┘
               │
    ┌──────────┴──────────┬──────────────────┐
    │                     │                  │
┌───▼────────┐   ┌───────▼──────┐   ┌──────▼────────┐
│ PyBoard    │   │ Raspberry Pi │   │ Arduino/ESP32 │
│ (hal_      │   │ (hal_        │   │ (hal_         │
│  pyboard)  │   │  raspberrypi)│   │  arduino)     │
└────────────┘   └──────────────┘   └───────────────┘
```

## Current Status

### ✅ Implemented
- **HAL Interface** (`hal.py`) - Abstract base classes
- **PyBoard Implementation** (`hal_pyboard.py`) - Current working platform
- **Platform Config** (`platform_config.py`) - Pin mappings for all platforms

### 🚧 To Implement
- **Raspberry Pi** (`hal_raspberrypi.py`) - Using RPi.GPIO or pigpio
- **Arduino/Teensy** (`hal_arduino.py`) - Via MicroPython or C++ bridge
- **ESP32** (`hal_esp32.py`) - Using MicroPython or ESP-IDF

## Adding a New Platform

### Step 1: Create HAL Implementation

Create `hal_yourplatform.py`:

```python
from hal import HALPlatform, HALTimer, HALPWMChannel, etc.

class YourPlatformPWMChannel(HALPWMChannel):
    def set_pulse_width(self, width_us: int):
        # Your platform-specific PWM code
        pass

class YourPlatformPlatform(HALPlatform):
    def get_timer(self, timer_id: int):
        # Return platform-specific timer
        pass
    
    @property
    def platform_name(self) -> str:
        return "YourPlatform"
```

### Step 2: Update Platform Config

Add configuration to `platform_config.py`:

```python
YOUR_PLATFORM_CONFIG = {
    'platform': 'your_platform',
    'esc_pins': {
        0: {'pin': 'GPIO1', ...},
        # ... etc
    },
    'rc_pins': { ... },
    'i2c': { ... }
}
```

### Step 3: Test

The flight controller will auto-detect your platform or you can manually set it:

```python
from hal import set_platform
from hal_yourplatform import YourPlatformPlatform

set_platform(YourPlatformPlatform())
```

## Platform-Specific Examples

### Raspberry Pi (Future)

**Key Differences:**
- Use `RPi.GPIO` or `pigpio` for GPIO
- Software PWM or hardware PWM via `pigpio`
- I2C via `smbus2` library
- Python threading for RC pulse capture

**Challenges:**
- Linux is not real-time (use real-time kernel or dedicated cores)
- GPIO interrupt latency higher than microcontroller
- Requires root access for GPIO

### Arduino/Teensy (Future)

**Key Differences:**
- C++ code with Python wrapper or MicroPython port
- Hardware timers for PWM and input capture
- Hardware interrupts for RC pulse capture
- Native I2C support

**Challenges:**
- Limited RAM on smaller Arduinos
- Teensy 4.x recommended for performance
- May need to run MicroPython or create Python bridge

### ESP32 (Future)

**Key Differences:**
- MicroPython or ESP-IDF
- LEDC for PWM output
- RMT or PCNT for RC input capture
- Dual-core can dedicate one to flight control

**Challenges:**
- WiFi can interfere with timing if enabled
- Need to tune FreeRTOS task priorities
- Some I2C libraries have quirks

## Migration Strategy

### For Existing Code

To migrate existing flight controller code:

1. **No changes needed to:**
   - `pid.py` - Pure Python, platform-independent
   - Core control logic in `main.py`

2. **Minimal changes to:**
   - Import HAL instead of `pyb` directly
   - Use HAL factory functions

3. **Example migration:**

**Before (PyBoard-specific):**
```python
from pyb import Timer, Pin
timer = Timer(5, prescaler=83, period=19999)
channel = timer.channel(1, Timer.PWM, pin=Pin('X1'))
```

**After (Platform-independent):**
```python
from hal import get_platform
platform = get_platform()
timer = platform.get_timer(5)
pwm = timer.create_pwm_channel('X1', frequency=50)
pwm.set_pulse_width(1500)
```

## Benefits

### ✅ Platform Independence
- Core logic works on any platform
- Easy to add new hardware support
- Test on PC, deploy to embedded

### ✅ Maintainability
- Single codebase for all platforms
- Platform-specific code isolated
- Clear interfaces between layers

### ✅ Testing
- Can mock HAL for unit tests
- Test on different platforms easily
- Verify logic before hardware

### ✅ Future-Proof
- Easy to support new boards
- Adapt to hardware changes
- Upgrade platforms without rewrite

## Performance Considerations

### Real-Time Requirements
- **Control loop**: 200 Hz (5ms period)
- **RC input**: <1ms latency
- **PWM output**: <100μs jitter

### Platform Capabilities
| Platform      | RT Capable | Loop Rate | Notes |
|---------------|------------|-----------|-------|
| PyBoard       | ✅ Yes     | 200+ Hz   | STM32, true RTOS |
| Teensy 4.x    | ✅ Yes     | 500+ Hz   | Cortex-M7 @ 600MHz |
| ESP32         | ⚠️ Maybe   | 200 Hz    | Needs tuning |
| Raspberry Pi  | ❌ No      | 100-200Hz | Use RT kernel |
| Arduino Mega  | ⚠️ Maybe   | 100 Hz    | Limited resources |

## Next Steps

1. **Test HAL with current PyBoard** ✅ Done
2. **Implement Raspberry Pi HAL** 🚧 Next
3. **Implement ESP32 HAL** 📅 Planned
4. **Create Arduino/Teensy HAL** 📅 Planned
5. **Performance benchmarks** 📅 Planned

## Example: Raspberry Pi Implementation Outline

```python
# hal_raspberrypi.py
import RPi.GPIO as GPIO
import pigpio
from smbus2 import SMBus

class RaspberryPiPWMChannel(HALPWMChannel):
    def __init__(self, pin, frequency):
        self.pi = pigpio.pi()
        self.pin = pin
        self.pi.set_PWM_frequency(pin, frequency)
    
    def set_pulse_width(self, width_us: int):
        self.pi.set_servo_pulsewidth(self.pin, width_us)

# ... rest of implementation
```

## Conclusion

The HAL design allows the flight controller to be **truly portable** across different hardware platforms while maintaining a clean, maintainable codebase. The initial PyBoard implementation serves as the reference, and new platforms can be added by implementing the HAL interfaces.

**Ready to fly on any platform!** 🚁
