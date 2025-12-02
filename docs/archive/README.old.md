# Flight Controller

A modernized stabilization system for hexacopter drones using PyBoard, designed for educational purposes.

## Overview

This flight controller implements a cascaded PID control system for hexacopter stabilization:
- **Outer Loop (Stabilization)**: Converts angle errors to rate targets
- **Inner Loop (Rate Control)**: Converts rate errors to motor outputs
- **Sensor Fusion**: MPU6050 IMU with DMP (Digital Motion Processor)
- **Motor Control**: 6-motor hexacopter configuration
- **RC Input**: 4-channel receiver (Throttle, Roll, Pitch, Yaw)

## Hardware Requirements

- **Microcontroller**: PyBoard (STM32-based)
- **IMU**: MPU6050 (I2C)
- **ESCs**: 6x brushless motor ESCs (PWM control)
- **RC Receiver**: 4+ channel PWM receiver
- **Motors**: 6x brushless motors (hexacopter configuration)

## Project Structure

```
flight_controller/
├── main.py          # Main flight controller logic and control loops
├── pid.py           # PID controller with anti-windup
├── mpu6050.py       # MPU6050 IMU driver with DMP support
├── esc.py           # ESC motor controller (PWM)
├── rc.py            # RC receiver input handler
├── LICENSE          # Project license
└── README.md        # This file
```

## Features

### Modern Code Architecture
- ✅ **Object-Oriented Design**: Clean class-based structure
- ✅ **Configuration Management**: Centralized constants and settings
- ✅ **Comprehensive Documentation**: Docstrings for all modules and functions
- ✅ **Error Handling**: Robust error handling and validation
- ✅ **Code Organization**: Logical separation of concerns

### Flight Control
- ✅ **Cascaded PID Control**: Dual-loop stabilization (angle + rate)
- ✅ **Integral Anti-Windup**: Prevents integrator saturation
- ✅ **Derivative Filtering**: Low-pass filter for smooth derivative
- ✅ **Motor Mixing**: Hexacopter motor layout with proper mixing
- ✅ **Safety Features**: Automatic disarm on low throttle

### Sensor Processing
- ✅ **DMP Integration**: Hardware-accelerated quaternion calculation
- ✅ **Euler Angles**: Roll, pitch, yaw from quaternions
- ✅ **Gyro Data**: Direct gyroscope rate measurements
- ✅ **FIFO Management**: Efficient buffer handling

## Pin Configuration

### RC Receiver (PWM Input)
| Channel   | Pin | Timer | Function |
|-----------|-----|-------|----------|
| Throttle  | Y8  | 12.2  | Throttle |
| Roll      | Y7  | 12.1  | Roll     |
| Pitch     | Y4  | 4.4   | Pitch    |
| Yaw       | Y3  | 4.3   | Yaw      |

### ESC Outputs (PWM Output)
| Motor | Pin | Timer | Position       |
|-------|-----|-------|----------------|
| 0     | X1  | 5.1   | Front          |
| 1     | X2  | 5.2   | Front Right    |
| 2     | X3  | 5.3   | Back Right     |
| 3     | X6  | 2.1   | Back           |
| 4     | Y9  | 2.3   | Back Left      |
| 5     | Y10 | 2.4   | Front Left     |

### I2C (IMU)
| Device  | Address | Bus | Pins    |
|---------|---------|-----|---------|
| MPU6050 | 0x68    | 1   | X9, X10 |

## Motor Layout (Hexacopter)

View from top (front is up):
```
        0 (Front)
    5       1
    
    4       2
        3 (Back)
```

## PID Tuning

### Rate PIDs (Inner Loop - Gyro Rates)
```python
ROLL_RATE:   P=0.7, I=1.0, Imax=50
PITCH_RATE:  P=0.7, I=1.0, Imax=50
YAW_RATE:    P=2.7, I=1.0, Imax=50
```

### Stabilization PIDs (Outer Loop - Angles)
```python
ROLL_STAB:   P=4.5
PITCH_STAB:  P=4.5
YAW_STAB:    P=10.0
```

### Tuning Tips
1. Start with low gains and increase gradually
2. Tune stabilization P gain first (angle response)
3. Then tune rate P gain (rate response)
4. Add I gain last (only if needed for steady-state error)
5. Adjust Imax to prevent excessive integral buildup

## Usage

### Installation
1. Copy all `.py` files to PyBoard
2. Connect hardware according to pin configuration
3. Calibrate ESCs (standard procedure)
4. Power on and test

### Running
```python
# main.py will run automatically on boot
# Or manually:
import main
main.main()
```

### Configuration
Edit the configuration classes in `main.py`:
- `PIDConfig`: PID tuning parameters
- `RCConfig`: RC channel mapping and limits
- `FlightConfig`: Flight control limits and safety settings

## Safety Features

- **Automatic Disarm**: Motors at minimum throttle when RC throttle < 1200μs
- **Integral Reset**: All integrators reset when disarmed
- **Pulse Validation**: RC inputs validated (950-1950μs)
- **Output Clamping**: All PID outputs clamped to safe limits
- **FIFO Overflow**: Automatic recovery from sensor buffer overflow

## Improvements from Original

### Code Quality
- **Better Structure**: Object-oriented with clear separation of concerns
- **Documentation**: Comprehensive docstrings and comments
- **Constants**: Named configuration instead of magic numbers
- **Readability**: Meaningful variable names and formatting

### Functionality
- **Error Handling**: Robust error handling and validation
- **Configuration**: Easy-to-modify configuration classes
- **Debugging**: Better error messages and state tracking
- **Maintainability**: Modular design for easy updates

### Performance
- **Efficient**: Optimized control loop structure
- **Filtering**: Proper derivative filtering for smooth control
- **Anti-Windup**: Integral clamping prevents saturation

## Credits

Based on excellent work from:
- **Main Loop**: [Owen's Quadcopter Autopilot](http://owenson.me/build-your-own-quadcopter-autopilot/)
- **MPU6050 Driver**: [PyComms by cTn-dev](https://github.com/cTn-dev/PyComms/tree/master/MPU6050)
- **PID Controller**: [ArduPilot](https://github.com/diydrones/ardupilot)
- **RC Input**: [MicroPython Timer Examples](http://wiki.micropython.org/platforms/boards/pyboard/modpyb/Timer-Examples)
- **ESC Control**: [MicroPython Quadruped Robot](https://hackaday.io/project/6877-micropython-quadruped-robot)

## License

See LICENSE file for details.

## Contributing

This is an educational project. Feel free to fork, modify, and improve!

### Possible Enhancements
- [ ] Add telemetry output (UART/USB)
- [ ] Implement flight modes (acro, stabilize, altitude hold)
- [ ] Add GPS for position hold
- [ ] Implement failsafe handling
- [ ] Add battery voltage monitoring
- [ ] Implement blackbox logging
- [ ] Add calibration routines

## Disclaimer

⚠️ **This is experimental flight control software for educational purposes.**

- Always test without propellers first
- Use in a safe, open area
- Follow local regulations
- Not suitable for commercial use
- No warranty or guarantees provided

**Fly safely and responsibly!** 🚁
