# Quick Start Guide

## 🚀 Getting Started

### For Hardware (PyBoard)

1. **Upload Files to PyBoard**
   ```bash
   # Copy core files to PyBoard (via USB mass storage or rshell)
   cp main.py pid.py esc.py rc.py mpu6050.py /path/to/PYBFLASH/
   ```

2. **Connect Hardware**
   - MPU6050 → I2C (pins X9, X10)
   - RC Receiver → See pin table in README.md
   - ESCs → See pin table in README.md

3. **Power On**
   - PyBoard will auto-run main.py
   - Ensure throttle is at minimum before connecting battery

### For Testing (Desktop)

1. **Run Smoke Tests**
   ```bash
   python3 test_simulator.py
   ```

2. **Run Flight Simulation**
   ```bash
   python3 test_flight_sim.py
   ```

## 📋 Pre-Flight Checklist

- [ ] **REMOVE PROPELLERS** for initial testing
- [ ] All wiring secure and correct
- [ ] RC receiver bound and responding
- [ ] Throttle at minimum (below 1200μs)
- [ ] Motors respond to throttle input
- [ ] IMU orientation correct (see README.md)
- [ ] Emergency stop procedure practiced

## 🔧 Configuration

Edit `main.py` configuration classes:

```python
class PIDConfig:
    # Adjust PID gains here
    ROLL_RATE_P = 0.7
    # ... etc

class RCConfig:
    # Adjust RC mapping here
    RC_MIN = 995
    RC_MAX = 1945
    # ... etc
```

## 📚 Documentation

- `README.md` - Complete user guide
- `TESTING.md` - Testing procedures and checklists
- `SMOKE_TEST_RESULTS.md` - Validation results

## ⚠️ Safety

**ALWAYS**:
- Test without propellers first
- Use in open areas away from people
- Have emergency stop ready
- Follow local regulations
- Never fly indoors initially

## 🆘 Troubleshooting

**Motors don't respond**
- Check ESC calibration
- Verify pin connections
- Ensure throttle > 1200μs

**IMU errors**
- Check I2C connections
- Verify MPU6050 address (0x68)
- Check orientation

**RC not working**
- Check receiver power
- Verify channel order
- Test RC range

## 📞 Support

See inline code documentation and comments for detailed information.

**Happy Flying! 🚁**
