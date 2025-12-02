# MAVLink Telemetry Testing Guide

This document describes how to test the MAVLink telemetry integration.

## Quick Start

### 1. Unit Tests (Verify MAVLink Module)

```bash
# Run all MAVLink tests
python -m pytest test_mavlink.py -v

# Or run as standalone
python test_mavlink.py
```

**Tests included:**
- Message encoding (HEARTBEAT, ATTITUDE, POSITION, etc.)
- UDP connection initialization
- Statistics tracking
- Message validation
- Telemetry stream simulation

### 2. Console Demo (No Ground Station Required)

```bash
# Simple telemetry demo with console output
python demo_telemetry.py
```

This simulates flight telemetry and prints it to console. Optional: Connect QGroundControl to `udp://localhost:14550` to see it in a ground station.

**Features:**
- Simulated flight dynamics (takeoff, hover, oscillations)
- Real-time console display of attitude, position, battery, motors
- MAVLink telemetry stream on UDP port 14550
- Command handling (ARM/DISARM from GCS)

### 3. PyBullet Integration Test

```bash
# Test telemetry with physics simulation
python -m pytest test_pybullet.py::test_pybullet_mavlink_integration -v
```

Verifies MAVLink works correctly with PyBullet simulator, sending real sensor data over telemetry.

### 4. Full Simulator with Telemetry

```bash
# PyBullet simulator with MAVLink telemetry
python example_mavlink.py --simulator pybullet

# Enhanced physics simulator with telemetry
python example_mavlink.py --simulator enhanced
```

**Connect Ground Control Station:**
- **QGroundControl**: `udp://localhost:14550`
- **MAVProxy**: `mavproxy.py --master=udp:127.0.0.1:14550`

## Ground Control Stations

### QGroundControl (Recommended)

1. Download: https://qgroundcontrol.com
2. Launch QGroundControl
3. It should auto-connect to `udp://localhost:14550`
4. You'll see:
   - Attitude indicator (roll, pitch, yaw)
   - Altitude and velocity
   - Battery status
   - Motor outputs
   - RC inputs

### MAVProxy (Command-Line)

```bash
# Install MAVProxy
pip install MAVProxy

# Connect to UDP
mavproxy.py --master=udp:127.0.0.1:14550

# With map and console
mavproxy.py --master=udp:127.0.0.1:14550 --console --map
```

### Mission Planner (Windows)

1. Download: https://ardupilot.org/planner/
2. Select "UDP" connection
3. Enter `127.0.0.1:14550`
4. Click "Connect"

## Test Coverage

**47 total tests:**
- 24 unit tests (PID, RC, ESC, IMU, HAL)
- 11 PyBullet tests (basic + advanced flight control)
- 12 MAVLink tests (message encoding, telemetry stream)

```bash
# Run all tests
python -m pytest -v

# Run specific test suites
python -m pytest test_mavlink.py -v       # MAVLink only
python -m pytest test_pybullet.py -v      # PyBullet only
```

## MAVLink Messages Tested

| Message | Rate | Test Coverage |
|---------|------|---------------|
| HEARTBEAT | 1 Hz | ✅ Verified |
| ATTITUDE | 50 Hz | ✅ Verified |
| LOCAL_POSITION_NED | 50 Hz | ✅ Verified |
| SYS_STATUS | 1 Hz | ✅ Verified |
| RC_CHANNELS | 10 Hz | ✅ Verified |
| SERVO_OUTPUT_RAW | 10 Hz | ✅ Verified |
| STATUSTEXT | As needed | ✅ Verified |
| COMMAND_LONG | Incoming | ✅ Handled |
| COMMAND_ACK | Response | ✅ Verified |

## Troubleshooting

### pymavlink Not Installed

```bash
pip install pymavlink
```

### QGroundControl Won't Connect

1. Check firewall allows UDP port 14550
2. Verify simulator is running
3. Try manual connection: Application Settings → Comm Links → Add → UDP → Port 14550

### No Telemetry Data

1. Check simulator is sending: Look for "TX=..." in console output
2. Verify port 14550 not in use: `lsof -i :14550` (macOS/Linux)
3. Check MAVLink statistics: Look for packets_sent > 0

### High Latency

1. UDP on localhost should be <1ms latency
2. Check CPU usage: PyBullet + QGC can be demanding
3. Reduce telemetry rate if needed (modify send intervals)

## Example Output

**Console Demo:**
```
═════════════════════════════════════════════════════════
Time: 15.2s  |  Loop: 760  |  ARMED
═════════════════════════════════════════════════════════
ATTITUDE:
  Roll:  +  5.73°  Rate: + 0.52°/s
  Pitch: - 2.41°  Rate: - 1.23°/s
  Yaw:   + 34.21°  Rate: + 1.15°/s

POSITION:
  Altitude:   9.87m
  Velocity: [+0.12, -0.08, +0.02] m/s

SYSTEM:
  Battery:  12.45V  16.2A  92%
  CPU Load: 35%

MOTORS (PWM μs):
  M0:1565  M1:1548  M2:1572
  M3:1556  M4:1541  M5:1563

TELEMETRY:
  TX: 412 packets  RX: 3 packets  Commands: 1
  ⚠ Ground Control Station connected!
```

## Next Steps

1. ✅ Test telemetry messages (unit tests)
2. ✅ Test with console demo
3. ✅ Test PyBullet integration
4. 🔄 Test with QGroundControl
5. 🔄 Test with hardware (PyBoard + telemetry radio)

## Hardware Testing

When testing with actual hardware:

```python
from hal_pyboard import PyBoardHAL
from mavlink_telemetry import MAVLinkTelemetry

# Initialize hardware
hal = PyBoardHAL()

# Initialize telemetry radio (UART)
mavlink = MAVLinkTelemetry(port='/dev/ttyUSB0', baudrate=57600)

# Main loop
while True:
    # Read sensors
    imu = hal.read_imu()
    rc = hal.read_rc()
    
    # Send telemetry
    mavlink.send_heartbeat(armed=True)
    mavlink.send_attitude(roll, pitch, yaw, p, q, r)
    mavlink.send_position(x, y, z, vx, vy, vz)
    
    # Handle commands
    cmd = mavlink.handle_commands()
    if cmd and cmd['command_name'] == 'ARM_DISARM':
        armed = (cmd['param1'] == 1.0)
        mavlink.send_command_ack(cmd['command'], result=0)
    
    time.sleep(0.02)  # 50Hz
```

---

**Last updated:** December 2, 2025
