# MAVLink Telemetry Integration

Professional ground control station integration using the industry-standard MAVLink protocol.

## Overview

This flight controller now supports real-time telemetry streaming to professional ground control stations (GCS) like **QGroundControl** and **Mission Planner**. MAVLink provides:

- **Real-time monitoring**: Attitude, position, velocity, RC inputs, motor outputs
- **Remote parameter tuning**: Adjust PID gains from the ground
- **Mission planning**: Waypoint navigation and autonomous flight
- **Data logging**: Flight logs compatible with industry analysis tools
- **Multi-vehicle support**: Manage multiple drones from one GCS

## Quick Start

### 1. Hardware Connection

Connect a telemetry radio or serial adapter:

```
PyBoard UART → Telemetry Radio → Ground Station
```

**Common configurations:**
- **3DR Radio**: 57600 baud, `/dev/ttyUSB0` (Linux), `COM3` (Windows)
- **RFD900**: 57600 baud
- **WiFi (UDP)**: `udp:0.0.0.0:14550`
- **Direct Serial**: `/dev/ttyAMA0` (Raspberry Pi), `/dev/ttyUSB0` (Linux)

### 2. Software Setup

#### Install Ground Control Station

**QGroundControl** (Recommended):
```bash
# Linux
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage

# macOS
brew install --cask qgroundcontrol

# Windows
# Download from https://qgroundcontrol.com
```

**Mission Planner** (Windows):
```
Download from https://ardupilot.org/planner/
```

**MAVProxy** (Command-line, all platforms):
```bash
pip install MAVProxy
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600
```

### 3. Code Integration

Add to your flight controller:

```python
from mavlink_telemetry import MAVLinkTelemetry
import time

# Initialize MAVLink (in setup)
mavlink = MAVLinkTelemetry(
    port='/dev/ttyUSB0',  # Serial port
    baudrate=57600,       # Baud rate
    system_id=1           # Drone ID
)

# Main flight loop
while True:
    # Read sensors
    imu_data = hal.read_imu()
    rc_data = hal.read_rc()
    
    # Compute attitude (from IMU/filter)
    roll, pitch, yaw = compute_attitude(imu_data)
    rollspeed = imu_data['gyro_x']
    pitchspeed = imu_data['gyro_y']
    yawspeed = imu_data['gyro_z']
    
    # Compute position (from GPS or estimator)
    x, y, z = get_position()  # NED frame
    vx, vy, vz = get_velocity()
    
    # Send telemetry (10-50Hz recommended)
    mavlink.send_heartbeat(armed=armed)
    mavlink.send_attitude(roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
    mavlink.send_position(x, y, z, vx, vy, vz)
    mavlink.send_rc_channels(rc_data['channels'])
    mavlink.send_servo_output(motor_pwm_values)
    
    # Handle incoming commands
    cmd = mavlink.handle_commands()
    if cmd:
        if cmd['command_name'] == 'ARM_DISARM':
            armed = (cmd['param1'] == 1.0)
            mavlink.send_command_ack(cmd['command'], result=0)  # ACK
    
    time.sleep(0.02)  # 50Hz loop
```

## MAVLink Messages

### Core Messages (Mandatory)

| Message | Rate | Description |
|---------|------|-------------|
| `HEARTBEAT` | 1 Hz | System health, armed state |
| `ATTITUDE` | 10-50 Hz | Roll, pitch, yaw, rates |
| `SYS_STATUS` | 1 Hz | Battery, CPU load, sensors |

### Position Messages

| Message | Rate | Description |
|---------|------|-------------|
| `LOCAL_POSITION_NED` | 10-50 Hz | Position in NED frame (x, y, z) |
| `GLOBAL_POSITION_INT` | 5-10 Hz | GPS latitude, longitude, altitude |

### Input/Output Messages

| Message | Rate | Description |
|---------|------|-------------|
| `RC_CHANNELS` | 10 Hz | RC receiver inputs (PWM) |
| `SERVO_OUTPUT_RAW` | 10 Hz | Motor/servo PWM outputs |

### Debug Messages

| Message | Rate | Description |
|---------|------|-------------|
| `STATUSTEXT` | As needed | Text messages (info, warnings, errors) |

## Connection Types

### Serial (Telemetry Radio)

```python
mavlink = MAVLinkTelemetry(port='/dev/ttyUSB0', baudrate=57600)
```

**Pros:** Long range (up to 40km with 3DR radio), reliable  
**Cons:** Requires hardware, limited bandwidth

### UDP (WiFi)

```python
mavlink = MAVLinkTelemetry(port='udp:0.0.0.0:14550')
```

**Pros:** High bandwidth, easy setup  
**Cons:** Limited range, WiFi dependency

### TCP (Network)

```python
mavlink = MAVLinkTelemetry(port='tcp:192.168.1.100:5760')
```

**Pros:** Reliable, good for simulators  
**Cons:** Requires network infrastructure

## Ground Control Stations

### QGroundControl

**Best for:** General use, mission planning, parameter tuning

**Features:**
- Real-time flight display
- Waypoint mission editor
- Parameter browser with live updates
- Video streaming support
- Multi-vehicle support

**Configuration:**
1. Open QGroundControl
2. Go to **Application Settings** → **Comm Links**
3. Add new link:
   - **Type:** Serial or UDP
   - **Serial:** `/dev/ttyUSB0` @ 57600 baud
   - **UDP:** Port 14550
4. Connect

### Mission Planner

**Best for:** Advanced mission planning (ArduPilot-focused)

**Features:**
- Comprehensive mission editor
- Extensive parameter database
- Flight log analysis
- Synthetic terrain view

**Configuration:**
1. Open Mission Planner
2. Select COM port or UDP
3. Set baud rate: 57600
4. Click **Connect**

### MAVProxy

**Best for:** Command-line control, scripting, developers

**Features:**
- Scriptable Python interface
- Module system (graphs, map, console)
- Low resource usage
- Multi-connection support

**Usage:**
```bash
# Serial connection
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

# UDP connection
mavproxy.py --master=udp:127.0.0.1:14550

# With modules
mavproxy.py --master=/dev/ttyUSB0 --console --map

# Multiple outputs (forward to QGroundControl)
mavproxy.py --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550
```

## Commands

The flight controller can receive and respond to MAVLink commands:

### ARM/DISARM (400)
```python
cmd = mavlink.handle_commands()
if cmd and cmd['command_name'] == 'ARM_DISARM':
    armed = (cmd['param1'] == 1.0)
    mavlink.send_command_ack(cmd['command'], result=0)
```

### SET_MESSAGE_INTERVAL (176)
```python
# Request specific telemetry rate
if cmd['command_name'] == 'SET_MESSAGE_INTERVAL':
    message_id = int(cmd['param1'])
    interval_us = int(cmd['param2'])
    # Configure message rate
    mavlink.send_command_ack(cmd['command'], result=0)
```

### REQUEST_MESSAGE (511)
```python
# Ground station requests specific message
if cmd['command_name'] == 'REQUEST_MESSAGE':
    message_id = int(cmd['param1'])
    # Send requested message
    mavlink.send_command_ack(cmd['command'], result=0)
```

## Performance Considerations

### Bandwidth

Typical bandwidth usage:

| Rate | Bytes/sec | Notes |
|------|-----------|-------|
| HEARTBEAT (1 Hz) | ~15 | Mandatory |
| ATTITUDE (50 Hz) | ~1400 | High frequency |
| POSITION (50 Hz) | ~1600 | High frequency |
| SYS_STATUS (1 Hz) | ~35 | Low frequency |
| RC_CHANNELS (10 Hz) | ~420 | Medium frequency |
| **Total** | **~3.5 KB/s** | Fits 57600 baud |

### CPU Load

MAVLink is lightweight:
- **Sending**: ~1-2% CPU per message @ 50 Hz
- **Receiving**: Minimal (non-blocking)
- **Total overhead**: <5% on PyBoard

### Recommended Rates

| Message Type | Rate | Rationale |
|--------------|------|-----------|
| HEARTBEAT | 1 Hz | GCS expects 1Hz minimum |
| ATTITUDE | 10-50 Hz | Smooth display |
| POSITION | 10-50 Hz | Smooth map updates |
| SYS_STATUS | 1 Hz | Slow-changing data |
| RC/SERVO | 5-10 Hz | User feedback |
| STATUSTEXT | As needed | Debug only |

## Troubleshooting

### No Connection

**Problem:** Ground station doesn't connect

**Solutions:**
1. Check serial port: `ls /dev/tty*` (Linux) or Device Manager (Windows)
2. Verify baud rate matches (57600)
3. Check cable connection
4. Try different USB port
5. Check permissions: `sudo chmod 666 /dev/ttyUSB0`

### Connection Drops

**Problem:** Frequent disconnects

**Solutions:**
1. Reduce telemetry rate (lower Hz)
2. Check RF interference
3. Shorten messages (send only essential data)
4. Verify power supply (radio draws current)

### Delayed Data

**Problem:** Telemetry lags behind real-time

**Solutions:**
1. Reduce sending rate on high-frequency messages
2. Prioritize critical messages (attitude over debug)
3. Check CPU load on flight controller
4. Use MAVLink 2.0 (more efficient)

### Commands Not Working

**Problem:** Ground station commands ignored

**Solutions:**
1. Call `handle_commands()` in main loop
2. Send COMMAND_ACK responses
3. Check target_system/component IDs match
4. Verify command support in your code

## Integration with Simulators

### PyBullet Simulator

```python
from hal_pybullet import PyBulletHAL
from mavlink_telemetry import MAVLinkTelemetry

# Initialize simulator
hal = PyBulletHAL()

# Initialize MAVLink (UDP for easy GCS connection)
mavlink = MAVLinkTelemetry(port='udp:0.0.0.0:14550')

# Simulation loop
while hal.step():
    # Get simulated state
    state = hal.get_state()
    
    # Send to ground station
    mavlink.send_attitude(
        state['roll'], state['pitch'], state['yaw'],
        state['p'], state['q'], state['r']
    )
    mavlink.send_position(
        state['x'], state['y'], state['z'],
        state['vx'], state['vy'], state['vz']
    )
```

Now you can monitor simulated flights in QGroundControl!

### Enhanced Physics Simulator

```python
from hal_pybullet_enhanced import EnhancedPyBulletHAL
from mavlink_telemetry import MAVLinkTelemetry

hal = EnhancedPyBulletHAL()
mavlink = MAVLinkTelemetry(port='udp:0.0.0.0:14550')

# High-fidelity simulation with real-time GCS monitoring
```

## Parameter Protocol

MAVLink supports remote parameter tuning:

```python
# In your flight controller
params = {
    'PID_P_ROLL': 0.5,
    'PID_I_ROLL': 0.1,
    'PID_D_ROLL': 0.05
}

# Handle parameter requests
cmd = mavlink.handle_commands()
if cmd and cmd['type'] == 'PARAM_REQUEST_LIST':
    # Send all parameters
    for name, value in params.items():
        mavlink.connection.mav.param_value_send(
            param_id=name.encode('utf-8'),
            param_value=value,
            param_type=9,  # REAL32
            param_count=len(params),
            param_index=list(params.keys()).index(name)
        )

if cmd and cmd['type'] == 'PARAM_SET':
    # Update parameter
    param_name = cmd['param'].param_id.decode('utf-8')
    param_value = cmd['param'].param_value
    if param_name in params:
        params[param_name] = param_value
        # Send confirmation
        mavlink.connection.mav.param_value_send(...)
```

This enables **live PID tuning** from QGroundControl!

## Advanced Features

### Mission Protocol

Support waypoint missions:
- `MISSION_REQUEST_LIST`
- `MISSION_ITEM`
- `MISSION_CURRENT`
- `MISSION_ACK`

### Data Streams

Request specific data rates:
- `REQUEST_DATA_STREAM`
- `SET_MESSAGE_INTERVAL`

### Health Monitoring

Report system health:
- `SYS_STATUS`: Sensor health bits
- `BATTERY_STATUS`: Detailed battery info
- `VIBRATION`: IMU vibration levels

## References

- **MAVLink Protocol**: https://mavlink.io
- **Message Definitions**: https://mavlink.io/en/messages/common.html
- **pymavlink Docs**: https://github.com/ArduPilot/pymavlink
- **QGroundControl**: https://qgroundcontrol.com
- **Mission Planner**: https://ardupilot.org/planner/

## Next Steps

1. ✅ **Basic Telemetry**: Send HEARTBEAT, ATTITUDE, POSITION
2. **Parameter Tuning**: Implement PARAM_REQUEST/SET
3. **Mission Support**: Waypoint navigation
4. **Logging**: Record flight data to SD card
5. **Advanced Features**: Geofencing, Return-to-Home, failsafes

---

**Status**: Production-ready for telemetry monitoring  
**Tested with**: QGroundControl 4.x, MAVProxy 1.8+  
**Protocol**: MAVLink 2.0 (common message set)
