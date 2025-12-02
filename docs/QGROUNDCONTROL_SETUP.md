

# QGroundControl Setup Guide

Complete guide for using QGroundControl with the flight controller.

## Table of Contents
- [What is QGroundControl?](#what-is-qgroundcontrol)
- [Installation](#installation)
- [Connection Setup](#connection-setup)
- [Using with Simulation](#using-with-simulation)
- [Data Logging and Analysis](#data-logging-and-analysis)
- [Troubleshooting](#troubleshooting)

---

## What is QGroundControl?

QGroundControl (QGC) is a free, open-source ground control station application for drones. It provides:

- **Real-time telemetry visualization** - Attitude, position, battery, etc.
- **Mission planning** - Create waypoint missions
- **Parameter tuning** - Adjust PID gains and other parameters
- **Flight log analysis** - Review flight data
- **Vehicle control** - Send commands to the drone

**Website:** http://qgroundcontrol.com/

---

## Installation

### macOS
```bash
# Download from website
open http://qgroundcontrol.com/downloads/

# Or install via Homebrew
brew install --cask qgroundcontrol
```

### Linux
```bash
# Download AppImage
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

# Make executable
chmod +x QGroundControl.AppImage

# Run
./QGroundControl.AppImage
```

### Windows
Download installer from: http://qgroundcontrol.com/downloads/

---

## Connection Setup

### 1. Configure Communication Link

1. **Open QGroundControl**

2. **Go to Application Settings**
   - Click the **Q** icon (top left)
   - Select **Application Settings**

3. **Add Comm Link**
   - Click **Comm Links** tab
   - Click **Add** button
   - Configure:
     ```
     Name: Flight Controller
     Type: UDP
     Port: 14550
     Server Addresses: (leave default)
     ```
   - Click **OK**

4. **Connect**
   - Select your new link
   - Click **Connect**

### 2. Start Telemetry Stream

The flight controller must be running and sending MAVLink telemetry to UDP port 14550.

---

## Using with Simulation

### Option 1: PyBullet Simulation with Logging

Run the flight simulation with built-in telemetry:

```bash
cd /path/to/flight_controller
python examples/flight_with_logging.py
```

This will:
- Start PyBullet 3D simulation (visual)
- Stream telemetry to QGroundControl on port 14550
- Log all data to CSV file in `flight_logs/`

**Command-line options:**
```bash
# Custom flight duration
python examples/flight_with_logging.py --duration 60

# Disable telemetry (logging only)
python examples/flight_with_logging.py --no-telemetry

# Custom log name
python examples/flight_with_logging.py --log-name test_flight_01
```

### Option 2: QGroundControl Connection Test

Test QGC connection without full simulation:

```bash
python examples/test_qgroundcontrol.py
```

This sends simulated telemetry data to verify the connection works.

### Option 3: Demo Telemetry

Run the interactive console demo:

```bash
python telemetry/demo_telemetry.py
```

---

## QGroundControl Features

### Main Display

Once connected, QGC shows:

- **Attitude Indicator** - Pitch and roll visualization
- **Compass** - Heading indicator
- **Altitude** - Height above ground
- **Speed** - Horizontal velocity
- **Battery** - Voltage and remaining percentage
- **GPS** - Satellites and position (if GPS available)

### Flight Data View

Click **Analyze Tools** to access:

- **Telemetry** - Real-time graphs of all telemetry data
- **Log Download** - Download logs from vehicle
- **GeoTag Images** - Associate photos with flight path

### Parameters

Access PID tuning and configuration:

1. Click **Vehicle Setup** (gear icon)
2. Select **Parameters**
3. Browse categories or search for specific parameters

**Common Parameters:**
- `PID_ROLL_*` - Roll PID gains
- `PID_PITCH_*` - Pitch PID gains
- `PID_YAW_*` - Yaw PID gains
- `RC_*` - RC input calibration
- `BATT_*` - Battery configuration

---

## Data Logging and Analysis

### Flight Logs

The flight controller logs all data to CSV files:

**Log Location:** `flight_logs/pybullet_flight_YYYYMMDD_HHMMSS.csv`

**Log Contents:**
- Timestamps (ms and μs)
- IMU data (gyro, accel, attitude)
- RC inputs
- PID outputs
- Motor commands
- System status

### Analyzing Logs

#### Option 1: PlotJuggler (Recommended)

PlotJuggler is a powerful time-series visualization tool:

```bash
# Install
# macOS
brew install plotjuggler

# Linux
sudo apt install plotjuggler

# Run with log file
plotjuggler flight_logs/pybullet_flight_*.csv
```

**Features:**
- Drag-and-drop plots
- Multiple synchronized views
- Zoom and pan
- Export to images

#### Option 2: Python/Pandas

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load log
df = pd.read_csv('flight_logs/pybullet_flight_20231202_143022.csv')

# Plot attitude
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp_ms'], df['roll'], label='Roll')
plt.plot(df['timestamp_ms'], df['pitch'], label='Pitch')
plt.plot(df['timestamp_ms'], df['yaw'], label='Yaw')
plt.xlabel('Time (ms)')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.grid(True)
plt.show()

# Analyze PID performance
print("Roll PID Statistics:")
print(df['pid_roll_output'].describe())
```

#### Option 3: Excel/LibreOffice

Simply open the CSV file in any spreadsheet application.

### Log Statistics

Get quick statistics from a log file:

```python
from src.data_logger import LoggerStats

stats = LoggerStats.analyze_log('flight_logs/pybullet_flight_20231202_143022.csv')
print(f"Flight duration: {stats['duration_seconds']:.1f}s")
print(f"Average loop rate: {stats['avg_loop_rate']:.1f} Hz")
print(f"Roll range: {stats['attitude_stats']['roll']['min']:.1f}° to "
      f"{stats['attitude_stats']['roll']['max']:.1f}°")
```

---

## Troubleshooting

### QGC doesn't show vehicle

**Check:**
1. Is the flight controller running?
2. Is telemetry sending to port 14550?
3. Is firewall blocking UDP port 14550?

```bash
# Test if port is listening (macOS/Linux)
netstat -an | grep 14550

# macOS firewall: System Preferences → Security & Privacy → Firewall
# Allow QGroundControl through firewall
```

### Connection lost/unstable

**Solutions:**
- Reduce telemetry rate
- Check network configuration
- Ensure no other application is using port 14550

### Wrong attitude/position displayed

**Check:**
- Roll/pitch/yaw units (should be radians for MAVLink)
- Coordinate frame (NED: North-East-Down)
- Position signs (down is negative in NED)

### Missing telemetry data

**Verify messages are being sent:**
```python
# Add debug output to your code
telemetry.send_heartbeat()
print(f"Sent heartbeat: {telemetry.stats()}")
```

### Port already in use

```bash
# Find what's using port 14550
lsof -i :14550

# Kill the process if needed
kill -9 <PID>
```

---

## Advanced Configuration

### Custom Telemetry Rates

Modify telemetry rates in your code:

```python
# High-rate attitude (50 Hz)
if loop_count % 5 == 0:  # 240 Hz / 5 = 48 Hz
    telemetry.send_attitude(...)

# Medium-rate position (10 Hz)
if loop_count % 24 == 0:  # 240 Hz / 24 = 10 Hz
    telemetry.send_local_position(...)

# Low-rate system status (1 Hz)
if loop_count % 240 == 0:  # 240 Hz / 240 = 1 Hz
    telemetry.send_sys_status(...)
```

### Multiple Vehicle Support

Connect multiple vehicles by using different system IDs:

```python
# Vehicle 1
telemetry1 = MAVLinkTelemetry(system_id=1)

# Vehicle 2
telemetry2 = MAVLinkTelemetry(system_id=2)
```

### Custom MAVLink Messages

Extend the telemetry module to send custom messages:

```python
def send_custom_message(self, data):
    msg = self.mav.command_long_encode(
        self.target_system,
        self.target_component,
        mavutil.mavlink.MAV_CMD_USER_1,
        0,  # confirmation
        data, 0, 0, 0, 0, 0, 0
    )
    self.connection.write(msg.pack(self.mav))
```

---

## Resources

### Documentation
- **QGroundControl User Guide:** https://docs.qgroundcontrol.com/
- **MAVLink Protocol:** https://mavlink.io/
- **PlotJuggler:** https://github.com/facontidavide/PlotJuggler

### Community
- **QGC Discussion Forum:** https://discuss.px4.io/c/qgroundcontrol
- **MAVLink Slack:** https://mavlink.io/en/about/support.html

### Alternative Ground Stations
- **Mission Planner** (Windows): https://ardupilot.org/planner/
- **MAVProxy** (Command-line): https://ardupilot.org/mavproxy/
- **APM Planner 2** (Cross-platform): https://ardupilot.org/planner2/

---

## Quick Reference

### Starting a Flight with Logging

```bash
# 1. Start QGroundControl
# 2. Configure UDP connection (port 14550)
# 3. Connect

# 4. Run simulation
python examples/flight_with_logging.py

# 5. Watch telemetry in QGC
# 6. Flight data is logged to flight_logs/
```

### Analyzing Flight Data

```bash
# View in PlotJuggler
plotjuggler flight_logs/pybullet_flight_*.csv

# Or analyze in Python
python -c "
from src.data_logger import LoggerStats
import sys
stats = LoggerStats.analyze_log(sys.argv[1])
print(stats)
" flight_logs/pybullet_flight_*.csv
```

### Common QGC Keyboard Shortcuts

- **Space** - Arm/disarm (when implemented)
- **T** - Takeoff (when implemented)
- **L** - Land (when implemented)
- **R** - Return to home (when implemented)
- **Esc** - Close dialogs

---

**Next Steps:**
1. Install QGroundControl
2. Run `python examples/test_qgroundcontrol.py` to verify connection
3. Run `python examples/flight_with_logging.py` for full simulation
4. Analyze logs with PlotJuggler
5. Tune PID parameters and repeat

Happy flying! 🚁
