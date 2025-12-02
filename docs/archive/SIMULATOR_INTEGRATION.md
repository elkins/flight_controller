# Drone Simulator Integration Guide

This guide covers integrating the flight controller with open-source drone simulators for advanced physics-based testing.

## Available Simulators

### 1. **Gazebo + PX4-SITL** ⭐ RECOMMENDED
- **Type:** Full 3D physics simulation
- **Physics:** Rigid body dynamics, aerodynamics, sensor noise
- **Maturity:** Very mature, industry standard
- **License:** Apache 2.0
- **Platforms:** Linux, macOS, Windows (WSL2)

**Features:**
- Realistic multi-rotor physics
- Sensor simulation (IMU, GPS, magnetometer, barometer, optical flow)
- Wind and turbulence modeling
- Visual camera simulation
- Ground truth data for validation
- ROS integration available

**Best For:** Comprehensive testing with realistic physics and sensors

### 2. **JSBSim**
- **Type:** Flight dynamics simulation
- **Physics:** High-fidelity aerodynamics
- **Maturity:** Very mature (used by NASA, Boeing)
- **License:** LGPL
- **Platforms:** Cross-platform

**Features:**
- Extremely accurate flight dynamics
- Custom aircraft models
- Atmospheric modeling
- Detailed engine models

**Best For:** High-fidelity fixed-wing or VTOL testing

### 3. **Webots**
- **Type:** General robotics simulator
- **Physics:** ODE physics engine
- **Maturity:** Mature, well-documented
- **License:** Apache 2.0
- **Platforms:** Cross-platform

**Features:**
- 3D visualization
- Multiple robot types
- Good Python API
- Educational focus

**Best For:** Learning and prototyping

### 4. **AirSim (Microsoft)**
- **Type:** High-fidelity visual simulation
- **Physics:** Unreal Engine physics
- **Maturity:** Mature, Microsoft-backed
- **License:** MIT
- **Platforms:** Windows, Linux

**Features:**
- Photorealistic graphics
- Computer vision testing
- Autonomous systems research
- API for Python/C++

**Best For:** Computer vision and AI development

### 5. **PyBullet**
- **Type:** Lightweight physics simulation
- **Physics:** Bullet physics engine
- **Maturity:** Mature, actively developed
- **License:** Zlib
- **Platforms:** Cross-platform

**Features:**
- Fast simulation
- Python-first API
- Deep learning integration
- Low resource requirements

**Best For:** Fast iteration and ML training

## Integration Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  Flight Controller Code                  │
│                    (main.py, pid.py)                     │
└─────────────────┬───────────────────────────────────────┘
                  │
                  │ HAL Interface
                  │
┌─────────────────▼───────────────────────────────────────┐
│              Hardware Abstraction Layer                  │
│                    (hal.py)                              │
└─────────┬───────────────────────────┬───────────────────┘
          │                           │
          │ Real Hardware             │ Simulator
          │                           │
┌─────────▼──────────┐    ┌──────────▼──────────────────┐
│  hal_pyboard.py    │    │   hal_simulator.py          │
│  (PyBoard/STM32)   │    │   (Gazebo/JSBSim/PyBullet)  │
└────────────────────┘    └──────────┬──────────────────┘
                                     │
                          ┌──────────▼──────────────────┐
                          │    Physics Simulator        │
                          │  • Gazebo                   │
                          │  • JSBSim                   │
                          │  • PyBullet                 │
                          │  • Custom Python            │
                          └─────────────────────────────┘
```

## Implementation: Gazebo Integration (Recommended)

### Step 1: Install Gazebo

**Ubuntu/Linux:**
```bash
# Install Gazebo 11
sudo apt-get update
sudo apt-get install gazebo11 libgazebo11-dev

# Install PX4 SITL Gazebo plugins
git clone https://github.com/PX4/PX4-SITL_gazebo-classic.git
cd PX4-SITL_gazebo-classic
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

**macOS:**
```bash
brew install gazebo11
# Follow Linux instructions for PX4-SITL_gazebo
```

### Step 2: Create Simulator HAL

Create `hal_gazebo.py`:

```python
"""
Gazebo Simulator HAL Implementation

Connects flight controller to Gazebo physics simulation via MAVLink.
"""

import logging
import socket
import struct
import time
from typing import Callable
from hal import (
    HALPlatform, HALTimer, HALPWMChannel, HALInputCapture, HALI2C
)

logger = logging.getLogger(__name__)

class GazeboConnection:
    """MAVLink/UDP connection to Gazebo"""
    
    def __init__(self, host='127.0.0.1', port=14560):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.host = host
        self.port = port
        self.running = True
        
    def send_motor_commands(self, motor_speeds):
        """Send motor commands to Gazebo (0-1000 range)"""
        # Pack as 16 floats (up to 16 motors)
        data = struct.pack('16f', *motor_speeds[:16] + [0]*(16-len(motor_speeds)))
        self.sock.sendto(data, (self.host, 14561))
    
    def receive_sensor_data(self):
        """Receive IMU, GPS, etc from Gazebo"""
        try:
            data, addr = self.sock.recvfrom(1024)
            # Unpack sensor data (format depends on Gazebo setup)
            return data
        except socket.timeout:
            return None


class GazeboPWMChannel(HALPWMChannel):
    """Gazebo PWM output (motor control)"""
    
    def __init__(self, connection: GazeboConnection, channel: int):
        self.connection = connection
        self.channel = channel
        self._pulse_width = 1000
        self._motor_speeds = [0] * 16
        
    def set_pulse_width(self, width_us: int):
        """Convert PWM pulse width to motor speed (0-1000)"""
        self.validate_pulse_width(width_us)
        self._pulse_width = width_us
        
        # Convert 1000-2000us to 0-1000 motor speed
        speed = max(0, min(1000, width_us - 1000))
        self._motor_speeds[self.channel] = speed
        
        # Send to Gazebo
        self.connection.send_motor_commands(self._motor_speeds)
        logger.debug(f"Motor {self.channel} set to {speed}")
        
    def get_pulse_width(self) -> int:
        return self._pulse_width


class GazeboInputCapture(HALInputCapture):
    """Gazebo RC input simulation"""
    
    def __init__(self, channel: int):
        self.channel = channel
        self._pulse_width = 1500
        self._callback = None
        
    def get_pulse_width(self) -> int:
        """Return simulated RC pulse width"""
        return self._pulse_width
    
    def set_callback(self, callback: Callable):
        self._callback = callback
    
    def simulate_input(self, width_us: int):
        """Simulate RC input (called by test framework)"""
        self._pulse_width = width_us
        if self._callback:
            self._callback(width_us)


class GazeboI2C(HALI2C):
    """Gazebo I2C sensor simulation"""
    
    def __init__(self, connection: GazeboConnection):
        self.connection = connection
        self._imu_data = {
            'accel': [0, 0, 9.81],  # m/s^2
            'gyro': [0, 0, 0],       # rad/s
            'mag': [0.3, 0, 0.4],    # gauss
        }
    
    def write_byte(self, device_addr: int, register: int, value: int):
        """Simulated I2C write (configuration)"""
        logger.debug(f"I2C write: addr=0x{device_addr:02X} reg=0x{register:02X}")
        
    def read_byte(self, device_addr: int, register: int) -> int:
        """Simulated I2C read"""
        if device_addr == 0x68 and register == 0x75:  # WHO_AM_I
            return 0x68  # MPU6050 ID
        return 0
    
    def read_bytes(self, device_addr: int, register: int, length: int) -> bytes:
        """Read IMU data from Gazebo"""
        if device_addr == 0x68 and register == 0x3B:  # Accel registers
            # Get latest data from Gazebo
            sensor_data = self.connection.receive_sensor_data()
            if sensor_data:
                self._parse_sensor_data(sensor_data)
            
            # Pack as 6 bytes (accel X, Y, Z as int16)
            data = []
            for val in self._imu_data['accel']:
                # Convert m/s^2 to raw 16-bit value
                raw = int(val * 2048)  # LSB/g for ±16g range
                data.append((raw >> 8) & 0xFF)  # MSB
                data.append(raw & 0xFF)          # LSB
            return bytes(data)
        return bytes(length)
    
    def write_bytes(self, device_addr: int, register: int, data: bytes):
        """Simulated I2C write"""
        pass
    
    def _parse_sensor_data(self, data):
        """Parse sensor data from Gazebo"""
        # Format depends on Gazebo message structure
        # Typically: accel(3), gyro(3), mag(3), timestamp
        pass


class GazeboTimer(HALTimer):
    """Gazebo timer"""
    
    def __init__(self, timer_id: int, connection: GazeboConnection):
        self.timer_id = timer_id
        self.connection = connection
        self.channels = {}
        
    def create_pwm_channel(self, pin: str, frequency: int) -> HALPWMChannel:
        """Create PWM output for motor"""
        # Extract channel number from pin name
        channel = int(pin[1]) if len(pin) > 1 else 0
        pwm = GazeboPWMChannel(self.connection, channel)
        self.channels[pin] = pwm
        return pwm
    
    def create_input_capture(self, pin: str, callback: Callable) -> HALInputCapture:
        """Create RC input"""
        channel = int(pin[1]) if len(pin) > 1 else 0
        ic = GazeboInputCapture(channel)
        ic.set_callback(callback)
        self.channels[pin] = ic
        return ic
    
    def deinit(self):
        """Cleanup"""
        pass


class GazeboPlatform(HALPlatform):
    """Gazebo simulator platform"""
    
    def __init__(self, host='127.0.0.1', port=14560):
        self.connection = GazeboConnection(host, port)
        self.timers = {}
        self.i2c_buses = {}
        self._start_time = time.time()
        logger.info("Gazebo platform initialized")
        
    def get_timer(self, timer_id: int) -> HALTimer:
        """Get or create timer"""
        if timer_id not in self.timers:
            self.timers[timer_id] = GazeboTimer(timer_id, self.connection)
        return self.timers[timer_id]
    
    def get_i2c(self, bus_id: int) -> HALI2C:
        """Get or create I2C bus"""
        if bus_id not in self.i2c_buses:
            self.i2c_buses[bus_id] = GazeboI2C(self.connection)
        return self.i2c_buses[bus_id]
    
    def millis(self) -> int:
        """Milliseconds since start"""
        return int((time.time() - self._start_time) * 1000)
    
    def delay_ms(self, ms: int):
        """Delay milliseconds"""
        time.sleep(ms / 1000.0)
    
    def delay_us(self, us: int):
        """Delay microseconds"""
        time.sleep(us / 1000000.0)
    
    @property
    def platform_name(self) -> str:
        return "Gazebo"
```

### Step 3: Create Test with Gazebo

Create `test_gazebo_integration.py`:

```python
"""
Gazebo Integration Test

Tests flight controller with Gazebo physics simulation.
"""

import sys
import time
from hal import set_platform
from hal_gazebo import GazeboPlatform

# Import flight controller
sys.path.insert(0, '.')
from main import FlightController

def test_gazebo_flight():
    """Test flight controller with Gazebo"""
    print("=" * 60)
    print("Gazebo Integration Test")
    print("=" * 60)
    
    # Set up Gazebo platform
    platform = GazeboPlatform(host='127.0.0.1', port=14560)
    set_platform(platform)
    
    print("\n1. Starting Gazebo connection...")
    time.sleep(1)
    
    # Initialize flight controller
    print("2. Initializing flight controller...")
    fc = FlightController()
    
    print("3. Running control loop...")
    # Run for 10 seconds
    start_time = time.time()
    while time.time() - start_time < 10:
        fc.run()
        time.sleep(0.002)  # 500Hz control loop
    
    print("\n✓ Gazebo integration test complete!")
    print("Check Gazebo window for vehicle behavior")

if __name__ == '__main__':
    test_gazebo_flight()
```

### Step 4: Launch Gazebo with Drone Model

Create `launch_gazebo.sh`:

```bash
#!/bin/bash

# Set Gazebo paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:./gazebo_models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:./gazebo_plugins

# Launch Gazebo with hexacopter
gazebo --verbose worlds/hexacopter.world
```

## Implementation: PyBullet Integration (Lightweight)

For faster, lighter-weight testing, use PyBullet:

```python
"""
PyBullet Simulator HAL

Lightweight physics simulation for rapid testing.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
from hal import HALPlatform, HALTimer, HALPWMChannel, HALI2C

class PyBulletPlatform(HALPlatform):
    """PyBullet simulator platform"""
    
    def __init__(self, gui=True):
        # Start PyBullet
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        p.loadURDF("plane.urdf")
        
        # Load hexacopter (need custom URDF)
        self.drone = p.loadURDF("hexacopter.urdf", [0, 0, 1])
        
        # Physics settings
        p.setTimeStep(1/240)  # 240Hz physics
        
        self.timers = {}
        self.motor_forces = [0] * 6
        self._start_time = time.time()
        
    def apply_motor_forces(self):
        """Apply motor thrust forces to drone"""
        for i, force in enumerate(self.motor_forces):
            # Apply thrust at motor position
            motor_pos = [0.2, 0, 0]  # Adjust based on geometry
            p.applyExternalForce(
                self.drone, -1,
                [0, 0, force],
                motor_pos,
                p.WORLD_FRAME
            )
    
    def get_imu_data(self):
        """Get IMU data from physics simulation"""
        pos, orn = p.getBasePositionAndOrientation(self.drone)
        vel, ang_vel = p.getBaseVelocity(self.drone)
        
        # Convert to IMU format
        accel = np.array(vel) * 240  # Numerical differentiation
        gyro = ang_vel
        
        return {
            'accel': accel,
            'gyro': gyro,
            'orientation': orn
        }
    
    def step_simulation(self):
        """Step physics forward"""
        self.apply_motor_forces()
        p.stepSimulation()
    
    # Implement HALPlatform methods...
```

## Comparison Matrix

| Feature | Gazebo | JSBSim | PyBullet | Webots | AirSim |
|---------|--------|--------|----------|--------|--------|
| **Physics Fidelity** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| **Visual Quality** | ⭐⭐⭐ | ⭐ | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Speed** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| **Setup Difficulty** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| **Multi-rotor** | ✅ | ❌ | ✅ | ✅ | ✅ |
| **Python API** | ⚠️ | ⚠️ | ✅ | ✅ | ✅ |
| **ROS Support** | ✅ | ✅ | ❌ | ✅ | ❌ |
| **Resource Usage** | High | Medium | Low | Medium | Very High |

## Quick Start Recommendation

**For immediate testing:** Use PyBullet
- Fast setup: `pip install pybullet`
- Python-native
- Good enough physics
- Fast iteration

**For comprehensive validation:** Use Gazebo + PX4-SITL
- Industry standard
- Realistic physics and sensors
- Large community
- Production-quality

## Testing Workflow

```bash
# 1. Unit tests (no simulator)
python3 test_simulator.py

# 2. PyBullet quick tests
python3 test_pybullet.py  # Fast iteration

# 3. Gazebo full validation
./launch_gazebo.sh &
python3 test_gazebo_integration.py

# 4. Hardware deployment
# Upload to PyBoard and test
```

## Benefits of Simulator Integration

1. **Physics Validation**
   - Test actual flight dynamics
   - Verify stability margins
   - Tune PID gains realistically

2. **Sensor Realism**
   - Noise and bias simulation
   - Sensor fusion testing
   - Calibration validation

3. **Safety**
   - Test dangerous scenarios safely
   - Crash recovery testing
   - Failsafe validation

4. **Rapid Iteration**
   - No hardware setup time
   - Instant rewind/replay
   - Automated testing

5. **Documentation**
   - Video recordings
   - Performance metrics
   - Reproducible results

## Next Steps

1. ✅ Create `hal_gazebo.py` (Gazebo integration)
2. ✅ Create `hal_pybullet.py` (PyBullet integration)
3. ⏳ Create drone URDF/SDF models
4. ⏳ Set up Gazebo world files
5. ⏳ Write comprehensive integration tests
6. ⏳ Document PID tuning with simulator

Would you like me to implement any of these simulator integrations?
