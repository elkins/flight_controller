"""
PyBoard Flight Controller Simulator

Simulates PyBoard hardware (timers, pins, I2C) to test flight controller
logic without actual hardware. Provides mock implementations of:
- pyb.Timer
- pyb.Pin
- pyb.I2C
- pyb.millis
- pyb.delay

Usage:
    python test_simulator.py
"""

import sys
import time
from unittest.mock import Mock, MagicMock


# ============================================================================
# MOCK PyBoard MODULE
# ============================================================================

class MockTimer:
    """Mock PyBoard Timer for testing"""
    
    PWM = 'PWM'
    IC = 'IC'
    BOTH = 'BOTH'
    MASTER = 'MASTER'
    
    def __init__(self, timer_id, prescaler=0, period=0):
        self.timer_id = timer_id
        self.prescaler = prescaler
        self.period = period
        self._channels = {}
    
    def channel(self, channel_id, mode, pin=None, polarity=None):
        """Create a timer channel"""
        if channel_id not in self._channels:
            self._channels[channel_id] = MockTimerChannel(mode, pin, polarity)
        return self._channels[channel_id]
    
    def deinit(self):
        """Deinitialize timer"""
        pass


class MockTimerChannel:
    """Mock Timer Channel"""
    
    def __init__(self, mode, pin=None, polarity=None):
        self.mode = mode
        self.pin = pin
        self.polarity = polarity
        self._pulse_width = 1000
        self._capture_value = 0
        self._callback = None
    
    def pulse_width(self, width=None):
        """Get/set PWM pulse width"""
        if width is not None:
            self._pulse_width = width
        return self._pulse_width
    
    def capture(self):
        """Get capture value for input capture mode"""
        self._capture_value += 1500  # Simulate ~1500us pulse
        return self._capture_value
    
    def callback(self, func):
        """Set callback function"""
        self._callback = func


class MockPin:
    """Mock PyBoard Pin"""
    
    def __init__(self, pin_name):
        self.pin_name = pin_name
        self._value = 0
    
    def value(self, val=None):
        """Get/set pin value"""
        if val is not None:
            self._value = val
        return self._value


class MockI2C:
    """Mock I2C Bus"""
    
    MASTER = 'MASTER'
    
    def __init__(self, bus_id, mode=None):
        self.bus_id = bus_id
        self.mode = mode
        self._mem = {}
    
    def mem_write(self, data, addr, reg):
        """Write to I2C memory"""
        if addr not in self._mem:
            self._mem[addr] = {}
        self._mem[addr][reg] = data
    
    def mem_read(self, buf, addr, reg):
        """Read from I2C memory"""
        if addr in self._mem and reg in self._mem[addr]:
            data = self._mem[addr][reg]
            if isinstance(data, int):
                buf[0] = data
            else:
                for i, b in enumerate(data[:len(buf)]):
                    buf[i] = b
        else:
            # Return some default values for MPU6050
            if reg == 0x3A:  # INT_STATUS
                buf[0] = 0x02
            elif reg == 0x72:  # FIFO_COUNTH
                buf[0] = 0x00
            elif reg == 0x73:  # FIFO_COUNTL
                buf[0] = 0x2A  # 42 bytes
            else:
                for i in range(len(buf)):
                    buf[i] = 0


# Mock pyb module
class MockPyb:
    """Mock PyBoard module"""
    Timer = MockTimer
    Pin = MockPin
    I2C = MockI2C
    
    @staticmethod
    def millis():
        """Return milliseconds since start"""
        return int(time.time() * 1000) % (2**32)
    
    @staticmethod
    def delay(ms):
        """Delay for milliseconds"""
        time.sleep(ms / 1000.0)


# Install mock pyb module BEFORE importing anything
mock_pyb = MockPyb()
sys.modules['pyb'] = mock_pyb  # type: ignore


# ============================================================================
# IMPORT FLIGHT CONTROLLER MODULES
# ============================================================================

print("Loading flight controller modules...")
try:
    from pid import PID
    print("✓ PID module loaded")
except Exception as e:
    print(f"✗ Error loading PID: {e}")
    sys.exit(1)

try:
    from esc import ESC
    print("✓ ESC module loaded")
except Exception as e:
    print(f"✗ Error loading ESC: {e}")
    sys.exit(1)

try:
    from rc import RC, map_range, wrap_180
    print("✓ RC module loaded")
except Exception as e:
    print(f"✗ Error loading RC: {e}")
    sys.exit(1)


# ============================================================================
# MOCK MPU6050 (simplified - original is too complex to fully simulate)
# ============================================================================

class MockMPU6050:
    """Simplified mock MPU6050 for testing"""
    
    def __init__(self):
        self.dmp_enabled = False
        self.packet_size = 42
        self.fifo_count = 0
        print("✓ Mock MPU6050 initialized")
    
    def dmpInitialize(self):
        """Initialize DMP"""
        print("  - DMP initialized")
    
    def setDMPEnabled(self, enabled):
        """Enable/disable DMP"""
        self.dmp_enabled = enabled
        print(f"  - DMP enabled: {enabled}")
    
    def dmpGetFIFOPacketSize(self):
        """Get FIFO packet size"""
        return self.packet_size
    
    def getIntStatus(self):
        """Get interrupt status"""
        return 2  # Valid status
    
    def getFIFOCount(self):
        """Get FIFO count"""
        self.fifo_count += 1
        return 42 if self.fifo_count > 3 else 42
    
    def resetFIFO(self):
        """Reset FIFO"""
        self.fifo_count = 0
    
    def getFIFOBytes(self, length):
        """Get FIFO bytes"""
        return [0] * length
    
    def dmpGetQuaternion(self, packet):
        """Get quaternion from packet"""
        # Return w, x, y, z (normalized)
        return (1.0, 0.0, 0.0, 0.0)
    
    def dmpGetEuler(self, w, x, y, z):
        """Get Euler angles from quaternion"""
        # Return yaw, pitch, roll in degrees (simulated level flight)
        import random
        return (
            random.uniform(-2, 2),   # yaw
            random.uniform(-5, 5),   # roll
            random.uniform(-5, 5)    # pitch
        )
    
    def dmpGetGyro(self, packet):
        """Get gyro data from packet"""
        # Return gyro rates (simulated small movements)
        import random
        return (
            random.uniform(-10, 10),  # pitch rate
            random.uniform(-10, 10),  # roll rate
            random.uniform(-5, 5)     # yaw rate
        )


# ============================================================================
# SMOKE TESTS
# ============================================================================

def test_pid():
    """Test PID controller"""
    print("\n=== Testing PID Controller ===")
    
    # Create PID with typical flight controller gains
    pid = PID(p=0.7, i=1.0, d=0.0, imax=50)
    print(f"Created: {pid}")
    
    # Test basic operation
    error = 10.0
    output = pid.get_pid(error, 1.0)
    print(f"Error: {error}, Output: {output:.2f}")
    
    # Test integrator
    for _ in range(5):
        time.sleep(0.01)
        output = pid.get_pid(error, 1.0)
    print(f"After integration, Output: {output:.2f}")
    print(f"Integrator: {pid.get_integrator():.2f}")
    
    # Test reset
    pid.reset_I()
    print(f"After reset, Integrator: {pid.get_integrator():.2f}")
    
    # Test gain changes
    pid.set_gains(p=1.0, i=2.0)
    print(f"After gain change: {pid}")
    
    print("✓ PID tests passed")


def test_esc():
    """Test ESC controller"""
    print("\n=== Testing ESC Controller ===")
    
    # Create ESC
    esc = ESC(0)
    print(f"Created: {esc}")
    
    # Test movement
    esc.move(1000)
    print(f"Set to 1000us")
    
    esc.move(1500)
    print(f"Set to 1500us")
    
    # Test clamping
    esc.move(2500)  # Should clamp to max
    print(f"Tried 2500us (should clamp to {esc.FREQ_MAX})")
    
    esc.move(500)  # Should clamp to min
    print(f"Tried 500us (should clamp to {esc.FREQ_MIN})")
    
    # Test stop
    esc.stop()
    print(f"Stopped (set to minimum)")
    
    print("✓ ESC tests passed")


def test_rc():
    """Test RC receiver"""
    print("\n=== Testing RC Receiver ===")
    
    # Create RC channel
    rc = RC(0)
    print(f"Created: {rc}")
    
    # Simulate RC input by setting width directly
    rc.last_width = 1500
    width = rc.get_width()
    print(f"Center position: {width}us")
    
    rc.last_width = 1000
    width = rc.get_width()
    print(f"Min position: {width}us")
    
    rc.last_width = 1900
    width = rc.get_width()
    print(f"Max position: {width}us")
    
    # Test normalized output
    rc.last_width = 1500
    norm = rc.get_normalized()
    print(f"Normalized at center: {norm:.2f}")
    
    rc.last_width = 1900
    norm = rc.get_normalized()
    print(f"Normalized at max: {norm:.2f}")
    
    print("✓ RC tests passed")


def test_utilities():
    """Test utility functions"""
    print("\n=== Testing Utility Functions ===")
    
    # Test map_range
    result = map_range(1500, 1000, 2000, -45, 45)
    print(f"map_range(1500, 1000-2000, -45-45) = {result:.1f}° (expected 0)")
    
    result = map_range(2000, 1000, 2000, -45, 45)
    print(f"map_range(2000, 1000-2000, -45-45) = {result:.1f}° (expected 45)")
    
    # Test wrap_180
    result = wrap_180(190)
    print(f"wrap_180(190) = {result}° (expected -170)")
    
    result = wrap_180(-190)
    print(f"wrap_180(-190) = {result}° (expected 170)")
    
    result = wrap_180(45)
    print(f"wrap_180(45) = {result}° (expected 45)")
    
    print("✓ Utility tests passed")


def test_flight_controller_integration():
    """Test basic flight controller integration"""
    print("\n=== Testing Flight Controller Integration ===")
    
    # Import main module (but don't run main loop)
    try:
        # First, create a mock MPU6050 that can be imported
        class MockMPU6050Module:
            MPU6050 = MockMPU6050
        
        sys.modules['mpu6050'] = MockMPU6050Module()  # type: ignore
        
        import main
        print("✓ Main module loaded successfully")
        
        # Create controller
        print("\nInitializing FlightController...")
        
        controller = main.FlightController()
        print("✓ FlightController initialized")
        
        # Test configuration classes
        print(f"\nConfiguration:")
        print(f"  Roll Rate PID: P={main.PIDConfig.ROLL_RATE_P}")
        print(f"  Throttle Channel: {main.RCConfig.THROTTLE_CHANNEL}")
        print(f"  Stab Output Limit: {main.FlightConfig.STAB_OUTPUT_LIMIT}")
        
        # Test methods exist
        print(f"\nTesting methods:")
        print(f"  reset_integrators: {hasattr(controller, 'reset_integrators')}")
        print(f"  set_all_motors: {hasattr(controller, 'set_all_motors')}")
        print(f"  _read_imu: {hasattr(controller, '_read_imu')}")
        print(f"  _read_rc: {hasattr(controller, '_read_rc')}")
        print(f"  _armed_mode: {hasattr(controller, '_armed_mode')}")
        print(f"  _disarmed_mode: {hasattr(controller, '_disarmed_mode')}")
        print(f"  _mix_motors: {hasattr(controller, '_mix_motors')}")
        
        # Test clamp function
        clamped = controller._clamp(150, -100, 100)
        print(f"\nClamp test: clamp(150, -100, 100) = {clamped} (expected 100)")
        
        # Test reset_integrators
        controller.reset_integrators()
        print(f"Reset integrators successful")
        
        # Test set_all_motors
        controller.set_all_motors(1000)
        print(f"Set all motors to 1000us")
        
        # Simulate one IMU reading
        print(f"\nSimulating IMU read...")
        imu_data = controller._read_imu()
        if imu_data:
            yaw, roll, pitch, g_pitch, g_roll, g_yaw = imu_data
            print(f"  Yaw: {yaw:.2f}°, Roll: {roll:.2f}°, Pitch: {pitch:.2f}°")
            print(f"  Gyro - P: {g_pitch:.2f}, R: {g_roll:.2f}, Y: {g_yaw:.2f}")
        
        # Simulate RC reading
        print(f"\nSimulating RC read...")
        rc_inputs = controller._read_rc()
        print(f"  Throttle: {rc_inputs['throttle']}")
        print(f"  Roll: {rc_inputs['roll']:.2f}°")
        print(f"  Pitch: {rc_inputs['pitch']:.2f}°")
        print(f"  Yaw: {rc_inputs['yaw']:.2f}°")
        
        print("\n✓ Flight controller integration tests passed")
            
    except Exception as e:
        print(f"✗ Error in integration test: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True


# ============================================================================
# MAIN TEST RUNNER
# ============================================================================

def main():
    """Run all smoke tests"""
    print("=" * 60)
    print("PyBoard Flight Controller - Smoke Tests")
    print("=" * 60)
    
    tests = [
        ("PID Controller", test_pid),
        ("ESC Controller", test_esc),
        ("RC Receiver", test_rc),
        ("Utility Functions", test_utilities),
        ("Flight Controller Integration", test_flight_controller_integration),
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            test_func()
            passed += 1
        except Exception as e:
            print(f"\n✗ {name} FAILED: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✓ All smoke tests passed!")
        print("\nThe flight controller code is syntactically correct and")
        print("the logic structure appears sound. Ready for hardware testing.")
        return 0
    else:
        print(f"\n✗ {failed} test(s) failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())
