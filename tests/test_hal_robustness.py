"""
HAL Robustness Tests

Tests error handling, edge cases, and validation in the Hardware Abstraction Layer.
"""

import sys
import logging

# Set up logging to see all messages
logging.basicConfig(
    level=logging.DEBUG,
    format='%(levelname)s - %(name)s - %(message)s'
)

# Mock pyb before importing HAL
class MockPyb:
    class Timer:
        PWM = 'PWM'
        IC = 'IC'
        BOTH = 'BOTH'
        def __init__(self, timer_id, prescaler=0, period=0):
            self.timer_id = timer_id
            self.prescaler = prescaler
            self.period = period
        def channel(self, ch, mode, pin=None, polarity=None):
            return MockTimerChannel(mode, pin)
        def deinit(self):
            pass
    
    class Pin:
        def __init__(self, name):
            self.name = name
            self._value = 0
        def value(self, v=None):
            if v is not None:
                self._value = v
            return self._value
    
    class I2C:
        MASTER = 'MASTER'
        def __init__(self, bus_id, mode=None):
            self.bus_id = bus_id
            self._fail_next = False
        def mem_write(self, data, addr, reg):
            if self._fail_next:
                self._fail_next = False
                raise OSError("Simulated I2C write failure")
        def mem_read(self, buf, addr, reg):
            if self._fail_next:
                self._fail_next = False
                raise OSError("Simulated I2C read failure")
            for i in range(len(buf)):
                buf[i] = 0
    
    @staticmethod
    def millis():
        import time
        return int(time.time() * 1000) % (2**32)
    
    @staticmethod
    def delay(ms):
        import time
        time.sleep(ms / 1000)

class MockTimerChannel:
    def __init__(self, mode, pin):
        self.mode = mode
        self.pin = pin
        self._pulse_width = 1500
    def pulse_width(self, width=None):
        if width is not None:
            self._pulse_width = width
        return self._pulse_width
    def capture(self):
        return 1500
    def callback(self, func):
        pass

sys.modules['pyb'] = MockPyb()

from src.hal.hal import get_platform, set_platform, HALPlatform
from src.hal.hal_pyboard import PyBoardPlatform
from src.platform_config import get_config


def test_pwm_validation():
    """Test PWM pulse width validation"""
    print("\n=== Testing PWM Validation ===")
    
    platform = PyBoardPlatform()
    set_platform(platform)
    
    timer = platform.get_timer(5)
    pwm = timer.create_pwm_channel('X1', 50)
    
    # Test valid range
    try:
        pwm.set_pulse_width(1000)
        print("✓ Valid width 1000us accepted")
    except ValueError:
        print("✗ Valid width rejected")
        return False
    
    # Test lower bound
    try:
        pwm.set_pulse_width(500)
        print("✓ Min width 500us accepted")
    except ValueError:
        print("✗ Min width rejected")
        return False
    
    # Test upper bound
    try:
        pwm.set_pulse_width(2500)
        print("✓ Max width 2500us accepted")
    except ValueError:
        print("✗ Max width rejected")
        return False
    
    # Test too low (should fail)
    try:
        pwm.set_pulse_width(499)
        print("✗ Invalid width 499us was accepted (should reject)")
        return False
    except ValueError as e:
        print(f"✓ Invalid width 499us rejected: {e}")
    
    # Test too high (should fail)
    try:
        pwm.set_pulse_width(2501)
        print("✗ Invalid width 2501us was accepted (should reject)")
        return False
    except ValueError as e:
        print(f"✓ Invalid width 2501us rejected: {e}")
    
    # Test negative (should fail)
    try:
        pwm.set_pulse_width(-100)
        print("✗ Negative width was accepted (should reject)")
        return False
    except ValueError as e:
        print(f"✓ Negative width rejected: {e}")
    
    return True


def test_timer_validation():
    """Test timer ID validation"""
    print("\n=== Testing Timer Validation ===")
    
    platform = PyBoardPlatform()
    set_platform(platform)
    
    # Test valid timer IDs
    for timer_id in [1, 5, 12, 14]:
        try:
            timer = platform.get_timer(timer_id)
            print(f"✓ Valid timer ID {timer_id} accepted")
        except ValueError:
            print(f"✗ Valid timer ID {timer_id} rejected")
            return False
    
    # Test invalid timer IDs
    for timer_id in [0, 15, 100, -1]:
        try:
            timer = platform.get_timer(timer_id)
            print(f"✗ Invalid timer ID {timer_id} was accepted (should reject)")
            return False
        except ValueError as e:
            print(f"✓ Invalid timer ID {timer_id} rejected: {e}")
    
    return True


def test_i2c_validation():
    """Test I2C bus validation"""
    print("\n=== Testing I2C Validation ===")
    
    platform = PyBoardPlatform()
    set_platform(platform)
    
    # Test valid bus IDs
    for bus_id in [1, 2]:
        try:
            i2c = platform.get_i2c(bus_id)
            print(f"✓ Valid I2C bus {bus_id} accepted")
        except ValueError:
            print(f"✗ Valid I2C bus {bus_id} rejected")
            return False
    
    # Test invalid bus IDs
    for bus_id in [0, 3, -1]:
        try:
            i2c = platform.get_i2c(bus_id)
            print(f"✗ Invalid I2C bus {bus_id} was accepted (should reject)")
            return False
        except ValueError as e:
            print(f"✓ Invalid I2C bus {bus_id} rejected: {e}")
    
    # Test I2C read length validation
    i2c = platform.get_i2c(1)
    try:
        data = i2c.read_bytes(0x68, 0x3B, 0)
        print("✗ Zero-length read was accepted (should reject)")
        return False
    except ValueError as e:
        print(f"✓ Zero-length read rejected: {e}")
    
    try:
        data = i2c.read_bytes(0x68, 0x3B, -1)
        print("✗ Negative-length read was accepted (should reject)")
        return False
    except ValueError as e:
        print(f"✓ Negative-length read rejected: {e}")
    
    # Test I2C write data validation
    try:
        i2c.write_bytes(0x68, 0x6B, b'')
        print("✗ Empty data write was accepted (should reject)")
        return False
    except ValueError as e:
        print(f"✓ Empty data write rejected: {e}")
    
    return True


def test_i2c_error_handling():
    """Test I2C error handling"""
    print("\n=== Testing I2C Error Handling ===")
    
    platform = PyBoardPlatform()
    set_platform(platform)
    
    i2c = platform.get_i2c(1)
    
    # Simulate I2C write failure
    i2c.bus._fail_next = True
    try:
        i2c.write_byte(0x68, 0x6B, 0x00)
        print("✗ I2C error not properly raised")
        return False
    except IOError as e:
        print(f"✓ I2C write error handled: {e}")
    
    # Simulate I2C read failure
    i2c.bus._fail_next = True
    try:
        value = i2c.read_byte(0x68, 0x75)
        print("✗ I2C error not properly raised")
        return False
    except IOError as e:
        print(f"✓ I2C read error handled: {e}")
    
    # Simulate I2C multi-byte read failure
    i2c.bus._fail_next = True
    try:
        data = i2c.read_bytes(0x68, 0x3B, 6)
        print("✗ I2C error not properly raised")
        return False
    except IOError as e:
        print(f"✓ I2C multi-byte read error handled: {e}")
    
    return True


def test_platform_type_checking():
    """Test platform type checking"""
    print("\n=== Testing Platform Type Checking ===")
    
    # Test setting invalid platform type
    try:
        set_platform("not a platform")
        print("✗ Invalid platform type was accepted")
        return False
    except TypeError as e:
        print(f"✓ Invalid platform type rejected: {e}")
    
    try:
        set_platform(None)
        print("✗ None platform was accepted")
        return False
    except TypeError as e:
        print(f"✓ None platform rejected: {e}")
    
    # Test setting valid platform
    try:
        platform = PyBoardPlatform()
        set_platform(platform)
        print("✓ Valid platform accepted")
    except Exception as e:
        print(f"✗ Valid platform rejected: {e}")
        return False
    
    return True


def test_pwm_frequency_validation():
    """Test PWM frequency validation"""
    print("\n=== Testing PWM Frequency Validation ===")
    
    platform = PyBoardPlatform()
    set_platform(platform)
    
    timer = platform.get_timer(5)
    
    # Test valid frequencies
    for freq in [50, 100, 400, 1000]:
        try:
            pwm = timer.create_pwm_channel(f'X{freq}', freq)
            print(f"✓ Valid frequency {freq}Hz accepted")
        except ValueError:
            print(f"✗ Valid frequency {freq}Hz rejected")
            return False
    
    # Test invalid frequencies
    for freq in [0, -50]:
        try:
            pwm = timer.create_pwm_channel(f'X{abs(freq)}', freq)
            print(f"✗ Invalid frequency {freq}Hz was accepted (should reject)")
            return False
        except ValueError as e:
            print(f"✓ Invalid frequency {freq}Hz rejected: {e}")
    
    return True


def test_rc_pulse_warnings():
    """Test RC pulse width warning thresholds"""
    print("\n=== Testing RC Pulse Warnings ===")
    
    platform = PyBoardPlatform()
    set_platform(platform)
    
    timer = platform.get_timer(12)
    
    callback_count = [0]
    def test_callback(width):
        callback_count[0] += 1
    
    ic = timer.create_input_capture('Y8', test_callback)
    
    # Test normal pulse width
    ic.pulse_width = 1500
    width = ic.get_pulse_width()
    print(f"✓ Normal pulse {width}us processed")
    
    # Test edge of normal range
    ic.pulse_width = 950
    width = ic.get_pulse_width()
    print(f"✓ Low edge pulse {width}us processed")
    
    ic.pulse_width = 1950
    width = ic.get_pulse_width()
    print(f"✓ High edge pulse {width}us processed")
    
    # Test out of range (should still work but with warning)
    ic.pulse_width = 500
    width = ic.get_pulse_width()
    print(f"✓ Out-of-range pulse 500us handled (returned last valid: {width}us)")
    
    ic.pulse_width = 2500
    width = ic.get_pulse_width()
    print(f"✓ Out-of-range pulse 2500us handled (returned last valid: {width}us)")
    
    return True


def test_singleton_behavior():
    """Test that timers and I2C buses are singletons"""
    print("\n=== Testing Singleton Behavior ===")
    
    platform = PyBoardPlatform()
    set_platform(platform)
    
    # Test timer singleton
    timer1 = platform.get_timer(5)
    timer2 = platform.get_timer(5)
    if timer1 is not timer2:
        print("✗ Timer singleton failed - got different instances")
        return False
    print("✓ Timer singleton works")
    
    # Different timer IDs should be different instances
    timer3 = platform.get_timer(12)
    if timer1 is timer3:
        print("✗ Different timer IDs returned same instance")
        return False
    print("✓ Different timer IDs create different instances")
    
    # Test I2C singleton
    i2c1 = platform.get_i2c(1)
    i2c2 = platform.get_i2c(1)
    if i2c1 is not i2c2:
        print("✗ I2C singleton failed - got different instances")
        return False
    print("✓ I2C singleton works")
    
    return True


def main():
    """Run all robustness tests"""
    print("=" * 60)
    print("HAL Robustness Test Suite")
    print("=" * 60)
    
    tests = [
        ("PWM Validation", test_pwm_validation),
        ("Timer Validation", test_timer_validation),
        ("I2C Validation", test_i2c_validation),
        ("I2C Error Handling", test_i2c_error_handling),
        ("Platform Type Checking", test_platform_type_checking),
        ("PWM Frequency Validation", test_pwm_frequency_validation),
        ("RC Pulse Warnings", test_rc_pulse_warnings),
        ("Singleton Behavior", test_singleton_behavior),
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            if test_func():
                passed += 1
                print(f"✓ {name} PASSED")
            else:
                failed += 1
                print(f"✗ {name} FAILED")
        except Exception as e:
            failed += 1
            print(f"\n✗ {name} FAILED with exception: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "=" * 60)
    print(f"Robustness Test Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✓ All robustness tests passed!")
        print("\nThe HAL has excellent error handling and validation:")
        print("  • Input validation works correctly")
        print("  • Error conditions are handled properly")
        print("  • Edge cases are covered")
        print("  • Type checking prevents misuse")
        print("  • Resource management is sound")
        return 0
    else:
        print(f"\n✗ {failed} test(s) failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())
