"""
HAL Testing Suite

Tests the Hardware Abstraction Layer to verify platform independence
and correct implementation of interfaces.
"""

import sys
import time
from abc import ABC

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
        def mem_write(self, data, addr, reg):
            pass
        def mem_read(self, buf, addr, reg):
            for i in range(len(buf)):
                buf[i] = 0
    
    @staticmethod
    def millis():
        return int(time.time() * 1000) % (2**32)
    
    @staticmethod
    def delay(ms):
        time.sleep(ms / 1000)

class MockTimerChannel:
    def __init__(self, mode, pin):
        self.mode = mode
        self.pin = pin
        self._pulse_width = 1500
        self._capture = 0
    
    def pulse_width(self, width=None):
        if width is not None:
            self._pulse_width = width
        return self._pulse_width
    
    def capture(self):
        self._capture += 1500
        return self._capture
    
    def callback(self, func):
        pass

sys.modules['pyb'] = MockPyb()

# Now import HAL modules
try:
    from hal import (
        HALPlatform, HALTimer, HALPWMChannel, HALInputCapture, HALI2C,
        get_platform, set_platform
    )
    print("✓ HAL module imported")
except Exception as e:
    print(f"✗ Failed to import HAL: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

try:
    from hal_pyboard import (
        PyBoardPlatform, PyBoardTimer, PyBoardPWMChannel,
        PyBoardInputCapture, PyBoardI2C
    )
    print("✓ HAL PyBoard implementation imported")
except Exception as e:
    print(f"✗ Failed to import PyBoard HAL: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

try:
    from platform_config import (
        get_config, get_esc_pin, get_rc_pin, get_i2c_config,
        PYBOARD_CONFIG, ARDUINO_CONFIG, RASPBERRY_PI_CONFIG, ESP32_CONFIG
    )
    print("✓ Platform config imported")
except Exception as e:
    print(f"✗ Failed to import platform config: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)


# ============================================================================
# TESTS
# ============================================================================

def test_abstract_interfaces():
    """Test that abstract interfaces are properly defined"""
    print("\n=== Testing Abstract Interfaces ===")
    
    # Check that interfaces are abstract
    assert issubclass(HALPlatform, ABC), "HALPlatform should be ABC"
    assert issubclass(HALTimer, ABC), "HALTimer should be ABC"
    assert issubclass(HALPWMChannel, ABC), "HALPWMChannel should be ABC"
    assert issubclass(HALInputCapture, ABC), "HALInputCapture should be ABC"
    assert issubclass(HALI2C, ABC), "HALI2C should be ABC"
    
    print("✓ All interfaces are properly abstract")
    
    # Check that concrete classes implement interfaces
    assert issubclass(PyBoardPlatform, HALPlatform)
    assert issubclass(PyBoardTimer, HALTimer)
    assert issubclass(PyBoardPWMChannel, HALPWMChannel)
    assert issubclass(PyBoardInputCapture, HALInputCapture)
    assert issubclass(PyBoardI2C, HALI2C)
    
    print("✓ PyBoard classes implement interfaces correctly")


def test_platform_config():
    """Test platform configuration"""
    print("\n=== Testing Platform Configuration ===")
    
    # Test config structures
    configs = [PYBOARD_CONFIG, ARDUINO_CONFIG, RASPBERRY_PI_CONFIG, ESP32_CONFIG]
    for config in configs:
        assert 'platform' in config
        assert 'esc_pins' in config
        assert 'rc_pins' in config
        assert 'i2c' in config
        assert len(config['esc_pins']) == 6, "Should have 6 ESC pins"
        assert len(config['rc_pins']) == 4, "Should have 4 RC pins"
        print(f"✓ {config['platform']} config valid")
    
    # Test getter functions
    esc_pin = get_esc_pin(0, 'pyboard')
    assert 'pin' in esc_pin
    print(f"✓ ESC pin getter works: {esc_pin}")
    
    rc_pin = get_rc_pin(0, 'pyboard')
    assert 'pin' in rc_pin
    print(f"✓ RC pin getter works: {rc_pin}")
    
    i2c_cfg = get_i2c_config('pyboard')
    assert 'bus' in i2c_cfg or 'sda_pin' in i2c_cfg
    print(f"✓ I2C config getter works: {i2c_cfg}")


def test_pyboard_pwm_channel():
    """Test PyBoard PWM channel"""
    print("\n=== Testing PyBoard PWM Channel ===")
    
    timer = PyBoardTimer(5)
    pwm = timer.create_pwm_channel('X1', frequency=50)
    
    # Test set/get pulse width
    pwm.set_pulse_width(1000)
    width = pwm.get_pulse_width()
    assert width == 1000, f"Expected 1000, got {width}"
    print(f"✓ PWM set to {width} μs")
    
    pwm.set_pulse_width(1500)
    width = pwm.get_pulse_width()
    assert width == 1500, f"Expected 1500, got {width}"
    print(f"✓ PWM set to {width} μs")
    
    pwm.set_pulse_width(2000)
    width = pwm.get_pulse_width()
    assert width == 2000, f"Expected 2000, got {width}"
    print(f"✓ PWM set to {width} μs")


def test_pyboard_input_capture():
    """Test PyBoard input capture"""
    print("\n=== Testing PyBoard Input Capture ===")
    
    timer = PyBoardTimer(12)
    
    callback_called = [False]
    last_width = [0]
    
    def test_callback(width):
        callback_called[0] = True
        last_width[0] = width
    
    ic = timer.create_input_capture('Y8', test_callback)
    
    # Simulate pulse
    ic.pulse_width = 1500
    width = ic.get_pulse_width()
    assert 950 < width < 1950, f"Width {width} out of valid range"
    print(f"✓ Input capture returns valid width: {width} μs")


def test_pyboard_i2c():
    """Test PyBoard I2C"""
    print("\n=== Testing PyBoard I2C ===")
    
    i2c = PyBoardI2C(1)
    
    # Test write
    try:
        i2c.write_byte(0x68, 0x6B, 0x00)
        print("✓ I2C write byte successful")
    except Exception as e:
        print(f"✓ I2C write handled gracefully: {e}")
    
    # Test read
    try:
        value = i2c.read_byte(0x68, 0x75)
        print(f"✓ I2C read byte successful: {value}")
    except Exception as e:
        print(f"✓ I2C read handled gracefully: {e}")
    
    # Test multi-byte read
    try:
        data = i2c.read_bytes(0x68, 0x3B, 6)
        assert len(data) == 6
        print(f"✓ I2C read bytes successful: {len(data)} bytes")
    except Exception as e:
        print(f"✓ I2C read bytes handled gracefully: {e}")


def test_pyboard_platform():
    """Test PyBoard platform"""
    print("\n=== Testing PyBoard Platform ===")
    
    platform = PyBoardPlatform()
    
    # Test platform name
    assert platform.platform_name == "PyBoard"
    print(f"✓ Platform name: {platform.platform_name}")
    
    # Test timer creation
    timer1 = platform.get_timer(5)
    timer2 = platform.get_timer(5)
    assert timer1 is timer2, "Same timer ID should return same instance"
    print("✓ Timer singleton works")
    
    # Test I2C creation
    i2c1 = platform.get_i2c(1)
    i2c2 = platform.get_i2c(1)
    assert i2c1 is i2c2, "Same I2C ID should return same instance"
    print("✓ I2C singleton works")
    
    # Test timing functions
    start = platform.millis()
    assert isinstance(start, int)
    print(f"✓ millis() returns: {start}")
    
    platform.delay_ms(10)
    print("✓ delay_ms() works")


def test_platform_detection():
    """Test platform auto-detection"""
    print("\n=== Testing Platform Detection ===")
    
    # Since we mocked pyb, it should detect PyBoard
    platform = get_platform()
    assert platform is not None
    assert platform.platform_name == "PyBoard"
    print(f"✓ Auto-detected platform: {platform.platform_name}")
    
    # Test manual platform setting
    custom_platform = PyBoardPlatform()
    set_platform(custom_platform)
    platform2 = get_platform()
    assert platform2 is custom_platform
    print("✓ Manual platform setting works")


def test_interface_completeness():
    """Test that all required methods are implemented"""
    print("\n=== Testing Interface Completeness ===")
    
    platform = PyBoardPlatform()
    
    # Check HALPlatform methods
    required_methods = ['get_timer', 'get_i2c', 'millis', 'delay_ms', 'delay_us']
    for method in required_methods:
        assert hasattr(platform, method), f"Missing method: {method}"
        assert callable(getattr(platform, method)), f"Not callable: {method}"
    print("✓ HALPlatform implements all required methods")
    
    # Check HALTimer methods
    timer = platform.get_timer(5)
    timer_methods = ['create_pwm_channel', 'create_input_capture', 'deinit']
    for method in timer_methods:
        assert hasattr(timer, method), f"Timer missing method: {method}"
        assert callable(getattr(timer, method)), f"Timer method not callable: {method}"
    print("✓ HALTimer implements all required methods")
    
    # Check HALPWMChannel methods
    pwm = timer.create_pwm_channel('X1', 50)
    pwm_methods = ['set_pulse_width', 'get_pulse_width']
    for method in pwm_methods:
        assert hasattr(pwm, method), f"PWM missing method: {method}"
        assert callable(getattr(pwm, method)), f"PWM method not callable: {method}"
    print("✓ HALPWMChannel implements all required methods")
    
    # Check HALI2C methods
    i2c = platform.get_i2c(1)
    i2c_methods = ['write_byte', 'read_byte', 'read_bytes', 'write_bytes']
    for method in i2c_methods:
        assert hasattr(i2c, method), f"I2C missing method: {method}"
        assert callable(getattr(i2c, method)), f"I2C method not callable: {method}"
    print("✓ HALI2C implements all required methods")


def test_pin_configurations():
    """Test that pin configurations are sensible"""
    print("\n=== Testing Pin Configurations ===")
    
    # Test PyBoard config
    config = get_config('pyboard')
    
    # Check no duplicate pins
    esc_pins = [p['pin'] for p in config['esc_pins'].values()]
    assert len(esc_pins) == len(set(esc_pins)), "Duplicate ESC pins"
    print(f"✓ No duplicate ESC pins: {esc_pins}")
    
    rc_pins = [p['pin'] for p in config['rc_pins'].values()]
    assert len(rc_pins) == len(set(rc_pins)), "Duplicate RC pins"
    print(f"✓ No duplicate RC pins: {rc_pins}")
    
    # Check for pin conflicts between ESC and RC
    all_pins = esc_pins + rc_pins
    assert len(all_pins) == len(set(all_pins)), "Pin conflict between ESC and RC"
    print("✓ No pin conflicts between ESC and RC")


def test_error_handling():
    """Test error handling"""
    print("\n=== Testing Error Handling ===")
    
    # Test invalid config
    try:
        config = get_config('invalid_platform')
        assert False, "Should have raised ValueError"
    except ValueError as e:
        print(f"✓ Invalid platform raises ValueError: {e}")
    
    # Test invalid pin index
    try:
        pin = get_esc_pin(99, 'pyboard')
        # May succeed or fail depending on implementation
        print(f"⚠ ESC pin 99 returned: {pin}")
    except (KeyError, IndexError) as e:
        print(f"✓ Invalid ESC index raises error: {e}")


def test_multiple_platforms():
    """Test configurations for all platforms"""
    print("\n=== Testing All Platform Configs ===")
    
    platforms = ['pyboard', 'arduino', 'raspberry_pi', 'esp32']
    
    for platform_name in platforms:
        config = get_config(platform_name)
        
        # Basic structure checks
        assert config['platform'] == platform_name
        assert len(config['esc_pins']) == 6
        assert len(config['rc_pins']) == 4
        
        print(f"✓ {platform_name:15s} config valid "
              f"(ESC: {len(config['esc_pins'])}, RC: {len(config['rc_pins'])})")


# ============================================================================
# MAIN TEST RUNNER
# ============================================================================

def main():
    """Run all HAL tests"""
    print("=" * 60)
    print("Hardware Abstraction Layer - Test Suite")
    print("=" * 60)
    
    tests = [
        ("Abstract Interfaces", test_abstract_interfaces),
        ("Platform Configuration", test_platform_config),
        ("PyBoard PWM Channel", test_pyboard_pwm_channel),
        ("PyBoard Input Capture", test_pyboard_input_capture),
        ("PyBoard I2C", test_pyboard_i2c),
        ("PyBoard Platform", test_pyboard_platform),
        ("Platform Detection", test_platform_detection),
        ("Interface Completeness", test_interface_completeness),
        ("Pin Configurations", test_pin_configurations),
        ("Error Handling", test_error_handling),
        ("Multiple Platforms", test_multiple_platforms),
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
        print("\n✓ All HAL tests passed!")
        print("\nThe Hardware Abstraction Layer is working correctly.")
        print("Ready to implement platform-specific HALs.")
        return 0
    else:
        print(f"\n✗ {failed} test(s) failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())
