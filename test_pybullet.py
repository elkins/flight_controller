"""
PyBullet Integration Tests

Tests flight controller with PyBullet physics simulation.
"""

import sys
import time
import numpy as np
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s - %(name)s - %(message)s'
)

# Import HAL
from hal import set_platform

# Check if PyBullet is available
try:
    import hal_pybullet
    from hal_pybullet import PyBulletPlatform
    PYBULLET_AVAILABLE = True
except ImportError as e:
    print(f"PyBullet not available: {e}")
    print("Install with: pip install pybullet")
    PYBULLET_AVAILABLE = False
    sys.exit(1)


def test_pybullet_platform():
    """Test PyBullet platform initialization"""
    print("\n=== Testing PyBullet Platform ===")
    
    # Create platform (headless for testing)
    platform = PyBulletPlatform(gui=False, start_position=[0, 0, 1.0])
    set_platform(platform)
    
    print(f"✓ Platform initialized: {platform.platform_name}")
    print(f"✓ Simulation time: {platform.millis()}ms")
    
    # Get state
    state = platform.get_simulation_state()
    print(f"✓ Initial position: {state['position']}")
    print(f"✓ Initial orientation (roll, pitch, yaw): {np.degrees(state['orientation_euler'])}")
    
    # Step simulation
    for _ in range(10):
        platform.step_simulation()
    
    print(f"✓ Simulation stepped 10 times")
    print(f"✓ New position: {platform.get_simulation_state()['position']}")
    
    platform.cleanup()
    return True


def test_pybullet_motors():
    """Test motor control"""
    print("\n=== Testing Motor Control ===")
    
    platform = PyBulletPlatform(gui=False)
    set_platform(platform)
    
    # Create motor PWM channels
    timer = platform.get_timer(5)
    motors = []
    for i in range(6):
        pin = f'X{i+1}'
        pwm = timer.create_pwm_channel(pin, 50)
        motors.append(pwm)
        print(f"✓ Created motor {i} on pin {pin}")
    
    # Test setting motor speeds
    print("\nTesting motor speeds...")
    
    # All motors at idle
    for i, motor in enumerate(motors):
        motor.set_pulse_width(1000)
    platform.step_simulation()
    print("✓ All motors at idle (1000μs)")
    
    # All motors at half throttle
    for motor in enumerate(motors):
        motor[1].set_pulse_width(1500)
    for _ in range(10):
        platform.step_simulation()
    state = platform.get_simulation_state()
    print(f"✓ All motors at 50% (1500μs), altitude: {state['position'][2]:.3f}m")
    
    # All motors at full throttle
    for motor in motors:
        motor.set_pulse_width(2000)
    for _ in range(100):
        platform.step_simulation()
    state = platform.get_simulation_state()
    print(f"✓ All motors at 100% (2000μs), altitude: {state['position'][2]:.3f}m")
    print(f"  Velocity: {state['velocity'][2]:.3f} m/s (upward)")
    
    platform.cleanup()
    return True


def test_pybullet_imu():
    """Test IMU sensor"""
    print("\n=== Testing IMU Sensor ===")
    
    platform = PyBulletPlatform(gui=False)
    set_platform(platform)
    
    # Get I2C bus for IMU
    i2c = platform.get_i2c(1)
    
    # Initialize IMU (MPU6050)
    i2c.write_byte(0x68, 0x6B, 0x00)  # Wake up
    who_am_i = i2c.read_byte(0x68, 0x75)
    print(f"✓ MPU6050 WHO_AM_I: 0x{who_am_i:02X}")
    
    # Read accelerometer
    accel_data = i2c.read_bytes(0x68, 0x3B, 6)
    accel_x = int.from_bytes(accel_data[0:2], 'big', signed=True)
    accel_y = int.from_bytes(accel_data[2:4], 'big', signed=True)
    accel_z = int.from_bytes(accel_data[4:6], 'big', signed=True)
    
    # Convert to g's (2048 LSB/g)
    accel_x_g = accel_x / 2048.0
    accel_y_g = accel_y / 2048.0
    accel_z_g = accel_z / 2048.0
    
    print(f"✓ Accelerometer (g): X={accel_x_g:.3f}, Y={accel_y_g:.3f}, Z={accel_z_g:.3f}")
    print(f"  (Should read ~0, 0, -1 when level)")
    
    # Read gyroscope
    gyro_data = i2c.read_bytes(0x68, 0x43, 6)
    gyro_x = int.from_bytes(gyro_data[0:2], 'big', signed=True)
    gyro_y = int.from_bytes(gyro_data[2:4], 'big', signed=True)
    gyro_z = int.from_bytes(gyro_data[4:6], 'big', signed=True)
    
    # Convert to deg/s (131 LSB per deg/s)
    gyro_x_dps = gyro_x / 131.0
    gyro_y_dps = gyro_y / 131.0
    gyro_z_dps = gyro_z / 131.0
    
    print(f"✓ Gyroscope (°/s): X={gyro_x_dps:.3f}, Y={gyro_y_dps:.3f}, Z={gyro_z_dps:.3f}")
    print(f"  (Should read ~0, 0, 0 when stationary)")
    
    platform.cleanup()
    return True


def test_pybullet_rc_input():
    """Test RC input"""
    print("\n=== Testing RC Input ===")
    
    platform = PyBulletPlatform(gui=False)
    set_platform(platform)
    
    # Create RC input channels
    timer = platform.get_timer(12)
    rc_channels = []
    channel_names = ['Throttle', 'Roll', 'Pitch', 'Yaw']
    
    for i in range(4):
        pin = f'Y{i+8}'
        ic = timer.create_input_capture(pin, lambda w: None)
        rc_channels.append(ic)
        print(f"✓ Created RC channel {i} ({channel_names[i]}) on pin {pin}")
    
    # Test setting RC inputs
    print("\nTesting RC input values...")
    
    # Set throttle to 50%
    rc_channels[0].set_simulated_input(1500)
    print(f"✓ Throttle set to 1500μs: {rc_channels[0].get_pulse_width()}μs")
    
    # Set roll to max right
    rc_channels[1].set_simulated_input(2000)
    print(f"✓ Roll set to 2000μs (right): {rc_channels[1].get_pulse_width()}μs")
    
    # Set pitch to max forward
    rc_channels[2].set_simulated_input(2000)
    print(f"✓ Pitch set to 2000μs (forward): {rc_channels[2].get_pulse_width()}μs")
    
    # Set yaw to center
    rc_channels[3].set_simulated_input(1500)
    print(f"✓ Yaw set to 1500μs (center): {rc_channels[3].get_pulse_width()}μs")
    
    platform.cleanup()
    return True


def test_pybullet_flight_simulation():
    """Test complete flight simulation"""
    print("\n=== Testing Flight Simulation ===")
    
    # Create platform with GUI
    print("Creating simulation (GUI=True, this will show a window)...")
    platform = PyBulletPlatform(gui=True, start_position=[0, 0, 2.0])
    set_platform(platform)
    
    # Create motor controls
    timer = platform.get_timer(5)
    motors = [timer.create_pwm_channel(f'X{i+1}', 50) for i in range(6)]
    
    print("\nRunning 5-second flight simulation...")
    print("Watch the PyBullet window!")
    
    # Hover test: gradually increase throttle to hover
    steps_per_second = 240  # 240Hz physics
    total_seconds = 5
    
    for second in range(total_seconds):
        print(f"\n--- Second {second+1} ---")
        
        if second == 0:
            # Ramp up to hover
            for step in range(steps_per_second):
                throttle = 1000 + int((step / steps_per_second) * 600)  # 1000 -> 1600
                for motor in motors:
                    motor.set_pulse_width(throttle)
                platform.step_simulation()
        
        elif second == 1:
            # Maintain hover
            for _ in range(steps_per_second):
                for motor in motors:
                    motor.set_pulse_width(1550)  # Adjust for hover
                platform.step_simulation()
        
        elif second == 2:
            # Tilt forward (more thrust on rear motors)
            for _ in range(steps_per_second):
                motors[0].set_pulse_width(1500)  # Front
                motors[1].set_pulse_width(1500)  # Front-right
                motors[2].set_pulse_width(1600)  # Rear-right
                motors[3].set_pulse_width(1600)  # Rear
                motors[4].set_pulse_width(1600)  # Rear-left
                motors[5].set_pulse_width(1500)  # Front-left
                platform.step_simulation()
        
        elif second == 3:
            # Return to level
            for _ in range(steps_per_second):
                for motor in motors:
                    motor.set_pulse_width(1550)
                platform.step_simulation()
        
        else:
            # Descend
            for step in range(steps_per_second):
                throttle = 1550 - int((step / steps_per_second) * 500)
                for motor in motors:
                    motor.set_pulse_width(throttle)
                platform.step_simulation()
        
        # Print state
        state = platform.get_simulation_state()
        pos = state['position']
        vel = state['velocity']
        euler = np.degrees(state['orientation_euler'])
        print(f"  Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"  Velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]")
        print(f"  Attitude (deg): Roll={euler[0]:.1f}, Pitch={euler[1]:.1f}, Yaw={euler[2]:.1f}")
    
    print("\n✓ Flight simulation complete!")
    print("  Close the PyBullet window when ready...")
    time.sleep(2)
    
    platform.cleanup()
    return True


def main():
    """Run all PyBullet tests"""
    print("=" * 60)
    print("PyBullet Integration Test Suite")
    print("=" * 60)
    
    if not PYBULLET_AVAILABLE:
        print("✗ PyBullet not available")
        return 1
    
    tests = [
        ("Platform Initialization", test_pybullet_platform),
        ("Motor Control", test_pybullet_motors),
        ("IMU Sensor", test_pybullet_imu),
        ("RC Input", test_pybullet_rc_input),
        ("Flight Simulation", test_pybullet_flight_simulation),
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            print()
            if test_func():
                passed += 1
                print(f"✓ {name} PASSED")
            else:
                failed += 1
                print(f"✗ {name} FAILED")
        except Exception as e:
            failed += 1
            print(f"✗ {name} FAILED: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✓ All PyBullet tests passed!")
        print("\nYou can now:")
        print("  • Run flight controller in simulation")
        print("  • Test PID tuning safely")
        print("  • Validate control algorithms")
        print("  • Record and analyze flight data")
        return 0
    else:
        print(f"\n✗ {failed} test(s) failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())
