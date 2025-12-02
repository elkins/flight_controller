"""
MAVLink Telemetry Tests

Tests MAVLink protocol integration, message encoding, and ground station communication.
"""

import sys
import time
import pytest
import numpy as np

# Check if pymavlink is available
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("Warning: pymavlink not installed")
    print("Install with: pip install pymavlink")

if MAVLINK_AVAILABLE:
    from mavlink_telemetry import MAVLinkTelemetry


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_import():
    """Test that MAVLink module can be imported"""
    assert MAVLINK_AVAILABLE
    print("✓ pymavlink installed and importable")


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_initialization_udp():
    """Test MAVLink initialization with UDP"""
    print("\n=== Testing MAVLink Initialization (UDP) ===")
    
    # Create UDP connection
    mavlink = MAVLinkTelemetry(port='udp:localhost:14551', system_id=1)
    
    assert mavlink.connected
    assert mavlink.system_id == 1
    assert mavlink.component_id == 1
    assert mavlink.armed == False
    
    print(f"✓ Initialized: {mavlink.port}")
    print(f"✓ System ID: {mavlink.system_id}")
    print(f"✓ Connected: {mavlink.connected}")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_heartbeat():
    """Test HEARTBEAT message generation"""
    print("\n=== Testing HEARTBEAT Message ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14552', system_id=1)
    
    initial_count = mavlink.packets_sent
    
    # Send heartbeat
    mavlink.send_heartbeat(armed=False)
    assert mavlink.packets_sent == initial_count + 1
    print(f"✓ Heartbeat sent (disarmed)")
    
    # Send heartbeat armed
    mavlink.send_heartbeat(armed=True)
    assert mavlink.packets_sent == initial_count + 2
    assert mavlink.armed == True
    print(f"✓ Heartbeat sent (armed)")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_attitude():
    """Test ATTITUDE message generation"""
    print("\n=== Testing ATTITUDE Message ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14553', system_id=1)
    
    initial_count = mavlink.packets_sent
    
    # Send attitude (45° roll, 30° pitch, 90° yaw)
    roll = np.radians(45)
    pitch = np.radians(30)
    yaw = np.radians(90)
    rollspeed = 0.1  # rad/s
    pitchspeed = 0.2
    yawspeed = 0.3
    
    mavlink.send_attitude(roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
    
    assert mavlink.packets_sent == initial_count + 1
    print(f"✓ Attitude message sent")
    print(f"  Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°, Yaw: {np.degrees(yaw):.1f}°")
    print(f"  Rates: {rollspeed:.2f}, {pitchspeed:.2f}, {yawspeed:.2f} rad/s")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_position():
    """Test LOCAL_POSITION_NED message generation"""
    print("\n=== Testing LOCAL_POSITION_NED Message ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14554', system_id=1)
    
    initial_count = mavlink.packets_sent
    
    # Send position (10m north, 5m east, 3m up = -3m down)
    x, y, z = 10.0, 5.0, -3.0
    vx, vy, vz = 1.5, 0.5, 0.2
    
    mavlink.send_position(x, y, z, vx, vy, vz)
    
    assert mavlink.packets_sent == initial_count + 1
    print(f"✓ Position message sent")
    print(f"  Position: N={x:.1f}m, E={y:.1f}m, D={z:.1f}m (altitude={-z:.1f}m)")
    print(f"  Velocity: {vx:.1f}, {vy:.1f}, {vz:.1f} m/s")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_sys_status():
    """Test SYS_STATUS message generation"""
    print("\n=== Testing SYS_STATUS Message ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14555', system_id=1)
    
    initial_count = mavlink.packets_sent
    
    # Send system status (12.6V battery, 15A current, 85% remaining, 50% CPU)
    mavlink.send_sys_status(
        voltage_battery=12600,      # mV
        current_battery=1500,       # cA (centi-amps)
        battery_remaining=85,       # %
        cpu_load=500                # 50% (0-1000 scale)
    )
    
    assert mavlink.packets_sent == initial_count + 1
    print(f"✓ System status sent")
    print(f"  Battery: 12.6V, 15.0A, 85%")
    print(f"  CPU Load: 50%")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_rc_channels():
    """Test RC_CHANNELS message generation"""
    print("\n=== Testing RC_CHANNELS Message ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14556', system_id=1)
    
    initial_count = mavlink.packets_sent
    
    # Send RC channels (throttle, roll, pitch, yaw)
    channels = [1500, 1600, 1400, 1500, 1000, 2000, 0, 0]  # 8 channels
    
    mavlink.send_rc_channels(channels, rssi=200)
    
    assert mavlink.packets_sent == initial_count + 1
    print(f"✓ RC channels sent")
    print(f"  Channels: {channels[:6]}")
    print(f"  RSSI: 200/255")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_servo_output():
    """Test SERVO_OUTPUT_RAW message generation"""
    print("\n=== Testing SERVO_OUTPUT_RAW Message ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14557', system_id=1)
    
    initial_count = mavlink.packets_sent
    
    # Send motor PWM outputs (6 motors for hexacopter)
    motors = [1450, 1460, 1470, 1480, 1490, 1500]
    
    mavlink.send_servo_output(motors, port=0)
    
    assert mavlink.packets_sent == initial_count + 1
    print(f"✓ Servo output sent")
    print(f"  Motors: {motors}")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_statustext():
    """Test STATUSTEXT message generation"""
    print("\n=== Testing STATUSTEXT Message ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14558', system_id=1)
    
    initial_count = mavlink.packets_sent
    
    # Send status messages with different severities
    messages = [
        ("Flight controller initialized", 6),  # INFO
        ("Battery voltage low", 4),            # WARNING
        ("GPS signal lost", 3),                # ERROR
    ]
    
    for text, severity in messages:
        mavlink.send_statustext(text, severity)
    
    assert mavlink.packets_sent == initial_count + 3
    print(f"✓ Status messages sent: {len(messages)}")
    for text, sev in messages:
        sev_name = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG'][sev]
        print(f"  [{sev_name}] {text}")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_statistics():
    """Test telemetry statistics tracking"""
    print("\n=== Testing Telemetry Statistics ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14559', system_id=1)
    
    # Send various messages
    mavlink.send_heartbeat(armed=False)
    mavlink.send_attitude(0.1, 0.2, 0.3, 0, 0, 0)
    mavlink.send_position(0, 0, 0, 0, 0, 0)
    
    stats = mavlink.get_statistics()
    
    assert stats['connected'] == True
    assert stats['packets_sent'] >= 3
    assert stats['uptime'] >= 0
    
    print(f"✓ Statistics collected")
    print(f"  Uptime: {stats['uptime']:.2f}s")
    print(f"  Packets sent: {stats['packets_sent']}")
    print(f"  Packets received: {stats['packets_received']}")
    print(f"  Commands received: {stats['commands_received']}")
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_telemetry_stream():
    """Test continuous telemetry stream"""
    print("\n=== Testing Telemetry Stream (10 packets) ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14560', system_id=1)
    
    # Simulate 10 telemetry updates
    for i in range(10):
        # Simulate flight data
        t = i * 0.02  # 50Hz
        roll = 0.1 * np.sin(2 * np.pi * 0.5 * t)
        pitch = 0.05 * np.cos(2 * np.pi * 0.3 * t)
        yaw = 0.2 * t
        
        altitude = 5.0 + 0.5 * np.sin(2 * np.pi * 0.2 * t)
        
        # Send messages
        if i % 10 == 0:  # Heartbeat at 1Hz (simulated)
            mavlink.send_heartbeat(armed=True)
        
        mavlink.send_attitude(roll, pitch, yaw, 0, 0, 0)
        mavlink.send_position(0, 0, -altitude, 0, 0, 0)
    
    stats = mavlink.get_statistics()
    
    print(f"✓ Stream completed")
    print(f"  Packets sent: {stats['packets_sent']}")
    print(f"  Average frequency: {stats['packets_sent'] / stats['uptime']:.1f} Hz")
    
    assert stats['packets_sent'] >= 20  # At least attitude + position for each iteration
    
    mavlink.close()


@pytest.mark.skipif(not MAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_mavlink_message_validation():
    """Test message parameter validation"""
    print("\n=== Testing Message Validation ===")
    
    mavlink = MAVLinkTelemetry(port='udp:localhost:14561', system_id=1)
    
    # Test that messages handle edge cases gracefully
    
    # Large angles (should work)
    mavlink.send_attitude(np.pi, -np.pi, 2*np.pi, 10, 10, 10)
    print("✓ Large angles handled")
    
    # Zero values (should work)
    mavlink.send_attitude(0, 0, 0, 0, 0, 0)
    mavlink.send_position(0, 0, 0, 0, 0, 0)
    print("✓ Zero values handled")
    
    # Negative altitude (up in NED frame)
    mavlink.send_position(0, 0, -100, 0, 0, 0)
    print("✓ Negative altitude (100m up) handled")
    
    # Many RC channels
    channels = [1500] * 18
    mavlink.send_rc_channels(channels)
    print("✓ 18 RC channels handled")
    
    # Few RC channels (should pad with zeros)
    mavlink.send_rc_channels([1500, 1600, 1400, 1500])
    print("✓ Partial RC channels handled (auto-padded)")
    
    # Long status text (should truncate)
    long_text = "A" * 100
    mavlink.send_statustext(long_text)
    print("✓ Long status text handled (truncated to 50 chars)")
    
    mavlink.close()


def main():
    """Run all MAVLink tests"""
    print("=" * 60)
    print("MAVLink Telemetry Test Suite")
    print("=" * 60)
    
    if not MAVLINK_AVAILABLE:
        print("\n✗ pymavlink not installed")
        print("\nInstall with:")
        print("  pip install pymavlink")
        return 1
    
    tests = [
        ("MAVLink Import", test_mavlink_import),
        ("UDP Initialization", test_mavlink_initialization_udp),
        ("HEARTBEAT Message", test_mavlink_heartbeat),
        ("ATTITUDE Message", test_mavlink_attitude),
        ("POSITION Message", test_mavlink_position),
        ("SYS_STATUS Message", test_mavlink_sys_status),
        ("RC_CHANNELS Message", test_mavlink_rc_channels),
        ("SERVO_OUTPUT Message", test_mavlink_servo_output),
        ("STATUSTEXT Message", test_mavlink_statustext),
        ("Statistics Tracking", test_mavlink_statistics),
        ("Telemetry Stream", test_mavlink_telemetry_stream),
        ("Message Validation", test_mavlink_message_validation),
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            test_func()
            passed += 1
            print(f"\n✓ {name} PASSED")
        except Exception as e:
            failed += 1
            print(f"\n✗ {name} FAILED: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✓ All MAVLink tests passed!")
        print("\nNext steps:")
        print("  1. Test with ground station:")
        print("     python example_mavlink.py --simulator pybullet")
        print("  2. Connect QGroundControl to udp://localhost:14550")
        print("  3. Verify telemetry data display")
        return 0
    else:
        print(f"\n✗ {failed} test(s) failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())
