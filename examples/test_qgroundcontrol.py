"""
QGroundControl Connection Test

Simple test to verify QGroundControl can connect to the flight controller
via MAVLink telemetry.
"""

import time
import sys

try:
    from telemetry.mavlink_telemetry import MAVLinkTelemetry
    MAVLINK_AVAILABLE = True
except ImportError:
    print("✗ pymavlink not installed")
    print("  Install with: pip install pymavlink")
    MAVLINK_AVAILABLE = False
    sys.exit(1)


def test_qgc_connection(duration=60):
    """
    Test QGroundControl connection by streaming telemetry data.
    
    Args:
        duration: How long to run the test (seconds)
    """
    print("\n=== QGroundControl Connection Test ===\n")
    print("This test streams simulated telemetry data to QGroundControl.\n")
    print("Steps:")
    print("1. Start QGroundControl")
    print("2. Go to: Application Settings → Comm Links")
    print("3. Add a new connection:")
    print("   - Type: UDP")
    print("   - Port: 14550")
    print("   - Server Address: (leave default)")
    print("4. Connect to the new link")
    print("5. You should see the vehicle appear with telemetry data\n")
    
    input("Press Enter when QGroundControl is ready...")
    
    print("\nStarting telemetry stream...")
    print("Broadcasting on UDP port 14550\n")
    
    # Initialize telemetry
    telemetry = MAVLinkTelemetry()
    
    start_time = time.time()
    packet_count = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            
            # Simulate changing attitude
            import math
            roll = 10 * math.sin(elapsed * 0.5)
            pitch = 5 * math.cos(elapsed * 0.3)
            yaw = elapsed * 10 % 360
            
            # Simulate altitude change
            altitude = 2 + math.sin(elapsed * 0.2)
            
            # Send telemetry at different rates
            
            # High rate (10 Hz) - Attitude
            telemetry.send_heartbeat()
            telemetry.send_attitude(
                roll=math.radians(roll),
                pitch=math.radians(pitch),
                yaw=math.radians(yaw),
                rollspeed=0.1,
                pitchspeed=0.1,
                yawspeed=0.1
            )
            packet_count += 2
            
            # Medium rate (5 Hz) - Position
            if int(elapsed * 5) != int((elapsed - 0.1) * 5):
                telemetry.send_local_position(
                    x=0.0,
                    y=0.0,
                    z=-altitude,  # NED frame (down is negative)
                    vx=0.0,
                    vy=0.0,
                    vz=0.0
                )
                packet_count += 1
            
            # Low rate (1 Hz) - System status
            if int(elapsed) != int(elapsed - 0.1):
                # Simulate battery draining
                battery_remaining = max(0, 100 - int(elapsed / 60.0 * 100))
                telemetry.send_sys_status(
                    voltage_battery=11100 - int(elapsed * 10),  # mV
                    current_battery=1500,  # mA
                    battery_remaining=battery_remaining
                )
                telemetry.send_rc_channels(
                    chan1_raw=1500, chan2_raw=1500, chan3_raw=1500, chan4_raw=1500
                )
                telemetry.send_servo_output([1500, 1500, 1500, 1500, 1500, 1500])
                packet_count += 3
                
                # Print status
                print(f"t={elapsed:5.1f}s | Roll={roll:6.2f}° Pitch={pitch:6.2f}° "
                      f"Yaw={yaw:6.2f}° Alt={altitude:.2f}m | "
                      f"Packets sent: {packet_count} | Battery: {battery_remaining}%")
            
            time.sleep(0.1)  # 10 Hz update rate
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    print(f"\n✓ Test complete")
    print(f"  Total packets sent: {packet_count}")
    print(f"  Duration: {time.time() - start_time:.1f} seconds")
    print("\nIf QGroundControl displayed the vehicle and telemetry, the connection works!")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Test QGroundControl connection')
    parser.add_argument('--duration', type=int, default=60, 
                       help='Test duration in seconds (default: 60)')
    
    args = parser.parse_args()
    
    test_qgc_connection(duration=args.duration)
