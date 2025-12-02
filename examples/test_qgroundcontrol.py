"""
QGroundControl Connection Test

Simple test to verify QGroundControl can connect to the flight controller
via MAVLink telemetry.
"""

import time
import sys
import os

# Add project root to path so we can import telemetry module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from telemetry.mavlink_telemetry import MAVLinkTelemetry
    MAVLINK_AVAILABLE = True
except ImportError:
    print("✗ pymavlink not installed")
    print("  Install with: pip install pymavlink")
    MAVLINK_AVAILABLE = False
    sys.exit(1)


def test_qgc_connection(duration=60, auto_start=False):
    """
    Test QGroundControl connection by streaming telemetry data.
    
    Args:
        duration: How long to run the test (seconds)
        auto_start: Skip the "Press Enter" prompt
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
    
    if not auto_start:
        input("Press Enter when QGroundControl is ready...")
    
    print("\nStarting telemetry stream...")
    print("Broadcasting on UDP port 14550\n")
    
    # Initialize telemetry - send TO QGC listening on 14550
    # udpout means we send packets out to QGC's listening port
    telemetry = MAVLinkTelemetry(port='udpout:127.0.0.1:14550')
    
    start_time = time.time()
    packet_count = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            
            # Simulate changing attitude
            import math
            roll = 10 * math.sin(elapsed * 0.5)
            pitch = 5 * math.cos(elapsed * 0.3)
            
            # Simulate altitude change
            altitude = 2 + math.sin(elapsed * 0.2)
            
            # Simulate velocity (larger circular motion for visible speed)
            vx = 5.0 * math.cos(elapsed * 0.3)  # m/s in X direction (up to 11 mph)
            vy = 5.0 * math.sin(elapsed * 0.3)  # m/s in Y direction
            vz = 1.0 * math.cos(elapsed * 0.2)  # m/s vertical velocity (up to 3.3 ft/s)
            
            # Calculate heading from velocity vector (direction of travel)
            # In NED frame: North=0°, East=90°, atan2 gives angle from East
            yaw = (90 - math.degrees(math.atan2(vy, vx))) % 360  # Convert to North-referenced heading
            
            # Simulate position change (integrate velocity)
            # This creates a circular flight path
            x_pos = 16.67 * math.sin(elapsed * 0.3)  # meters (integral of vx)
            y_pos = -16.67 * math.cos(elapsed * 0.3)  # meters (integral of vy)
            
            # Convert position to lat/lon offset (approximate: 1 degree lat = 111km)
            lat_offset = (y_pos / 111000.0) * 1e7  # degrees * 1e7
            lon_offset = (x_pos / (111000.0 * math.cos(math.radians(37.7749)))) * 1e7  # degrees * 1e7
            
            current_lat = int(37.7749 * 1e7 + lat_offset)
            current_lon = int(-122.4194 * 1e7 + lon_offset)
            
            # Send telemetry at different rates
            
            # High rate (10 Hz) - Attitude
            telemetry.send_heartbeat(armed=True)  # Set armed=True to enable all displays
            telemetry.send_attitude(
                roll=math.radians(roll),
                pitch=math.radians(pitch),
                yaw=math.radians(yaw),
                rollspeed=0.1,
                pitchspeed=0.1,
                yawspeed=0.1
            )
            packet_count += 2
            
            # Medium rate (5 Hz) - Position and GPS
            if int(elapsed * 5) != int((elapsed - 0.1) * 5):
                # Calculate ground speed
                groundspeed = math.sqrt(vx*vx + vy*vy)  # m/s
                
                telemetry.send_position(
                    x=x_pos,
                    y=y_pos,
                    z=-altitude,  # NED frame (down is negative)
                    vx=vx,
                    vy=vy,
                    vz=-vz  # NED: negative is up
                )
                # Send GPS data with moving position
                telemetry.send_gps(
                    lat=current_lat,
                    lon=current_lon,
                    alt=int(altitude * 1000),  # Altitude in mm above sea level
                    vx=int(vx * 100),  # Ground speed X in cm/s
                    vy=int(vy * 100),  # Ground speed Y in cm/s
                    vz=int(-vz * 100),  # Ground speed Z in cm/s (NED)
                    hdg=int(yaw * 100),  # Heading in degrees * 100
                    fix_type=3,  # 3D GPS fix
                    satellites_visible=12
                )
                # Send global position with moving coordinates
                telemetry.send_global_position(
                    lat=current_lat,
                    lon=current_lon,
                    alt=int(altitude * 1000),  # mm above MSL
                    relative_alt=int(altitude * 1000),  # mm above home
                    vx=int(vx * 100),  # Ground speed X in cm/s
                    vy=int(vy * 100),  # Ground speed Y in cm/s
                    vz=int(-vz * 100),  # Ground speed Z in cm/s (NED)
                    hdg=int(yaw * 100)
                )
                # Send VFR HUD for speed displays
                telemetry.send_vfr_hud(
                    airspeed=groundspeed,  # Use groundspeed as airspeed (no wind)
                    groundspeed=groundspeed,
                    heading=int(yaw),
                    throttle=50,  # 50% throttle
                    alt=altitude,
                    climb=-vz  # NED: negative vz means climbing
                )
                packet_count += 4
            
            # Low rate (1 Hz) - System status
            if int(elapsed) != int(elapsed - 0.1):
                # Simulate battery draining
                battery_remaining = max(0, 100 - int(elapsed / 60.0 * 100))
                telemetry.send_sys_status(
                    voltage_battery=11100 - int(elapsed * 10),  # mV
                    current_battery=1500,  # mA
                    battery_remaining=battery_remaining
                )
                telemetry.send_rc_channels([1500, 1500, 1500, 1500])
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
    parser.add_argument('--auto-start', action='store_true',
                       help='Skip the confirmation prompt and start immediately')
    
    args = parser.parse_args()
    
    test_qgc_connection(duration=args.duration, auto_start=args.auto_start)
