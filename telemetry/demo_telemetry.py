"""
MAVLink Telemetry Console Demo

Simple demonstration of MAVLink telemetry without requiring a ground station.
Prints telemetry data to console to verify the MAVLink module is working.

Usage:
    python demo_telemetry.py
"""

import time
import math
import sys

try:
    from telemetry.mavlink_telemetry import MAVLinkTelemetry
    MAVLINK_AVAILABLE = True
except ImportError:
    print("✗ pymavlink not installed")
    print("Install with: pip install pymavlink")
    MAVLINK_AVAILABLE = False
    sys.exit(1)


def demo_console_telemetry():
    """Demonstrate telemetry output to console"""
    print("=" * 70)
    print("MAVLink Telemetry Console Demo")
    print("=" * 70)
    print("\nThis demo simulates telemetry data and displays it in the console.")
    print("In production, this data would be sent to QGroundControl or Mission Planner.")
    print("\nPress Ctrl+C to stop\n")
    
    # Create telemetry instance (UDP port for potential GCS connection)
    print("[1/2] Initializing MAVLink telemetry...")
    mavlink = MAVLinkTelemetry(port='udp:0.0.0.0:14550', system_id=1)
    print(f"✓ MAVLink initialized on UDP port 14550")
    print(f"  You can connect QGroundControl to: udp://localhost:14550")
    print(f"  System ID: {mavlink.system_id}\n")
    
    print("[2/2] Starting telemetry stream...")
    print("=" * 70)
    
    # Flight simulation parameters
    armed = False
    altitude = 0.0
    battery_voltage = 12.6  # V
    battery_current = 5.0   # A
    battery_remaining = 100  # %
    
    loop_count = 0
    start_time = time.time()
    
    try:
        while True:
            loop_start = time.time()
            elapsed = loop_start - start_time
            
            # Simulate flight dynamics
            if elapsed > 5.0 and not armed:
                armed = True
                print("\n>>> ARMED <<<\n")
            
            if armed:
                # Simulate takeoff and hover with oscillations
                target_altitude = 10.0
                altitude = target_altitude * (1 - math.exp(-elapsed/10.0))
                
                # Simulated attitude (small oscillations)
                roll = 0.1 * math.sin(2 * math.pi * 0.5 * elapsed)
                pitch = 0.05 * math.cos(2 * math.pi * 0.3 * elapsed)
                yaw = 0.02 * elapsed  # Slow rotation
                
                # Angular rates
                rollspeed = 0.1 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * elapsed)
                pitchspeed = -0.05 * 2 * math.pi * 0.3 * math.sin(2 * math.pi * 0.3 * elapsed)
                yawspeed = 0.02
                
                # Velocity (small)
                vx = 0.1 * math.sin(elapsed)
                vy = 0.1 * math.cos(elapsed)
                vz = -0.05 if altitude < target_altitude else 0.0
                
                # Battery drain
                battery_remaining = max(0, 100 - (elapsed / 600.0) * 100)
                battery_voltage = 12.6 - (100 - battery_remaining) * 0.03
                battery_current = 15.0 + 5.0 * math.sin(elapsed)
            else:
                # On ground, disarmed
                roll = pitch = yaw = 0.0
                rollspeed = pitchspeed = yawspeed = 0.0
                vx = vy = vz = 0.0
            
            # Motor PWM (simulated hover values)
            if armed:
                base_throttle = 1550
                motor_pwm = [
                    base_throttle + int(50 * math.sin(elapsed + i * math.pi / 3))
                    for i in range(6)
                ]
            else:
                motor_pwm = [1000] * 6
            
            # RC inputs (simulated)
            rc_channels = [
                1500 + int(100 * math.sin(elapsed)),      # Throttle
                1500 + int(200 * math.cos(elapsed * 0.5)), # Roll
                1500 + int(150 * math.sin(elapsed * 0.7)), # Pitch
                1500,                                       # Yaw
                1000, 2000, 1500, 1500                     # Aux channels
            ]
            
            # === Send MAVLink Messages ===
            
            # HEARTBEAT (1Hz)
            if loop_count % 50 == 0:
                mavlink.send_heartbeat(armed=armed)
            
            # ATTITUDE (50Hz)
            mavlink.send_attitude(roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
            
            # LOCAL_POSITION_NED (50Hz)
            mavlink.send_position(0, 0, -altitude, vx, vy, vz)
            
            # SYS_STATUS (1Hz)
            if loop_count % 50 == 0:
                mavlink.send_sys_status(
                    voltage_battery=int(battery_voltage * 1000),
                    current_battery=int(battery_current * 100),
                    battery_remaining=int(battery_remaining),
                    cpu_load=int(350)  # 35% CPU
                )
            
            # RC_CHANNELS (10Hz)
            if loop_count % 5 == 0:
                mavlink.send_rc_channels(rc_channels, rssi=200)
            
            # SERVO_OUTPUT_RAW (10Hz)
            if loop_count % 5 == 0:
                mavlink.send_servo_output(motor_pwm)
            
            # STATUSTEXT (occasional)
            if loop_count == 50:
                mavlink.send_statustext("Flight controller initialized", severity=6)
            elif loop_count == 250:
                mavlink.send_statustext("Takeoff complete", severity=6)
            elif loop_count == 500 and battery_remaining < 20:
                mavlink.send_statustext("Battery low", severity=4)
            
            # Handle incoming commands
            cmd = mavlink.handle_commands()
            if cmd:
                if cmd.get('command_name') == 'ARM_DISARM':
                    armed = (cmd['param1'] == 1.0)
                    mavlink.send_command_ack(cmd['command'], result=0)
                    print(f"\n>>> {'ARMED' if armed else 'DISARMED'} (via GCS command) <<<\n")
            
            # === Console Display (1Hz) ===
            if loop_count % 50 == 0:
                # Clear previous line (if supported)
                print(f"\r{' ' * 120}", end='')
                
                print(f"\n{'=' * 70}")
                print(f"Time: {elapsed:.1f}s  |  Loop: {loop_count}  |  {'ARMED' if armed else 'DISARMED'}")
                print(f"{'=' * 70}")
                
                print(f"ATTITUDE:")
                print(f"  Roll:  {math.degrees(roll):+7.2f}°  Rate: {math.degrees(rollspeed):+6.2f}°/s")
                print(f"  Pitch: {math.degrees(pitch):+7.2f}°  Rate: {math.degrees(pitchspeed):+6.2f}°/s")
                print(f"  Yaw:   {math.degrees(yaw):+7.2f}°  Rate: {math.degrees(yawspeed):+6.2f}°/s")
                
                print(f"\nPOSITION:")
                print(f"  Altitude: {altitude:6.2f}m")
                print(f"  Velocity: [{vx:+5.2f}, {vy:+5.2f}, {vz:+5.2f}] m/s")
                
                print(f"\nSYSTEM:")
                print(f"  Battery:  {battery_voltage:.2f}V  {battery_current:.1f}A  {battery_remaining:.0f}%")
                print(f"  CPU Load: 35%")
                
                print(f"\nMOTORS (PWM μs):")
                print(f"  M0:{motor_pwm[0]}  M1:{motor_pwm[1]}  M2:{motor_pwm[2]}")
                print(f"  M3:{motor_pwm[3]}  M4:{motor_pwm[4]}  M5:{motor_pwm[5]}")
                
                print(f"\nRC INPUT:")
                print(f"  THR:{rc_channels[0]}  ROLL:{rc_channels[1]}  "
                      f"PITCH:{rc_channels[2]}  YAW:{rc_channels[3]}")
                
                # Telemetry statistics
                stats = mavlink.get_statistics()
                print(f"\nTELEMETRY:")
                print(f"  TX: {stats['packets_sent']} packets  "
                      f"RX: {stats['packets_received']} packets  "
                      f"Commands: {stats['commands_received']}")
                
                if stats['packets_received'] > 0:
                    print(f"  ⚠ Ground Control Station connected!")
            
            loop_count += 1
            
            # Maintain 50Hz loop rate
            elapsed_loop = time.time() - loop_start
            if elapsed_loop < 0.02:
                time.sleep(0.02 - elapsed_loop)
    
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("Telemetry Demo Stopped")
        print("=" * 70)
        
        stats = mavlink.get_statistics()
        print(f"\nSession Statistics:")
        print(f"  Runtime:           {stats['uptime']:.1f} seconds")
        print(f"  Packets sent:      {stats['packets_sent']}")
        print(f"  Packets received:  {stats['packets_received']}")
        print(f"  Commands received: {stats['commands_received']}")
        print(f"  Average TX rate:   {stats['packets_sent'] / stats['uptime']:.1f} packets/sec")
        
        if stats['packets_received'] > 0:
            print(f"\n✓ Ground Control Station was connected!")
        else:
            print(f"\nℹ No Ground Control Station detected")
            print(f"  To connect GCS: udp://localhost:14550")
    
    finally:
        mavlink.close()


def main():
    if not MAVLINK_AVAILABLE:
        return 1
    
    print("\nMAVLink Console Telemetry Demo")
    print("This demonstrates telemetry without requiring QGroundControl")
    print("\nOptional: Open QGroundControl and connect to udp://localhost:14550")
    print("          to see telemetry in the ground station\n")
    
    input("Press Enter to start...")
    
    demo_console_telemetry()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
