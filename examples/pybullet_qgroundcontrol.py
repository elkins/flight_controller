"""
3D PyBullet Flight Simulation with Live QGroundControl Telemetry

Combines research-grade physics simulation with real-time telemetry streaming
to QGroundControl. Watch your hexacopter fly in 3D while monitoring all
telemetry data in a professional ground control station.

Features:
- Enhanced PyBullet physics (ground effect, aerodynamic drag)
- Full MAVLink telemetry suite (attitude, GPS, VFR HUD, etc.)
- Multiple flight modes and maneuvers
- Real-time visualization in PyBullet GUI
- Live telemetry display in QGroundControl

Usage:
    # Terminal 1: Start QGroundControl (it auto-listens on UDP 14550)
    open -a QGroundControl
    
    # Terminal 2: Run simulation
    python examples/pybullet_qgroundcontrol.py --duration 120 --gui
"""

import sys
import os
import time
import math
import numpy as np
import argparse

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Check for PyBullet
try:
    import pybullet as p
    PYBULLET_AVAILABLE = True
except ImportError:
    print("✗ PyBullet not installed")
    print("  Install with: pip install pybullet")
    PYBULLET_AVAILABLE = False
    sys.exit(1)

# Check for MAVLink
try:
    from telemetry.mavlink_telemetry import MAVLinkTelemetry
    MAVLINK_AVAILABLE = True
except ImportError:
    print("✗ pymavlink not installed")
    print("  Install with: pip install pymavlink")
    MAVLINK_AVAILABLE = False
    sys.exit(1)

# Import enhanced PyBullet HAL
from src.hal.hal_pybullet_enhanced import EnhancedPyBulletPlatform


def normalize_angle(angle_deg):
    """Normalize angle to [-180, 180] range"""
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


class SimplePID:
    """Simple PID controller for stabilization"""
    def __init__(self, kp, ki, kd, output_min=-500, output_max=500, angle_wrap=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.angle_wrap = angle_wrap  # Set True for yaw control
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()
    
    def update(self, setpoint, measured):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.001
        
        error = setpoint - measured
        
        # Wrap angle error to [-180, 180] for yaw control
        if self.angle_wrap:
            error = normalize_angle(error)
        
        # Update integral with anti-windup (clamp to ±50)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -50, 50)
        derivative = (error - self.last_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = np.clip(output, self.output_min, self.output_max)
        
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID state"""
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()


def run_simulation(duration=120, gui=True, flight_mode='hover', auto_start=False):
    """
    Run PyBullet simulation with live QGroundControl telemetry.
    
    Args:
        duration: Simulation duration in seconds
        gui: Show PyBullet GUI window
        flight_mode: Flight pattern ('hover', 'circle', 'square', 'figure8')
        auto_start: Skip the confirmation prompt
    """
    print("\n=== 3D PyBullet Simulation with QGroundControl ===\n")
    
    # Initialize MAVLink telemetry
    print("Initializing MAVLink telemetry...")
    telemetry = MAVLinkTelemetry(port='udpout:127.0.0.1:14550')
    print("✓ Broadcasting on UDP 127.0.0.1:14550")
    print("  QGroundControl should auto-connect\n")
    
    # Initialize PyBullet platform
    print("Initializing PyBullet simulation...")
    platform = EnhancedPyBulletPlatform(
        gui=gui,
        start_position=[0, 0, 0.05]  # Start just above ground
    )
    print("✓ Enhanced physics simulation ready")
    print(f"  Mode: {'GUI' if gui else 'Headless'}")
    print(f"  Flight pattern: {flight_mode}\n")
    
    # Start video recording if requested
    video_log_id = None
    if args.record:
        if gui:
            video_log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, args.record)
            print(f"📹 Recording video to: {args.record}\n")
        else:
            print("⚠️  Video recording requires --gui mode\n")
    
    # Initialize PID controllers (tuned for simulation)
    roll_pid = SimplePID(kp=4.0, ki=0.1, kd=2.0, output_min=-300, output_max=300)
    pitch_pid = SimplePID(kp=4.0, ki=0.1, kd=2.0, output_min=-300, output_max=300)
    yaw_pid = SimplePID(kp=1.0, ki=0.05, kd=0.5, output_min=-200, output_max=200, angle_wrap=True)
    alt_pid = SimplePID(kp=100, ki=2, kd=80, output_min=-250, output_max=250)
    
    # GPS home position (San Francisco)
    HOME_LAT = 37.7749 * 1e7  # degrees * 1e7
    HOME_LON = -122.4194 * 1e7
    METERS_TO_DEGREES_LAT = 1.0 / 111000.0 * 1e7
    
    # Flight parameters
    start_time = time.time()
    loop_count = 0
    armed = False
    target_altitude = 0.0
    battery_voltage = 12600  # mV (fully charged 3S LiPo)
    
    print("Starting flight simulation...")
    print(f"Duration: {duration}s\n")
    print("Flight sequence:")
    print("  0-3s:     Pre-flight (disarmed)")
    print("  3-8s:     Takeoff to 1.5m")
    print("  8-(T-8)s: Flight pattern")
    print("  (T-8)-(T-3)s: Landing approach (gradual descent)")
    print("  Last 3s:  Final landing (touch down)\n")
    
    try:
        while time.time() - start_time < duration:
            loop_start = time.time()
            elapsed = time.time() - start_time
            
            # Get current state from simulation
            state = platform.get_state()
            position = state['position']
            euler = state['euler']  # (roll, pitch, yaw) in radians
            velocity = state['velocity']
            angular_velocity = state['angular_velocity']
            
            # Flight phase state machine
            if elapsed < 3:
                # Pre-flight: disarmed
                armed = False
                target_altitude = 0.0
                throttle_base = 1000
                flight_phase = "PRE-FLIGHT"
            elif elapsed < 8:
                # Takeoff phase
                armed = True
                target_altitude = 1.5
                throttle_base = 1500  # Increased base throttle for takeoff
                flight_phase = "TAKEOFF"
            elif elapsed < duration - 8:
                # Flight pattern phase
                armed = True
                target_altitude = 1.5
                throttle_base = 1500  # Keep higher throttle during flight
                flight_phase = "PATTERN"
            elif elapsed < duration - 3:
                # Landing approach - gradual descent
                armed = True
                landing_time = elapsed - (duration - 8)
                # Gradual descent over 5 seconds: 1.5m -> 0.2m
                target_altitude = max(0.2, 1.5 - (landing_time / 5.0) * 1.3)
                throttle_base = 1100  # Further reduced for better descent
                flight_phase = f"LANDING({target_altitude:.2f}m)"
            else:
                # Final landing - cut throttle when near ground
                armed = True
                if position[2] < 0.15:
                    target_altitude = 0.0
                    throttle_base = 1000  # Minimal throttle
                    armed = False  # Disarm on ground
                    flight_phase = "LANDED"
                else:
                    target_altitude = 0.1
                    throttle_base = 1000  # Minimal throttle for final descent
                    flight_phase = "FINAL_LANDING"
            
            # Calculate flight pattern setpoints
            if flight_mode == 'circle' and elapsed >= 8 and elapsed < duration - 8:
                # Gentle rotating circle - yaw rotation only
                pattern_time = elapsed - 8
                angular_speed = 0.15  # rad/s (slow rotation)
                angle = pattern_time * angular_speed
                
                # Keep level, just rotate heading
                roll_setpoint = 0
                pitch_setpoint = 0
                yaw_setpoint = math.degrees(angle) % 360
                
            elif flight_mode == 'figure8' and elapsed >= 8 and elapsed < duration - 8:
                # Figure-8 pattern with very gentle banking
                pattern_time = elapsed - 8
                angular_speed = 0.2  # rad/s (slower for stability)
                angle = angular_speed * pattern_time
                
                # Figure-8 motion: x = sin(t), y = sin(2t)/2
                # Heading follows the path tangent
                yaw_setpoint = math.degrees(angle * 2) % 360
                
                # Very gentle coordinated turns (reduced from ±2° to ±1°)
                roll_setpoint = -1 * math.sin(angle * 2)  # Minimal banking
                pitch_setpoint = 0  # Keep level pitch
                
            elif flight_mode == 'square' and elapsed >= 8 and elapsed < duration - 8:
                # Square waypoint pattern with timed legs
                pattern_time = elapsed - 8
                leg_duration = 8  # seconds per leg
                current_leg = int(pattern_time / leg_duration) % 4
                
                # Smooth transitions between legs
                if current_leg == 0:  # North leg
                    roll_setpoint = 0
                    pitch_setpoint = 2
                    yaw_setpoint = 0
                elif current_leg == 1:  # East leg
                    roll_setpoint = 2
                    pitch_setpoint = 0
                    yaw_setpoint = 90
                elif current_leg == 2:  # South leg
                    roll_setpoint = 0
                    pitch_setpoint = -2
                    yaw_setpoint = 180
                else:  # West leg
                    roll_setpoint = -2
                    pitch_setpoint = 0
                    yaw_setpoint = 270
            
            elif flight_mode == 'waypoint' and elapsed >= 8 and elapsed < duration - 8:
                # Waypoint navigation - triangle course
                pattern_time = elapsed - 8
                waypoint_duration = 10  # seconds per waypoint
                current_wp = int(pattern_time / waypoint_duration) % 3
                
                if current_wp == 0:  # Waypoint 1: Forward
                    roll_setpoint = 0
                    pitch_setpoint = 3
                    yaw_setpoint = 0
                elif current_wp == 1:  # Waypoint 2: Right
                    roll_setpoint = 0
                    pitch_setpoint = 0
                    yaw_setpoint = 120
                else:  # Waypoint 3: Return
                    roll_setpoint = 0
                    pitch_setpoint = 0
                    yaw_setpoint = 240
            else:
                # Hover (default)
                roll_setpoint = 0
                pitch_setpoint = 0
                yaw_setpoint = math.degrees(euler[2])  # Hold current heading
            
            # Altitude control with velocity damping
            altitude_error = target_altitude - position[2]
            altitude_correction = alt_pid.update(target_altitude, position[2])
            
            # Smart damping: prevent overshoot during climb, allow descent during landing
            if elapsed < duration - 8:
                # During normal flight: strong damping to prevent overshoot
                velocity_damping = -velocity[2] * 150
            else:
                # During landing: only damp upward velocity, don't fight descent
                velocity_damping = -max(0, velocity[2]) * 80
            
            throttle = throttle_base + int(altitude_correction + velocity_damping)
            
            # Attitude stabilization
            roll_correction = roll_pid.update(roll_setpoint, math.degrees(euler[0]))
            pitch_correction = pitch_pid.update(pitch_setpoint, math.degrees(euler[1]))
            yaw_correction = yaw_pid.update(yaw_setpoint, math.degrees(euler[2]))
            
            # Hexacopter motor mixing
            # Motor layout: 0=Front, 1=FR, 2=BR, 3=Back, 4=BL, 5=FL
            motors = [throttle] * 6
            motors[0] += int(pitch_correction + yaw_correction)      # Front
            motors[1] += int(pitch_correction/2 - roll_correction - yaw_correction)  # FR
            motors[2] += int(-pitch_correction/2 - roll_correction + yaw_correction) # BR
            motors[3] += int(-pitch_correction - yaw_correction)     # Back
            motors[4] += int(-pitch_correction/2 + roll_correction - yaw_correction) # BL
            motors[5] += int(pitch_correction/2 + roll_correction + yaw_correction)  # FL
            
            # Clip to valid PWM range
            motors = [int(np.clip(m, 1000, 2000)) for m in motors]
            
            # Apply motor commands to simulation
            for i, motor_pwm in enumerate(motors):
                platform.motors[i].pulse_width_us(motor_pwm)
            
            # Step physics simulation
            try:
                platform.delay_ms(4)  # ~240Hz control loop
            except Exception as e:
                print(f"\n✗ Simulation error: {e}")
                print("  (PyBullet window may have been closed)")
                break
            
            # === Send MAVLink Telemetry ===
            
            # Convert position to GPS coordinates
            lat_offset = (position[1] / 111000.0) * 1e7  # North offset
            lon_offset = (position[0] / (111000.0 * math.cos(math.radians(37.7749)))) * 1e7  # East offset
            current_lat = int(HOME_LAT + lat_offset)
            current_lon = int(HOME_LON + lon_offset)
            
            # Calculate ground speed and heading from velocity
            groundspeed = math.sqrt(velocity[0]**2 + velocity[1]**2)  # m/s
            if groundspeed > 0.1:
                heading = (90 - math.degrees(math.atan2(velocity[1], velocity[0]))) % 360
            else:
                heading = (math.degrees(euler[2]) % 360)
            
            # Ensure heading is in range [0, 360) - sometimes gets 360.0 which causes issues
            if heading >= 360:
                heading = 0
            
            # Send telemetry messages at appropriate rates
            
            # High rate (10 Hz) - Attitude
            telemetry.send_heartbeat(armed=armed)
            telemetry.send_attitude(
                roll=euler[0],
                pitch=euler[1],
                yaw=euler[2],
                rollspeed=angular_velocity[0],
                pitchspeed=angular_velocity[1],
                yawspeed=angular_velocity[2]
            )
            
            # Medium rate (5 Hz) - Position and GPS
            if loop_count % 2 == 0:
                telemetry.send_position(
                    x=position[0],
                    y=position[1],
                    z=-position[2],  # NED frame
                    vx=velocity[0],
                    vy=velocity[1],
                    vz=-velocity[2]
                )
                
                telemetry.send_gps(
                    lat=current_lat,
                    lon=current_lon,
                    alt=int(position[2] * 1000),  # mm above MSL
                    vx=int(velocity[0] * 100),  # cm/s
                    vy=int(velocity[1] * 100),
                    vz=int(-velocity[2] * 100),
                    hdg=int(heading * 100),
                    fix_type=3,
                    satellites_visible=12
                )
                
                telemetry.send_global_position(
                    lat=current_lat,
                    lon=current_lon,
                    alt=int(position[2] * 1000),
                    relative_alt=int(position[2] * 1000),
                    vx=int(velocity[0] * 100),
                    vy=int(velocity[1] * 100),
                    vz=int(-velocity[2] * 100),
                    hdg=int(heading * 100)
                )
                
                telemetry.send_vfr_hud(
                    airspeed=groundspeed,
                    groundspeed=groundspeed,
                    heading=int(heading) % 360,  # Ensure 0-359 range
                    throttle=int((throttle - 1000) / 10),  # 0-100%
                    alt=position[2],
                    climb=-velocity[2]
                )
            
            # Low rate (1 Hz) - System status
            if loop_count % 10 == 0:
                # Simulate battery drain (0.1V per minute)
                battery_voltage = int(12600 - (elapsed / 60.0) * 100)
                battery_remaining = max(0, int(100 - (elapsed / duration) * 100))
                
                telemetry.send_sys_status(
                    voltage_battery=battery_voltage,
                    current_battery=1500,
                    battery_remaining=battery_remaining
                )
                
                telemetry.send_rc_channels([1500, 1500, throttle, 1500])
                telemetry.send_servo_output(motors)
            
            loop_count += 1
            
            # Print status every second
            if loop_count % 10 == 0:
                status = "ARMED" if armed else "DISARMED"
                print(f"t={elapsed:5.1f}s [{flight_phase:12s}] | "
                      f"Alt={position[2]:5.2f}m (target={target_altitude:.2f}m) | "
                      f"Roll={math.degrees(euler[0]):6.1f}° "
                      f"Pitch={math.degrees(euler[1]):6.1f}° "
                      f"Yaw={heading:6.1f}° | "
                      f"Speed={groundspeed*2.237:5.1f}mph | "
                      f"Batt={battery_remaining}%")
            
            # Maintain 10 Hz telemetry rate (PyBullet runs at 240Hz internally)
            loop_time = time.time() - loop_start
            if loop_time < 0.1:
                time.sleep(0.1 - loop_time)
    
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
    
    finally:
        # Cleanup
        if video_log_id is not None:
            p.stopStateLogging(video_log_id)
            print(f"\n✅ Video saved to: {args.record}")
        
        platform.disconnect()
        print("\n✓ Simulation complete!")
        print(f"  Total time: {time.time() - start_time:.1f}s")
        print(f"  Total loops: {loop_count}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='3D PyBullet simulation with live QGroundControl telemetry',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Hovering flight with GUI
  python examples/pybullet_qgroundcontrol.py --duration 60 --gui --mode hover
  
  # Circular flight pattern
  python examples/pybullet_qgroundcontrol.py --duration 120 --gui --mode circle
  
  # Figure-8 aerobatics
  python examples/pybullet_qgroundcontrol.py --duration 180 --gui --mode figure8
  
  # Headless simulation (faster)
  python examples/pybullet_qgroundcontrol.py --duration 300 --mode circle

Before running:
  1. Start QGroundControl (auto-listens on UDP 14550)
  2. Run this script
  3. Watch the drone fly in PyBullet AND QGroundControl!
        """
    )
    
    parser.add_argument(
        '--duration', 
        type=int, 
        default=120,
        help='Simulation duration in seconds (default: 120)'
    )
    
    parser.add_argument(
        '--gui', 
        action='store_true',
        help='Show PyBullet GUI window (default: headless for speed)'
    )
    
    parser.add_argument(
        '--mode',
        type=str,
        choices=['hover', 'circle', 'square', 'figure8', 'waypoint'],
        default='circle',
        help='Flight pattern mode (default: circle)'
    )
    
    parser.add_argument(
        '--auto-start',
        action='store_true',
        help='Skip the confirmation prompt and start immediately'
    )
    
    parser.add_argument(
        '--record',
        type=str,
        metavar='FILENAME',
        help='Record video to MP4 file (e.g., flight_demo.mp4)'
    )
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("  3D PYBULLET SIMULATION + QGROUNDCONTROL TELEMETRY")
    print("="*60)
    
    if not args.auto_start:
        print("\nMake sure QGroundControl is running!")
        print("  macOS: open -a QGroundControl")
        print("  Linux: ./QGroundControl.AppImage")
        print("  Windows: QGroundControl.exe\n")
        
        input("Press Enter when QGroundControl is ready...")
    
    run_simulation(
        duration=args.duration,
        gui=args.gui,
        flight_mode=args.mode,
        auto_start=args.auto_start
    )
