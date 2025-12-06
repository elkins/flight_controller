"""
PyBullet Flight with Cascaded Control Architecture

Demonstrates proper flight control using cascaded PID loops:
    Position → Velocity → Attitude → Rate → Motors

This is how REAL flight controllers work (PX4, Betaflight, etc.)
"""

import sys
import os
import time
import math
import numpy as np
import argparse

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pybullet as p
except ImportError:
    print("✗ PyBullet not installed")
    print("  Install with: pip install pybullet")
    sys.exit(1)

from src.hal.hal_pybullet_enhanced import EnhancedPyBulletPlatform
from src.control.cascaded_controller import CascadedMulticopterController, ControlSetpoint

try:
    from telemetry.mavlink_telemetry import MAVLinkTelemetry
    MAVLINK_AVAILABLE = True
except ImportError:
    print("⚠️  MAVLink not available (optional)")
    MAVLINK_AVAILABLE = False


def run_cascaded_demo(duration=60, gui=True, flight_mode='hover'):
    """
    Run PyBullet simulation with cascaded control.

    Args:
        duration: Simulation duration in seconds
        gui: Show PyBullet GUI window
        flight_mode: 'hover', 'figure8', 'circle', 'square'
    """
    print("\n" + "="*70)
    print("  PYBULLET SIMULATION WITH CASCADED CONTROL")
    print("="*70)
    print("\nControl Architecture:")
    print("  Position → Velocity → Attitude → Rate → Motors")
    print("  (Just like PX4, Betaflight, and other real flight controllers)\n")

    # Initialize PyBullet platform
    print("Initializing enhanced PyBullet simulation...")
    platform = EnhancedPyBulletPlatform(
        gui=gui,
        start_position=[0, 0, 0.05]
    )
    print("✓ Physics simulation ready")

    # Initialize cascaded controller
    print("Initializing cascaded controller...")
    controller = CascadedMulticopterController(num_motors=6, mass_kg=1.5)
    print("✓ Controller ready")
    print(f"  Flight mode: {flight_mode}")
    print(f"  Duration: {duration}s\n")

    # Initialize MAVLink telemetry (optional)
    if MAVLINK_AVAILABLE:
        telemetry = MAVLinkTelemetry(port='udpout:127.0.0.1:14550')
        print("✓ MAVLink telemetry enabled")
    else:
        telemetry = None

    # Recording
    video_log_id = None
    if args.record and gui:
        video_log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, args.record)
        print(f"📹 Recording to: {args.record}\n")

    # Flight sequence
    print("Flight sequence:")
    print("  0-3s:      Startup (disarmed)")
    print("  3-8s:      Takeoff to 2.0m")
    print("  8-(T-10)s: Flight pattern")
    print("  (T-10)-T:  Landing\n")

    # Timing
    start_time = time.time()
    loop_count = 0
    armed = False

    # GPS home (for telemetry)
    HOME_LAT = 37.7749 * 1e7
    HOME_LON = -122.4194 * 1e7

    print("="*70)
    print("Starting flight...\n")

    try:
        while time.time() - start_time < duration:
            loop_start = time.time()
            elapsed = time.time() - start_time

            # Get current state
            state = platform.get_state()

            # Flight phase state machine
            if elapsed < 3:
                # Startup: disarmed
                armed = False
                setpoint = None
                flight_phase = "STARTUP"
            elif elapsed < 8:
                # Takeoff: position control to 2.0m
                armed = True
                takeoff_progress = (elapsed - 3) / 5.0
                target_z = 2.0 * takeoff_progress
                setpoint = ControlSetpoint(
                    position=np.array([0.0, 0.0, target_z]),
                    yaw=0.0
                )
                flight_phase = "TAKEOFF"
            elif elapsed < duration - 10:
                # Flight pattern
                armed = True
                pattern_time = elapsed - 8

                if flight_mode == 'hover':
                    setpoint = ControlSetpoint(
                        position=np.array([0.0, 0.0, 2.0]),
                        yaw=0.0
                    )
                elif flight_mode == 'circle':
                    # Circular pattern: 2m radius
                    angle = pattern_time * 0.2  # rad/s
                    x = 2.0 * np.cos(angle)
                    y = 2.0 * np.sin(angle)
                    setpoint = ControlSetpoint(
                        position=np.array([x, y, 2.0]),
                        yaw=angle
                    )
                elif flight_mode == 'figure8':
                    # Figure-8 pattern
                    angle = pattern_time * 0.3
                    x = 3.0 * np.sin(angle)
                    y = 1.5 * np.sin(2 * angle)
                    yaw = angle
                    setpoint = ControlSetpoint(
                        position=np.array([x, y, 2.0]),
                        yaw=yaw
                    )
                elif flight_mode == 'square':
                    # Square pattern: 2m x 2m
                    leg_duration = 5.0
                    leg = int(pattern_time / leg_duration) % 4
                    corners = [
                        [1.5, 1.5, 2.0],   # NE
                        [1.5, -1.5, 2.0],  # SE
                        [-1.5, -1.5, 2.0], # SW
                        [-1.5, 1.5, 2.0],  # NW
                    ]
                    setpoint = ControlSetpoint(
                        position=np.array(corners[leg]),
                        yaw=leg * np.pi/2
                    )

                flight_phase = f"PATTERN({flight_mode.upper()})"
            else:
                # Landing: smooth descent
                armed = True
                landing_progress = (elapsed - (duration - 10)) / 10.0
                target_z = 2.0 * (1.0 - landing_progress)
                setpoint = ControlSetpoint(
                    position=np.array([0.0, 0.0, max(0.0, target_z)]),
                    yaw=0.0
                )
                flight_phase = "LANDING"

                # Disarm on ground
                if state['position'][2] < 0.15:
                    armed = False
                    flight_phase = "LANDED"

            # Compute control using cascaded controller
            if armed and setpoint is not None:
                motor_pwm = controller.compute_control(state, setpoint)
            else:
                # Disarmed: motors off
                motor_pwm = np.array([1000] * 6)

            # Apply motor commands
            for i, pwm in enumerate(motor_pwm):
                platform.motors[i].pulse_width_us(int(pwm))

            # Step simulation
            platform.delay_ms(10)  # 100Hz control loop

            # Send telemetry (if available)
            if telemetry and loop_count % 10 == 0:
                position = state['position']
                euler = state['euler']
                velocity = state['velocity']

                # GPS position
                lat_offset = (position[1] / 111000.0) * 1e7
                lon_offset = (position[0] / (111000.0 * math.cos(math.radians(37.7749)))) * 1e7
                current_lat = int(HOME_LAT + lat_offset)
                current_lon = int(HOME_LON + lon_offset)

                groundspeed = math.sqrt(velocity[0]**2 + velocity[1]**2)
                if groundspeed > 0.1:
                    heading = (90 - math.degrees(math.atan2(velocity[1], velocity[0]))) % 360
                else:
                    heading = (math.degrees(euler[2]) % 360)

                telemetry.send_heartbeat(armed=armed)
                telemetry.send_attitude(
                    roll=euler[0], pitch=euler[1], yaw=euler[2],
                    rollspeed=state['angular_velocity'][0],
                    pitchspeed=state['angular_velocity'][1],
                    yawspeed=state['angular_velocity'][2]
                )

                if loop_count % 20 == 0:  # 5 Hz
                    telemetry.send_gps(
                        lat=current_lat, lon=current_lon,
                        alt=int(position[2] * 1000),
                        vx=int(velocity[0] * 100),
                        vy=int(velocity[1] * 100),
                        vz=int(-velocity[2] * 100),
                        hdg=int(heading * 100),
                        fix_type=3, satellites_visible=12
                    )
                    telemetry.send_vfr_hud(
                        airspeed=groundspeed, groundspeed=groundspeed,
                        heading=int(heading) % 360,
                        throttle=int((motor_pwm.mean() - 1000) / 10),
                        alt=position[2], climb=-velocity[2]
                    )

            loop_count += 1

            # Print status every second
            if loop_count % 100 == 0:
                pos = state['position']
                euler = state['euler']
                vel = state['velocity']
                hover_est = controller.vel_controller.hover_thrust_estimate
                print(f"t={elapsed:5.1f}s [{flight_phase:20s}] | "
                      f"Pos=[{pos[0]:5.2f}, {pos[1]:5.2f}, {pos[2]:5.2f}]m | "
                      f"Vel=[{vel[0]:4.2f}, {vel[1]:4.2f}, {vel[2]:4.2f}]m/s | "
                      f"Att=[{np.degrees(euler[0]):5.1f}°, {np.degrees(euler[1]):5.1f}°, {np.degrees(euler[2]):5.1f}°] | "
                      f"HoverEst={hover_est:.3f}")

            # Maintain 100Hz loop rate
            loop_time = time.time() - loop_start
            if loop_time < 0.01:
                time.sleep(0.01 - loop_time)

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
        print(f"  Average loop rate: {loop_count / (time.time() - start_time):.1f} Hz")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='PyBullet flight simulation with cascaded control',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--duration', type=int, default=60,
                        help='Simulation duration in seconds (default: 60)')
    parser.add_argument('--gui', action='store_true',
                        help='Show PyBullet GUI window')
    parser.add_argument('--mode', type=str,
                        choices=['hover', 'circle', 'figure8', 'square'],
                        default='hover',
                        help='Flight pattern mode (default: hover)')
    parser.add_argument('--record', type=str, metavar='FILENAME',
                        help='Record video to MP4 file')

    args = parser.parse_args()

    run_cascaded_demo(
        duration=args.duration,
        gui=args.gui,
        flight_mode=args.mode
    )
