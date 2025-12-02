"""
PyBullet Flight Simulation with Data Logging

Demonstrates flight controller with comprehensive data logging for analysis.
Compatible with QGroundControl via MAVLink telemetry.
"""

import sys
import time
import numpy as np

# Import HAL and set platform
from src.hal.hal import set_platform
from src.hal.hal_pybullet import PyBulletPlatform
from src.data_logger import FlightDataLogger

# Check if PyBullet and MAVLink are available
try:
    import pybullet as p
    PYBULLET_AVAILABLE = True
except ImportError:
    print("PyBullet not available. Install with: conda install -c conda-forge pybullet")
    PYBULLET_AVAILABLE = False
    sys.exit(1)

try:
    from telemetry.mavlink_telemetry import MAVLinkTelemetry
    MAVLINK_AVAILABLE = True
except ImportError:
    print("MAVLink not available. Install with: pip install pymavlink")
    MAVLINK_AVAILABLE = False


class SimplePID:
    """Simple PID controller for simulation"""
    def __init__(self, kp, ki, kd, output_min=-500, output_max=500):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()
    
    def update(self, setpoint, measured):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.001
        
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = np.clip(output, self.output_min, self.output_max)
        
        self.last_error = error
        self.last_time = current_time
        
        return output


def run_logged_flight(duration=30.0, enable_telemetry=True, log_name="pybullet_flight"):
    """
    Run flight simulation with data logging and optional MAVLink telemetry.
    
    Args:
        duration: Flight duration in seconds
        enable_telemetry: Enable MAVLink telemetry (for QGroundControl)
        log_name: Base name for log file
    """
    print("\n=== PyBullet Flight Simulation with Data Logging ===\n")
    
    # Initialize PyBullet platform
    platform = PyBulletPlatform(gui=True, start_orientation=[0, 0, 0])
    set_platform(platform)
    
    # Initialize data logger
    logger = FlightDataLogger()
    log_file = logger.start_logging(log_name)
    print(f"Logging to: {log_file}")
    
    # Initialize MAVLink telemetry if available and enabled
    telemetry = None
    if MAVLINK_AVAILABLE and enable_telemetry:
        telemetry = MAVLinkTelemetry(connection_string='udpout:localhost:14550')
        telemetry.start()
        print("MAVLink telemetry started on UDP port 14550")
        print("Connect QGroundControl to: UDP, port 14550")
    
    # Initialize PID controllers
    roll_pid = SimplePID(kp=1.5, ki=0.1, kd=0.8)
    pitch_pid = SimplePID(kp=1.5, ki=0.1, kd=0.8)
    yaw_pid = SimplePID(kp=2.0, ki=0.05, kd=1.0)
    
    # Flight parameters
    base_throttle = 1500
    start_time = time.time()
    loop_count = 0
    
    print("\nFlight sequence:")
    print("  0-5s:   Throttle up")
    print("  5-15s:  Hover and stabilize")
    print("  15-20s: Roll maneuver")
    print("  20-25s: Pitch maneuver")
    print("  25-30s: Yaw maneuver")
    print("  30s+:   Throttle down\n")
    
    try:
        while time.time() - start_time < duration:
            loop_start = time.time()
            elapsed = loop_start - start_time
            
            # Read IMU data
            gyro = platform.read_gyro()
            accel = platform.read_accel()
            attitude = platform.get_orientation()
            
            # Flight sequence with maneuvers
            if elapsed < 5:
                # Throttle up
                throttle = 1000 + int((elapsed / 5.0) * 500)
                roll_setpoint = 0
                pitch_setpoint = 0
                yaw_setpoint = 0
            elif elapsed < 15:
                # Hover
                throttle = base_throttle
                roll_setpoint = 0
                pitch_setpoint = 0
                yaw_setpoint = 0
            elif elapsed < 20:
                # Roll maneuver
                throttle = base_throttle
                roll_setpoint = 10 * np.sin((elapsed - 15) * 2 * np.pi / 5)
                pitch_setpoint = 0
                yaw_setpoint = 0
            elif elapsed < 25:
                # Pitch maneuver
                throttle = base_throttle
                roll_setpoint = 0
                pitch_setpoint = 10 * np.sin((elapsed - 20) * 2 * np.pi / 5)
                yaw_setpoint = 0
            elif elapsed < 30:
                # Yaw maneuver
                throttle = base_throttle
                roll_setpoint = 0
                pitch_setpoint = 0
                yaw_setpoint = 15 * np.sin((elapsed - 25) * 2 * np.pi / 5)
            else:
                # Throttle down
                throttle = base_throttle - int(((elapsed - 30) / 5.0) * 500)
                roll_setpoint = 0
                pitch_setpoint = 0
                yaw_setpoint = 0
            
            # Calculate PID outputs
            roll_correction = roll_pid.update(roll_setpoint, attitude[0])
            pitch_correction = pitch_pid.update(pitch_setpoint, attitude[1])
            yaw_correction = yaw_pid.update(yaw_setpoint, attitude[2])
            
            # Calculate motor commands (hexacopter mixing)
            motors = [throttle] * 6
            motors[0] += int(-pitch_correction + yaw_correction)
            motors[1] += int(pitch_correction + yaw_correction)
            motors[2] += int(-roll_correction - yaw_correction)
            motors[3] += int(roll_correction - yaw_correction)
            motors[4] += int(roll_correction + yaw_correction)
            motors[5] += int(-roll_correction + yaw_correction)
            
            # Clip to valid range
            motors = [np.clip(m, 1000, 2000) for m in motors]
            
            # Send motor commands
            for i, motor_cmd in enumerate(motors):
                platform.set_motor(i, motor_cmd)
            
            # Step simulation
            platform.step()
            
            # Log data
            log_data = {
                'gyro_x': gyro[0],
                'gyro_y': gyro[1],
                'gyro_z': gyro[2],
                'accel_x': accel[0],
                'accel_y': accel[1],
                'accel_z': accel[2],
                'roll': attitude[0],
                'pitch': attitude[1],
                'yaw': attitude[2],
                'rc_throttle': throttle,
                'rc_roll': 1500,
                'rc_pitch': 1500,
                'rc_yaw': 1500,
                'pid_roll_output': roll_correction,
                'pid_pitch_output': pitch_correction,
                'pid_yaw_output': yaw_correction,
                'motor_0': motors[0],
                'motor_1': motors[1],
                'motor_2': motors[2],
                'motor_3': motors[3],
                'motor_4': motors[4],
                'motor_5': motors[5],
                'battery_voltage': 11.1,
                'cpu_usage': 0,
                'loop_rate': 240
            }
            logger.log_data(log_data)
            
            # Send telemetry
            if telemetry:
                position = platform.get_position()
                velocity = platform.get_velocity()
                
                telemetry.send_heartbeat()
                telemetry.send_attitude(
                    roll=np.radians(attitude[0]),
                    pitch=np.radians(attitude[1]),
                    yaw=np.radians(attitude[2]),
                    rollspeed=np.radians(gyro[0]),
                    pitchspeed=np.radians(gyro[1]),
                    yawspeed=np.radians(gyro[2])
                )
                telemetry.send_local_position(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                    vx=velocity[0],
                    vy=velocity[1],
                    vz=velocity[2]
                )
                telemetry.send_sys_status(
                    battery_voltage=11100,
                    battery_current=1500,
                    battery_remaining=85
                )
                telemetry.send_servo_output(motors)
            
            loop_count += 1
            
            # Print status every second
            if loop_count % 240 == 0:
                print(f"t={elapsed:.1f}s: Roll={attitude[0]:6.2f}° Pitch={attitude[1]:6.2f}° "
                      f"Yaw={attitude[2]:6.2f}° Alt={platform.get_position()[2]:.2f}m "
                      f"Records={logger.log_count}")
            
            # Maintain 240 Hz loop rate
            loop_time = time.time() - loop_start
            if loop_time < 1.0/240:
                time.sleep(1.0/240 - loop_time)
    
    except KeyboardInterrupt:
        print("\n\nFlight interrupted by user")
    
    finally:
        # Stop logging
        logger.stop_logging()
        
        # Stop telemetry
        if telemetry:
            telemetry.stop()
            print("MAVLink telemetry stopped")
        
        # Disconnect PyBullet
        p.disconnect()
        
        print(f"\n✓ Flight complete!")
        print(f"✓ Log saved to: {log_file}")
        print(f"\nAnalyze with:")
        print(f"  - PlotJuggler: plotjuggler {log_file}")
        print(f"  - Python pandas: pd.read_csv('{log_file}')")
        print(f"  - Excel/LibreOffice: Open CSV file")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='PyBullet flight simulation with logging')
    parser.add_argument('--duration', type=float, default=30.0, help='Flight duration in seconds')
    parser.add_argument('--no-telemetry', action='store_true', help='Disable MAVLink telemetry')
    parser.add_argument('--log-name', type=str, default='pybullet_flight', help='Base name for log file')
    
    args = parser.parse_args()
    
    run_logged_flight(
        duration=args.duration,
        enable_telemetry=not args.no_telemetry,
        log_name=args.log_name
    )
