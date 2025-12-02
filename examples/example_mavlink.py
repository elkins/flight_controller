"""
Example MAVLink Integration for Hexacopter Flight Controller

Demonstrates how to integrate MAVLink telemetry with the flight controller.
This example can be run with either hardware (PyBoard) or simulators (PyBullet).

Usage:
    # Hardware (PyBoard with telemetry radio)
    python example_mavlink.py --hardware --port /dev/ttyUSB0
    
    # PyBullet simulator (connect QGroundControl to udp://localhost:14550)
    python example_mavlink.py --simulator pybullet
    
    # Enhanced physics simulator
    python example_mavlink.py --simulator enhanced

Connect QGroundControl to:
- Serial: /dev/ttyUSB0 @ 57600 baud
- UDP: udp://localhost:14550
"""

import time
import math
import argparse


def example_hardware():
    """Example with hardware (PyBoard)"""
    print("=" * 60)
    print("MAVLink Hardware Integration Example")
    print("=" * 60)
    
    try:
        from hal_pyboard import PyBoardHAL
        from mavlink_telemetry import MAVLinkTelemetry
    except ImportError as e:
        print(f"✗ Import failed: {e}")
        print("\nRequired components:")
        print("  - hal_pyboard.py (for PyBoard hardware)")
        print("  - mavlink_telemetry.py (MAVLink support)")
        print("  - pymavlink (pip install pymavlink)")
        return
    
    # Initialize hardware
    print("\n[1/3] Initializing hardware...")
    try:
        hal = PyBoardHAL()
        print("✓ PyBoard HAL initialized")
    except Exception as e:
        print(f"✗ Hardware initialization failed: {e}")
        return
    
    # Initialize MAVLink telemetry
    print("\n[2/3] Initializing MAVLink...")
    try:
        mavlink = MAVLinkTelemetry(port='/dev/ttyUSB0', baudrate=57600)
        print("✓ MAVLink telemetry initialized")
        print(f"  Port: {mavlink.port}")
        print(f"  Baud: {mavlink.baudrate}")
        print(f"  System ID: {mavlink.system_id}")
    except Exception as e:
        print(f"✗ MAVLink initialization failed: {e}")
        print("\nTroubleshooting:")
        print("  1. Check telemetry radio connection")
        print("  2. Verify port name: ls /dev/tty*")
        print("  3. Check permissions: sudo chmod 666 /dev/ttyUSB0")
        return
    
    # Main telemetry loop
    print("\n[3/3] Starting telemetry stream...")
    print("Connect QGroundControl to your telemetry radio")
    print("Press Ctrl+C to stop\n")
    
    armed = False
    loop_count = 0
    
    try:
        while True:
            loop_start = time.time()
            
            # Read sensors
            imu_data = hal.read_imu()
            rc_data = hal.read_rc()
            
            # Simple attitude estimation (for demo - use real filter in production)
            # This is placeholder - real implementation needs IMU fusion
            roll = math.atan2(imu_data['accel_y'], imu_data['accel_z'])
            pitch = math.atan2(-imu_data['accel_x'], 
                              math.sqrt(imu_data['accel_y']**2 + imu_data['accel_z']**2))
            yaw = 0.0  # Need magnetometer for yaw
            
            # Angular rates from gyro
            rollspeed = imu_data['gyro_x']
            pitchspeed = imu_data['gyro_y']
            yawspeed = imu_data['gyro_z']
            
            # Position (placeholder - need GPS or position estimator)
            x, y, z = 0.0, 0.0, 0.0
            vx, vy, vz = 0.0, 0.0, 0.0
            
            # Motor PWM outputs (placeholder)
            motor_pwm = [1500] * 6  # Neutral for all 6 motors
            
            # Send MAVLink telemetry
            mavlink.send_heartbeat(armed=armed)
            mavlink.send_attitude(roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
            mavlink.send_position(x, y, z, vx, vy, vz)
            mavlink.send_rc_channels(rc_data['channels'])
            mavlink.send_servo_output(motor_pwm)
            
            # Send system status every 1 second
            if loop_count % 50 == 0:
                mavlink.send_sys_status(
                    voltage_battery=11800,  # 11.8V in mV
                    current_battery=1500,   # 15.0A in cA
                    battery_remaining=85    # 85%
                )
            
            # Handle incoming commands
            cmd = mavlink.handle_commands()
            if cmd:
                if cmd.get('command_name') == 'ARM_DISARM':
                    armed = (cmd['param1'] == 1.0)
                    mavlink.send_command_ack(cmd['command'], result=0)
                    print(f"  → {'ARMED' if armed else 'DISARMED'}")
            
            # Status update
            if loop_count % 50 == 0:
                stats = mavlink.get_statistics()
                print(f"  Loop {loop_count}: "
                      f"TX={stats['packets_sent']} "
                      f"RX={stats['packets_received']} "
                      f"CMD={stats['commands_received']}")
            
            loop_count += 1
            
            # Maintain 50Hz loop rate
            elapsed = time.time() - loop_start
            if elapsed < 0.02:
                time.sleep(0.02 - elapsed)
    
    except KeyboardInterrupt:
        print("\n\n✓ Telemetry stopped")
        stats = mavlink.get_statistics()
        print("\nSession statistics:")
        print(f"  Runtime: {stats['uptime']:.1f} seconds")
        print(f"  Packets sent: {stats['packets_sent']}")
        print(f"  Packets received: {stats['packets_received']}")
        print(f"  Commands received: {stats['commands_received']}")
    
    finally:
        mavlink.close()


def example_pybullet():
    """Example with PyBullet simulator"""
    print("=" * 60)
    print("MAVLink PyBullet Simulator Integration Example")
    print("=" * 60)
    
    try:
        from hal_pybullet import PyBulletHAL
        from mavlink_telemetry import MAVLinkTelemetry
    except ImportError as e:
        print(f"✗ Import failed: {e}")
        print("\nRequired components:")
        print("  - hal_pybullet.py (PyBullet simulator)")
        print("  - mavlink_telemetry.py (MAVLink support)")
        print("  - pymavlink (pip install pymavlink)")
        print("  - pybullet (requires Python ≤3.12)")
        return
    
    # Initialize simulator
    print("\n[1/3] Initializing PyBullet simulator...")
    try:
        hal = PyBulletHAL(gui=True)
        print("✓ PyBullet simulator initialized")
    except Exception as e:
        print(f"✗ Simulator initialization failed: {e}")
        return
    
    # Initialize MAVLink (UDP for easy GCS connection)
    print("\n[2/3] Initializing MAVLink over UDP...")
    try:
        mavlink = MAVLinkTelemetry(port='udp:0.0.0.0:14550', system_id=1)
        print("✓ MAVLink telemetry initialized")
        print("  Protocol: UDP")
        print("  Port: 14550")
        print("  System ID: 1")
    except Exception as e:
        print(f"✗ MAVLink initialization failed: {e}")
        return
    
    # Main simulation loop
    print("\n[3/3] Starting simulation with telemetry...")
    print("Connect QGroundControl to: udp://localhost:14550")
    print("Press Ctrl+C to stop\n")
    
    armed = False
    loop_count = 0
    
    try:
        while hal.step():
            # Get simulated state
            state = hal.get_state()
            
            # Send MAVLink telemetry
            mavlink.send_heartbeat(armed=armed)
            mavlink.send_attitude(
                state['roll'], state['pitch'], state['yaw'],
                state['p'], state['q'], state['r']
            )
            mavlink.send_position(
                state['x'], state['y'], state['z'],
                state['vx'], state['vy'], state['vz']
            )
            
            # Motor outputs
            motor_pwm = [int(pwm * 1000 + 1000) for pwm in state.get('motor_speeds', [0.5]*6)]
            mavlink.send_servo_output(motor_pwm)
            
            # System status every 1 second
            if loop_count % 240 == 0:  # 240Hz sim, so every second
                mavlink.send_sys_status(
                    voltage_battery=12600,
                    current_battery=2000,
                    battery_remaining=90
                )
            
            # Handle commands
            cmd = mavlink.handle_commands()
            if cmd:
                if cmd.get('command_name') == 'ARM_DISARM':
                    armed = (cmd['param1'] == 1.0)
                    mavlink.send_command_ack(cmd['command'], result=0)
                    mavlink.send_statustext(f"{'ARMED' if armed else 'DISARMED'}", severity=6)
            
            # Status update
            if loop_count % 240 == 0:
                stats = mavlink.get_statistics()
                print(f"  Sim step {loop_count}: "
                      f"Alt={-state['z']:.2f}m "
                      f"TX={stats['packets_sent']} "
                      f"RX={stats['packets_received']}")
            
            loop_count += 1
    
    except KeyboardInterrupt:
        print("\n\n✓ Simulation stopped")
        stats = mavlink.get_statistics()
        print("\nSession statistics:")
        print(f"  Runtime: {stats['uptime']:.1f} seconds")
        print(f"  Simulation steps: {loop_count}")
        print(f"  Packets sent: {stats['packets_sent']}")
        print(f"  Packets received: {stats['packets_received']}")
        print(f"  Commands received: {stats['commands_received']}")
    
    finally:
        mavlink.close()
        hal.close()


def example_enhanced():
    """Example with enhanced physics simulator"""
    print("=" * 60)
    print("MAVLink Enhanced Physics Simulator Integration Example")
    print("=" * 60)
    
    try:
        from hal_pybullet_enhanced import EnhancedPyBulletHAL
        from mavlink_telemetry import MAVLinkTelemetry
    except ImportError as e:
        print(f"✗ Import failed: {e}")
        print("\nRequired components:")
        print("  - hal_pybullet_enhanced.py (enhanced physics)")
        print("  - mavlink_telemetry.py (MAVLink support)")
        print("  - pymavlink (pip install pymavlink)")
        print("  - pybullet (requires Python ≤3.12)")
        return
    
    print("\n[1/3] Initializing enhanced physics simulator...")
    try:
        hal = EnhancedPyBulletHAL(gui=True)
        print("✓ Enhanced physics simulator initialized")
        print("  Ground effect: ENABLED")
        print("  Aerodynamic drag: ENABLED")
        print("  Validated coefficients: Crazyflie 2.x")
    except Exception as e:
        print(f"✗ Simulator initialization failed: {e}")
        return
    
    print("\n[2/3] Initializing MAVLink over UDP...")
    try:
        mavlink = MAVLinkTelemetry(port='udp:0.0.0.0:14550', system_id=1)
        print("✓ MAVLink telemetry initialized")
    except Exception as e:
        print(f"✗ MAVLink initialization failed: {e}")
        return
    
    print("\n[3/3] Starting high-fidelity simulation...")
    print("Connect QGroundControl to: udp://localhost:14550")
    print("Watch for:")
    print("  - Ground effect near landing")
    print("  - Realistic drag at higher speeds")
    print("  - Accurate motor dynamics")
    print("\nPress Ctrl+C to stop\n")
    
    armed = False
    loop_count = 0
    
    try:
        while hal.step():
            state = hal.get_state()
            
            # Full telemetry suite
            mavlink.send_heartbeat(armed=armed)
            mavlink.send_attitude(
                state['roll'], state['pitch'], state['yaw'],
                state['p'], state['q'], state['r']
            )
            mavlink.send_position(
                state['x'], state['y'], state['z'],
                state['vx'], state['vy'], state['vz']
            )
            
            motor_pwm = [int(pwm * 1000 + 1000) for pwm in state.get('motor_speeds', [0.5]*6)]
            mavlink.send_servo_output(motor_pwm)
            
            if loop_count % 240 == 0:
                mavlink.send_sys_status(
                    voltage_battery=12400,
                    current_battery=2500,
                    battery_remaining=85
                )
                
                # Send enhanced physics status
                altitude = -state['z']
                if altitude < 0.5:
                    mavlink.send_statustext(f"Ground effect active (alt={altitude:.2f}m)", severity=6)
            
            cmd = mavlink.handle_commands()
            if cmd:
                if cmd.get('command_name') == 'ARM_DISARM':
                    armed = (cmd['param1'] == 1.0)
                    mavlink.send_command_ack(cmd['command'], result=0)
            
            if loop_count % 240 == 0:
                stats = mavlink.get_statistics()
                print(f"  Enhanced sim {loop_count}: "
                      f"Alt={-state['z']:.2f}m "
                      f"Vel={math.sqrt(state['vx']**2 + state['vy']**2 + state['vz']**2):.2f}m/s "
                      f"TX={stats['packets_sent']}")
            
            loop_count += 1
    
    except KeyboardInterrupt:
        print("\n\n✓ Simulation stopped")
    
    finally:
        mavlink.close()
        hal.close()


def main():
    parser = argparse.ArgumentParser(
        description='MAVLink integration examples for hexacopter flight controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Hardware with telemetry radio
  %(prog)s --hardware --port /dev/ttyUSB0
  
  # PyBullet simulator
  %(prog)s --simulator pybullet
  
  # Enhanced physics simulator
  %(prog)s --simulator enhanced

Ground Control Station Setup:
  QGroundControl:
    - Serial: /dev/ttyUSB0 @ 57600 baud
    - UDP: udp://localhost:14550
  
  MAVProxy:
    - mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600
    - mavproxy.py --master=udp:127.0.0.1:14550
        """
    )
    
    parser.add_argument('--hardware', action='store_true',
                       help='Run with hardware (PyBoard)')
    parser.add_argument('--simulator', choices=['pybullet', 'enhanced'],
                       help='Run with simulator')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port for hardware (default: /dev/ttyUSB0)')
    
    args = parser.parse_args()
    
    # Run appropriate example
    if args.hardware:
        example_hardware()
    elif args.simulator == 'pybullet':
        example_pybullet()
    elif args.simulator == 'enhanced':
        example_enhanced()
    else:
        parser.print_help()
        print("\n")
        print("Quick Start:")
        print("  1. Install pymavlink: pip install pymavlink")
        print("  2. Choose mode:")
        print("     - Hardware: python example_mavlink.py --hardware")
        print("     - Simulator: python example_mavlink.py --simulator pybullet")
        print("  3. Open QGroundControl and connect")


if __name__ == '__main__':
    main()
