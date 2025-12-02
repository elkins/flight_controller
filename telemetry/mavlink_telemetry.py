"""
MAVLink Telemetry Integration for Hexacopter Flight Controller

Provides real-time telemetry streaming to ground control stations (QGroundControl,
Mission Planner) using the industry-standard MAVLink protocol.

Features:
- HEARTBEAT: System health and status
- ATTITUDE: Roll, pitch, yaw, angular rates
- LOCAL_POSITION_NED: Position and velocity in NED frame
- SYS_STATUS: Battery, CPU load, sensors
- RC_CHANNELS: RC input monitoring
- SERVO_OUTPUT_RAW: Motor PWM outputs
- STATUSTEXT: Debug messages

Compatible with MAVLink 2.0 (common message set).

Usage:
    from mavlink_telemetry import MAVLinkTelemetry
    
    # Initialize with UART port
    mavlink = MAVLinkTelemetry(port='/dev/ttyUSB0', baudrate=57600)
    
    # In main loop:
    mavlink.send_heartbeat()
    mavlink.send_attitude(roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
    mavlink.send_position(x, y, z, vx, vy, vz)
    mavlink.handle_commands()  # Process incoming commands
"""

from pymavlink import mavutil
import time
import math


class MAVLinkTelemetry:
    """MAVLink telemetry interface for flight controller"""
    
    # MAVLink constants
    MAV_TYPE_HEXAROTOR = 13
    MAV_AUTOPILOT_GENERIC = 0
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_STATE_STANDBY = 3
    MAV_STATE_ACTIVE = 4
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=57600, system_id=1, component_id=1):
        """
        Initialize MAVLink connection
        
        Args:
            port: Serial port (e.g. '/dev/ttyUSB0', '/dev/ttyAMA0', 'COM3')
            baudrate: Serial baud rate (default: 57600, common: 9600, 57600, 115200)
            system_id: MAVLink system ID (1-255, default: 1)
            component_id: MAVLink component ID (default: 1 for autopilot)
        """
        self.port = port
        self.baudrate = baudrate
        self.system_id = system_id
        self.component_id = component_id
        
        # Connection state
        self.connection = None
        self.connected = False
        self.last_heartbeat = 0
        
        # Flight state
        self.armed = False
        self.custom_mode = 0
        
        # Timing
        self.boot_time = time.time()
        self.last_status_send = 0
        
        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.commands_received = 0
        
        # Initialize connection
        self._connect()
    
    def _connect(self):
        """Establish MAVLink connection"""
        try:
            # Create connection (supports serial, UDP, TCP)
            self.connection = mavutil.mavlink_connection(
                self.port,
                baud=self.baudrate,
                source_system=self.system_id,
                source_component=self.component_id
            )
            self.connected = True
            print(f"✓ MAVLink connected: {self.port} @ {self.baudrate} baud")
            print(f"  System ID: {self.system_id}, Component ID: {self.component_id}")
        except Exception as e:
            print(f"✗ MAVLink connection failed: {e}")
            self.connected = False
    
    def get_time_boot_ms(self):
        """Get time since boot in milliseconds"""
        return int((time.time() - self.boot_time) * 1000)
    
    def send_heartbeat(self, armed=None):
        """
        Send HEARTBEAT message (required at 1Hz minimum)
        
        Args:
            armed: Override armed state (None to use internal state)
        """
        if not self.connected:
            return
        
        if armed is not None:
            self.armed = armed
        
        # Build base mode flags
        base_mode = self.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base_mode |= self.MAV_MODE_FLAG_SAFETY_ARMED
        
        # System state
        system_status = self.MAV_STATE_ACTIVE if self.armed else self.MAV_STATE_STANDBY
        
        try:
            self.connection.mav.heartbeat_send(
                type=self.MAV_TYPE_HEXAROTOR,
                autopilot=self.MAV_AUTOPILOT_GENERIC,
                base_mode=base_mode,
                custom_mode=self.custom_mode,
                system_status=system_status,
                mavlink_version=3  # MAVLink 2.0
            )
            self.packets_sent += 1
            self.last_heartbeat = time.time()
        except Exception as e:
            print(f"✗ Failed to send heartbeat: {e}")
    
    def send_attitude(self, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed):
        """
        Send ATTITUDE message
        
        Args:
            roll: Roll angle in radians (-pi to +pi)
            pitch: Pitch angle in radians (-pi to +pi)
            yaw: Yaw angle in radians (-pi to +pi)
            rollspeed: Roll angular speed in rad/s
            pitchspeed: Pitch angular speed in rad/s
            yawspeed: Yaw angular speed in rad/s
        """
        if not self.connected:
            return
        
        try:
            self.connection.mav.attitude_send(
                time_boot_ms=self.get_time_boot_ms(),
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                rollspeed=rollspeed,
                pitchspeed=pitchspeed,
                yawspeed=yawspeed
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send attitude: {e}")
    
    def send_position(self, x, y, z, vx, vy, vz):
        """
        Send LOCAL_POSITION_NED message
        
        Args:
            x: X position in meters (North)
            y: Y position in meters (East)
            z: Z position in meters (Down, negative = up)
            vx: X velocity in m/s
            vy: Y velocity in m/s
            vz: Z velocity in m/s
        """
        if not self.connected:
            return
        
        try:
            self.connection.mav.local_position_ned_send(
                time_boot_ms=self.get_time_boot_ms(),
                x=x,
                y=y,
                z=z,
                vx=vx,
                vy=vy,
                vz=vz
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send position: {e}")
    
    def send_gps(self, lat, lon, alt, vx=0, vy=0, vz=0, hdg=65535, fix_type=3, satellites_visible=10):
        """
        Send GPS_RAW_INT message (required for QGC altitude display)
        
        Args:
            lat: Latitude in degrees * 1e7 (e.g., 37.7749 * 1e7)
            lon: Longitude in degrees * 1e7 (e.g., -122.4194 * 1e7)
            alt: Altitude in mm above MSL
            vx: GPS ground speed X (cm/s)
            vy: GPS ground speed Y (cm/s)
            vz: GPS ground speed Z (cm/s)
            hdg: Heading in degrees * 100 (65535 = unknown)
            fix_type: GPS fix type (0=no fix, 3=3D fix)
            satellites_visible: Number of visible satellites
        """
        if not self.connected:
            return
        
        try:
            self.connection.mav.gps_raw_int_send(
                time_usec=self.get_time_boot_ms() * 1000,
                fix_type=fix_type,
                lat=int(lat),
                lon=int(lon),
                alt=int(alt),
                eph=100,  # GPS HDOP horizontal dilution of position
                epv=100,  # GPS VDOP vertical dilution of position
                vel=int(math.sqrt(vx*vx + vy*vy)),  # Ground speed (cm/s)
                cog=hdg,  # Course over ground (degrees * 100)
                satellites_visible=satellites_visible
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send GPS: {e}")
    
    def send_global_position(self, lat, lon, alt, relative_alt, vx=0, vy=0, vz=0, hdg=65535):
        """
        Send GLOBAL_POSITION_INT message (required for QGC to display altitude/speed)
        
        Args:
            lat: Latitude in degrees * 1e7
            lon: Longitude in degrees * 1e7
            alt: Altitude above MSL in mm
            relative_alt: Altitude above home in mm
            vx: Ground speed X in cm/s
            vy: Ground speed Y in cm/s
            vz: Ground speed Z in cm/s
            hdg: Heading in degrees * 100
        """
        if not self.connected:
            return
        
        try:
            self.connection.mav.global_position_int_send(
                time_boot_ms=self.get_time_boot_ms(),
                lat=int(lat),
                lon=int(lon),
                alt=int(alt),
                relative_alt=int(relative_alt),
                vx=int(vx),
                vy=int(vy),
                vz=int(vz),
                hdg=int(hdg) if hdg != 65535 else 65535
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send global position: {e}")
    
    def send_vfr_hud(self, airspeed, groundspeed, heading, throttle, alt, climb):
        """
        Send VFR_HUD message (required for QGC speed displays)
        
        Args:
            airspeed: Current airspeed in m/s
            groundspeed: Current ground speed in m/s
            heading: Current heading in degrees (0-359, wraps at 360)
            throttle: Current throttle % (0-100)
            alt: Current altitude in meters (MSL)
            climb: Current climb rate in m/s
        """
        if not self.connected:
            return
        
        try:
            # Ensure heading is in valid range [0, 359]
            heading_int = int(heading) % 360
            throttle_int = max(0, min(100, int(throttle)))  # Clamp to 0-100
            
            self.connection.mav.vfr_hud_send(
                airspeed=airspeed,
                groundspeed=groundspeed,
                heading=heading_int,
                throttle=throttle_int,
                alt=alt,
                climb=climb
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send VFR HUD: {e} (hdg={heading_int}, thr={throttle_int}, alt={alt})")
    
    def send_sys_status(self, voltage_battery=0, current_battery=0, battery_remaining=-1,
                       cpu_load=0, sensors_health=0xFFFFFFFF):
        """
        Send SYS_STATUS message
        
        Args:
            voltage_battery: Battery voltage in mV (0 = unknown)
            current_battery: Battery current in cA (0 = unknown)
            battery_remaining: Battery remaining in % (0-100, -1 = unknown)
            cpu_load: CPU load in % (0-1000, where 1000 = 100%)
            sensors_health: Bitmask of sensor health (1 = healthy)
        """
        if not self.connected:
            return
        
        try:
            self.connection.mav.sys_status_send(
                onboard_control_sensors_present=sensors_health,
                onboard_control_sensors_enabled=sensors_health,
                onboard_control_sensors_health=sensors_health,
                load=cpu_load,
                voltage_battery=voltage_battery,
                current_battery=current_battery,
                battery_remaining=battery_remaining,
                drop_rate_comm=0,
                errors_comm=0,
                errors_count1=0,
                errors_count2=0,
                errors_count3=0,
                errors_count4=0
            )
            self.packets_sent += 1
            self.last_status_send = time.time()
        except Exception as e:
            print(f"✗ Failed to send sys_status: {e}")
    
    def send_rc_channels(self, channels, rssi=255):
        """
        Send RC_CHANNELS message (RC input from receiver)
        
        Args:
            channels: List of 18 RC channel values in microseconds (1000-2000)
                      Use 0 or 65535 for unused channels
            rssi: Receive signal strength (0-255, 255 = unknown)
        """
        if not self.connected:
            return
        
        # Ensure we have exactly 18 channels
        channels = list(channels) + [0] * (18 - len(channels))
        channels = channels[:18]
        
        try:
            self.connection.mav.rc_channels_send(
                time_boot_ms=self.get_time_boot_ms(),
                chancount=len([c for c in channels if c > 0]),
                chan1_raw=channels[0],
                chan2_raw=channels[1],
                chan3_raw=channels[2],
                chan4_raw=channels[3],
                chan5_raw=channels[4],
                chan6_raw=channels[5],
                chan7_raw=channels[6],
                chan8_raw=channels[7],
                chan9_raw=channels[8],
                chan10_raw=channels[9],
                chan11_raw=channels[10],
                chan12_raw=channels[11],
                chan13_raw=channels[12],
                chan14_raw=channels[13],
                chan15_raw=channels[14],
                chan16_raw=channels[15],
                chan17_raw=channels[16],
                chan18_raw=channels[17],
                rssi=rssi
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send rc_channels: {e}")
    
    def send_servo_output(self, servo_values, port=0):
        """
        Send SERVO_OUTPUT_RAW message (motor PWM outputs)
        
        Args:
            servo_values: List of up to 8 servo/motor PWM values (1000-2000 µs)
                         Use 0 for unused channels
            port: Servo port (0 = MAIN, 1 = AUX)
        """
        if not self.connected:
            return
        
        # Ensure we have 8 values (MAVLink 1.0 supports only 8 servos)
        servo_values = list(servo_values) + [0] * (8 - len(servo_values))
        servo_values = servo_values[:8]
        
        try:
            self.connection.mav.servo_output_raw_send(
                time_usec=self.get_time_boot_ms() * 1000,  # Convert to microseconds
                port=port,
                servo1_raw=servo_values[0],
                servo2_raw=servo_values[1],
                servo3_raw=servo_values[2],
                servo4_raw=servo_values[3],
                servo5_raw=servo_values[4],
                servo6_raw=servo_values[5],
                servo7_raw=servo_values[6],
                servo8_raw=servo_values[7]
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send servo_output: {e}")
    
    def send_statustext(self, text, severity=6):
        """
        Send STATUSTEXT message (debug/info message)
        
        Args:
            text: Status text (max 50 characters)
            severity: MAV_SEVERITY (0=EMERGENCY, 1=ALERT, 2=CRITICAL, 3=ERROR,
                     4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG)
        """
        if not self.connected:
            return
        
        try:
            # Truncate to 50 characters
            text = text[:50]
            self.connection.mav.statustext_send(
                severity=severity,
                text=text.encode('utf-8')
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send statustext: {e}")
    
    def handle_commands(self):
        """
        Process incoming MAVLink commands (non-blocking)
        
        Returns:
            dict or None: Command message if received, None otherwise
        """
        if not self.connected:
            return None
        
        try:
            # Non-blocking receive
            msg = self.connection.recv_match(blocking=False)
            
            if msg is not None:
                self.packets_received += 1
                msg_type = msg.get_type()
                
                # Handle specific message types
                if msg_type == 'COMMAND_LONG':
                    self.commands_received += 1
                    return self._handle_command_long(msg)
                elif msg_type == 'PARAM_REQUEST_LIST':
                    return {'type': 'PARAM_REQUEST_LIST'}
                elif msg_type == 'PARAM_SET':
                    return {'type': 'PARAM_SET', 'param': msg}
                elif msg_type == 'HEARTBEAT':
                    # Ground station heartbeat
                    return {'type': 'HEARTBEAT_GCS', 'system_id': msg.get_srcSystem()}
                
                return {'type': msg_type, 'message': msg}
        
        except Exception as e:
            print(f"✗ Error handling commands: {e}")
        
        return None
    
    def _handle_command_long(self, msg):
        """Handle COMMAND_LONG message"""
        command = msg.command
        
        # Common commands
        command_map = {
            400: 'ARM_DISARM',
            176: 'SET_MESSAGE_INTERVAL',
            511: 'REQUEST_MESSAGE',
            520: 'REQUEST_AUTOPILOT_CAPABILITIES',
            42501: 'SET_STANDARD_MODE'
        }
        
        command_name = command_map.get(command, f'UNKNOWN_{command}')
        
        return {
            'type': 'COMMAND_LONG',
            'command': command,
            'command_name': command_name,
            'param1': msg.param1,
            'param2': msg.param2,
            'param3': msg.param3,
            'param4': msg.param4,
            'param5': msg.param5,
            'param6': msg.param6,
            'param7': msg.param7,
            'target_system': msg.target_system,
            'target_component': msg.target_component
        }
    
    def send_command_ack(self, command, result):
        """
        Send COMMAND_ACK message
        
        Args:
            command: Command ID that was executed
            result: MAV_RESULT (0=ACCEPTED, 1=TEMP_REJECTED, 2=DENIED,
                   3=UNSUPPORTED, 4=FAILED, 5=IN_PROGRESS)
        """
        if not self.connected:
            return
        
        try:
            self.connection.mav.command_ack_send(
                command=command,
                result=result
            )
            self.packets_sent += 1
        except Exception as e:
            print(f"✗ Failed to send command_ack: {e}")
    
    def get_statistics(self):
        """Get telemetry statistics"""
        uptime = time.time() - self.boot_time
        return {
            'connected': self.connected,
            'uptime': uptime,
            'packets_sent': self.packets_sent,
            'packets_received': self.packets_received,
            'commands_received': self.commands_received,
            'last_heartbeat': time.time() - self.last_heartbeat if self.last_heartbeat else None
        }
    
    def close(self):
        """Close MAVLink connection"""
        if self.connection:
            self.connection.close()
            self.connected = False
            print("✓ MAVLink connection closed")


# Example usage and testing
if __name__ == '__main__':
    print("MAVLink Telemetry Test")
    print("=" * 50)
    
    # Example: Create connection (use UDP for testing without hardware)
    # For serial: mavlink = MAVLinkTelemetry(port='/dev/ttyUSB0', baudrate=57600)
    # For UDP: mavlink = MAVLinkTelemetry(port='udp:localhost:14550')
    
    # Simulated telemetry test
    print("\nSimulated telemetry stream:")
    print("In production, connect to:")
    print("  - Serial: MAVLinkTelemetry(port='/dev/ttyUSB0', baudrate=57600)")
    print("  - UDP: MAVLinkTelemetry(port='udp:localhost:14550')")
    print("  - TCP: MAVLinkTelemetry(port='tcp:localhost:5760')")
    print("\nThen in your main loop:")
    print("  mavlink.send_heartbeat(armed=True)")
    print("  mavlink.send_attitude(roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)")
    print("  mavlink.send_position(x, y, z, vx, vy, vz)")
    print("  cmd = mavlink.handle_commands()")
    print("\nGround stations:")
    print("  - QGroundControl: https://qgroundcontrol.com")
    print("  - Mission Planner: https://ardupilot.org/planner/")
    print("  - MAVProxy: pip install MAVProxy")
