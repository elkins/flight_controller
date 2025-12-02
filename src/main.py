"""
Flight Controller Main Module

This module implements a hexacopter flight controller using MPU6050 IMU,
PID control loops, RC receiver input, and ESC motor control.

Architecture:
- Cascaded PID control (stabilization + rate)
- 6 motors in hexacopter configuration
- 4-channel RC input (throttle, roll, pitch, yaw)
"""

from src.mpu6050 import MPU6050
from src.pid import PID
from src.rc import RC, map_range, wrap_180
from src.esc import ESC


# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================

class PIDConfig:
    """PID controller tuning parameters"""
    # Rate PIDs (inner loop - gyro rates)
    ROLL_RATE_P = 0.7
    ROLL_RATE_I = 1.0
    ROLL_RATE_IMAX = 50
    
    PITCH_RATE_P = 0.7
    PITCH_RATE_I = 1.0
    PITCH_RATE_IMAX = 50
    
    YAW_RATE_P = 2.7
    YAW_RATE_I = 1.0
    YAW_RATE_IMAX = 50
    
    # Stabilization PIDs (outer loop - angles)
    ROLL_STAB_P = 4.5
    PITCH_STAB_P = 4.5
    YAW_STAB_P = 10.0


class RCConfig:
    """RC receiver channel configuration"""
    THROTTLE_CHANNEL = 0
    ROLL_CHANNEL = 1
    PITCH_CHANNEL = 2
    YAW_CHANNEL = 3
    
    # RC input mapping
    RC_MIN = 995
    RC_MAX = 1945
    ANGLE_MIN = -45
    ANGLE_MAX = 45
    YAW_MIN = -180
    YAW_MAX = 180
    
    # Flight modes
    THROTTLE_ARM_THRESHOLD = 1200


class FlightConfig:
    """Flight control limits"""
    STAB_OUTPUT_LIMIT = 250
    YAW_STAB_OUTPUT_LIMIT = 360
    RATE_OUTPUT_LIMIT = 500
    YAW_DEADBAND = 5  # degrees
    
    # IMU calibration offset
    YAW_OFFSET = -6


# ============================================================================
# HARDWARE INITIALIZATION
# ============================================================================

class FlightController:
    """Main flight controller class"""
    
    def __init__(self):
        """Initialize all flight controller components"""
        # IMU
        self.mpu = MPU6050()
        self.mpu.dmpInitialize()
        self.mpu.setDMPEnabled(True)
        self.packet_size = self.mpu.dmpGetFIFOPacketSize()
        
        # Rate PIDs (inner loop)
        self.roll_rate_pid = PID(
            p=PIDConfig.ROLL_RATE_P,
            i=PIDConfig.ROLL_RATE_I,
            imax=PIDConfig.ROLL_RATE_IMAX
        )
        self.pitch_rate_pid = PID(
            p=PIDConfig.PITCH_RATE_P,
            i=PIDConfig.PITCH_RATE_I,
            imax=PIDConfig.PITCH_RATE_IMAX
        )
        self.yaw_rate_pid = PID(
            p=PIDConfig.YAW_RATE_P,
            i=PIDConfig.YAW_RATE_I,
            imax=PIDConfig.YAW_RATE_IMAX
        )
        
        # Stabilization PIDs (outer loop)
        self.roll_stab_pid = PID(p=PIDConfig.ROLL_STAB_P)
        self.pitch_stab_pid = PID(p=PIDConfig.PITCH_STAB_P)
        self.yaw_stab_pid = PID(p=PIDConfig.YAW_STAB_P)
        
        # RC Receiver
        self.rc_throttle = RC(RCConfig.THROTTLE_CHANNEL)
        self.rc_roll = RC(RCConfig.ROLL_CHANNEL)
        self.rc_pitch = RC(RCConfig.PITCH_CHANNEL)
        self.rc_yaw = RC(RCConfig.YAW_CHANNEL)
        
        # ESCs (6 motors for hexacopter)
        self.motors = [ESC(i) for i in range(6)]
        
        # State variables
        self.yaw_target = 0
    
    def reset_integrators(self):
        """Reset all PID integrators (when disarmed)"""
        self.roll_rate_pid.reset_I()
        self.pitch_rate_pid.reset_I()
        self.yaw_rate_pid.reset_I()
        self.roll_stab_pid.reset_I()
        self.pitch_stab_pid.reset_I()
        self.yaw_stab_pid.reset_I()
    
    def set_all_motors(self, throttle):
        """Set all motors to the same throttle value"""
        for motor in self.motors:
            motor.move(throttle)
    
    def run(self):
        """Main flight control loop"""
        while True:
            # Read IMU data
            imu_data = self._read_imu()
            if imu_data is None:
                continue
            
            yaw, roll, pitch, gyro_pitch, gyro_roll, gyro_yaw = imu_data
            
            # Read RC inputs
            rc_inputs = self._read_rc()
            
            # Check if armed (throttle above threshold)
            if rc_inputs['throttle'] > RCConfig.THROTTLE_ARM_THRESHOLD:
                self._armed_mode(yaw, roll, pitch, gyro_pitch, gyro_roll, gyro_yaw, rc_inputs)
            else:
                self._disarmed_mode(yaw, rc_inputs['throttle'])
    
    def _read_imu(self):
        """Read and process IMU data from MPU6050"""
        mpu_int_status = self.mpu.getIntStatus()
        fifo_count = self.mpu.getFIFOCount()
        
        # Check for FIFO overflow or invalid data
        if mpu_int_status < 2 or fifo_count == 1024:
            self.mpu.resetFIFO()
            print('FIFO overflow!')
            return None
        
        # Wait for complete packet
        while fifo_count < self.packet_size:
            fifo_count = self.mpu.getFIFOCount()
        
        # Read FIFO data
        fifo_count -= self.packet_size
        fifo_buffer = self.mpu.getFIFOBytes(self.packet_size)
        
        # Extract orientation data
        yaw, roll, pitch = self.mpu.dmpGetEuler(*self.mpu.dmpGetQuaternion(fifo_buffer))
        gyro_pitch, gyro_roll, gyro_yaw = self.mpu.dmpGetGyro(fifo_buffer)
        
        # Apply calibration offset
        yaw += FlightConfig.YAW_OFFSET
        
        return (yaw, roll, pitch, gyro_pitch, gyro_roll, gyro_yaw)
    
    def _read_rc(self):
        """Read RC receiver inputs"""
        throttle = self.rc_throttle.get_width()
        roll = map_range(
            self.rc_roll.get_width(),
            RCConfig.RC_MIN, RCConfig.RC_MAX,
            RCConfig.ANGLE_MIN, RCConfig.ANGLE_MAX
        )
        pitch = map_range(
            self.rc_pitch.get_width(),
            RCConfig.RC_MIN, RCConfig.RC_MAX,
            RCConfig.ANGLE_MIN, RCConfig.ANGLE_MAX
        )
        yaw = map_range(
            self.rc_yaw.get_width(),
            RCConfig.RC_MIN, RCConfig.RC_MAX,
            RCConfig.YAW_MIN, RCConfig.YAW_MAX
        )
        
        return {
            'throttle': throttle,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }
    
    def _armed_mode(self, yaw, roll, pitch, gyro_pitch, gyro_roll, gyro_yaw, rc_inputs):
        """Execute armed flight control mode"""
        # Stabilization PIDs (outer loop) - convert angle error to rate target
        roll_stab_out = self._clamp(
            self.roll_stab_pid.get_pid(rc_inputs['roll'] + roll, 1),
            -FlightConfig.STAB_OUTPUT_LIMIT,
            FlightConfig.STAB_OUTPUT_LIMIT
        )
        pitch_stab_out = self._clamp(
            self.pitch_stab_pid.get_pid(rc_inputs['pitch'] + pitch, 1),
            -FlightConfig.STAB_OUTPUT_LIMIT,
            FlightConfig.STAB_OUTPUT_LIMIT
        )
        yaw_stab_out = self._clamp(
            self.yaw_stab_pid.get_pid(wrap_180(self.yaw_target + yaw), 1),
            -FlightConfig.YAW_STAB_OUTPUT_LIMIT,
            FlightConfig.YAW_STAB_OUTPUT_LIMIT
        )
        
        # Check if pilot commanding yaw change
        if abs(rc_inputs['yaw']) > FlightConfig.YAW_DEADBAND:
            yaw_stab_out = rc_inputs['yaw']
            self.yaw_target = yaw
        
        # Rate PIDs (inner loop) - convert rate error to motor output
        roll_out = self._clamp(
            self.roll_rate_pid.get_pid(roll_stab_out + gyro_roll, 1),
            -FlightConfig.RATE_OUTPUT_LIMIT,
            FlightConfig.RATE_OUTPUT_LIMIT
        )
        pitch_out = self._clamp(
            self.pitch_rate_pid.get_pid(pitch_stab_out + gyro_pitch, 1),
            -FlightConfig.RATE_OUTPUT_LIMIT,
            FlightConfig.RATE_OUTPUT_LIMIT
        )
        yaw_out = self._clamp(
            self.yaw_rate_pid.get_pid(yaw_stab_out + gyro_yaw, 1),
            -FlightConfig.RATE_OUTPUT_LIMIT,
            FlightConfig.RATE_OUTPUT_LIMIT
        )
        
        # Override yaw output with direct RC command (for now)
        yaw_out = rc_inputs['yaw']
        
        # Mix outputs for hexacopter motor layout
        self._mix_motors(rc_inputs['throttle'], roll_out, pitch_out, yaw_out)
    
    def _disarmed_mode(self, yaw, throttle):
        """Execute disarmed mode - motors at low throttle"""
        self.set_all_motors(throttle)
        self.yaw_target = yaw
        self.reset_integrators()
    
    def _mix_motors(self, throttle, roll, pitch, yaw):
        """
        Mix throttle and control outputs for hexacopter motor layout
        
        Hexacopter motor layout (view from top, front is up):
                0
            5       1
            
            4       2
                3
        """
        # Motor mixing for hexacopter configuration
        self.motors[0].move(throttle - (0.866 * pitch) + (0.5 * roll) - yaw)
        self.motors[1].move(throttle - (0.866 * pitch) - (0.5 * roll) + yaw)
        self.motors[2].move(throttle - roll - yaw)
        self.motors[3].move(throttle + (0.866 * pitch) - (0.5 * roll) + yaw)
        self.motors[4].move(throttle + (0.866 * pitch) + (0.5 * roll) - yaw)
        self.motors[5].move(throttle + roll + yaw)
    
    @staticmethod
    def _clamp(value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min(value, max_val), min_val)


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main entry point for flight controller"""
    controller = FlightController()
    controller.run()


if __name__ == '__main__':
    main()

