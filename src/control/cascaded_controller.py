"""
Cascaded PID Controller for Multicopter Control

Architecture (from outermost to innermost):
    Position → Velocity → Attitude → Rate → Motors

Based on:
- gym-pybullet-drones (https://github.com/utiasDSL/gym-pybullet-drones)
- PX4 Autopilot (https://github.com/PX4/PX4-Autopilot)
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import Tuple, Union
from .quaternion_math import (
    quaternion_error, quaternion_error_to_angular_velocity,
    euler_to_quaternion, quaternion_to_euler, pybullet_to_quaternion
)


@dataclass
class ControlSetpoint:
    """Multi-level setpoint for cascaded control"""
    # Position setpoint (outermost)
    position: np.ndarray = None  # [x, y, z] in meters

    # Velocity setpoint (can bypass position loop)
    velocity: np.ndarray = None  # [vx, vy, vz] in m/s

    # Attitude setpoint (can bypass position + velocity loops)
    # Can be either Euler angles or quaternion
    attitude: np.ndarray = None  # [roll, pitch, yaw] in radians OR [w,x,y,z] quaternion
    attitude_quaternion: np.ndarray = None  # [w, x, y, z] quaternion (preferred)

    # Rate setpoint (can bypass all outer loops)
    rate: np.ndarray = None      # [roll_rate, pitch_rate, yaw_rate] in rad/s

    # Direct thrust command (altitude control)
    thrust: float = None          # Normalized [0, 1]

    # Yaw setpoint (for position/velocity modes)
    yaw: float = 0.0              # radians


class SimplePID:
    """Simple PID controller with anti-windup"""
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0), integral_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits if integral_limits else output_limits

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def update(self, setpoint, measurement, dt=None):
        """Update PID controller"""
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

        if dt <= 0:
            dt = 0.001

        error = setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1])
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # Compute output
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        self.last_error = error

        return output

    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()


class RateController:
    """
    Rate controller (innermost loop, ~100-400 Hz).
    Controls angular velocities (rad/s) using PID.

    Based on PX4 rate controller architecture.
    """
    def __init__(self):
        # PID gains for hexacopter - CONSERVATIVE for stability
        # These are scaled DOWN significantly for the simulation environment
        self.roll_rate_pid = SimplePID(
            kp=0.03, ki=0.01, kd=0.001,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
        )
        self.pitch_rate_pid = SimplePID(
            kp=0.03, ki=0.01, kd=0.001,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
        )
        # Yaw: Even more conservative
        self.yaw_rate_pid = SimplePID(
            kp=0.02, ki=0.005, kd=0.0,
            output_limits=(-0.2, 0.2),
            integral_limits=(-0.05, 0.05)
        )

    def update(self, rate_setpoint: np.ndarray, current_rate: np.ndarray, dt: float) -> np.ndarray:
        """
        Args:
            rate_setpoint: [p, q, r] desired rates in rad/s
            current_rate: [p, q, r] measured rates in rad/s
            dt: Time step in seconds

        Returns:
            torques: [tau_roll, tau_pitch, tau_yaw] normalized torques [-1, 1]
        """
        tau_roll = self.roll_rate_pid.update(rate_setpoint[0], current_rate[0], dt)
        tau_pitch = self.pitch_rate_pid.update(rate_setpoint[1], current_rate[1], dt)
        tau_yaw = self.yaw_rate_pid.update(rate_setpoint[2], current_rate[2], dt)

        return np.array([tau_roll, tau_pitch, tau_yaw])

    def reset(self):
        """Reset all PIDs"""
        self.roll_rate_pid.reset()
        self.pitch_rate_pid.reset()
        self.yaw_rate_pid.reset()


class AttitudeController:
    """
    Quaternion-based attitude controller (P-only).
    Converts attitude errors to rate setpoints using quaternion math.

    Uses quaternions to avoid gimbal lock and provide smooth control.
    Based on PX4 attitude controller architecture.
    """
    def __init__(self):
        # P gain: unified for all axes (can be tuned individually if needed)
        self.kp = 3.0  # Attitude error → rate setpoint

        # Rate limits (safety) - reduced for stability
        self.max_rate_roll = np.deg2rad(60)    # 60 deg/s (gentle)
        self.max_rate_pitch = np.deg2rad(60)
        self.max_rate_yaw = np.deg2rad(45)     # 45 deg/s yaw

    def update(self, att_setpoint: Union[np.ndarray, None], current_att_quat: np.ndarray,
               att_setpoint_quat: Union[np.ndarray, None] = None) -> np.ndarray:
        """
        Args:
            att_setpoint: [roll, pitch, yaw] desired attitude in radians (optional if using quaternion)
            current_att_quat: [w, x, y, z] current attitude quaternion
            att_setpoint_quat: [w, x, y, z] desired attitude quaternion (preferred)

        Returns:
            rate_setpoint: [p, q, r] rate commands in rad/s
        """
        # Convert Euler to quaternion if needed
        if att_setpoint_quat is None:
            if att_setpoint is not None:
                att_setpoint_quat = euler_to_quaternion(att_setpoint)
            else:
                # No setpoint - return zero rates
                return np.zeros(3)

        # Compute quaternion error: q_error = q_desired * q_current^(-1)
        q_error = quaternion_error(att_setpoint_quat, current_att_quat)

        # Convert quaternion error to angular velocity using P control
        # This is the PX4 method: extracts rotation axis and applies proportional gain
        rate_setpoint = quaternion_error_to_angular_velocity(q_error, kp=self.kp)

        # Apply rate limits
        rate_setpoint[0] = np.clip(rate_setpoint[0], -self.max_rate_roll, self.max_rate_roll)
        rate_setpoint[1] = np.clip(rate_setpoint[1], -self.max_rate_pitch, self.max_rate_pitch)
        rate_setpoint[2] = np.clip(rate_setpoint[2], -self.max_rate_yaw, self.max_rate_yaw)

        return rate_setpoint


class VelocityController:
    """
    Velocity controller (PID).
    Converts velocity errors to attitude setpoints and thrust.

    Based on PX4 velocity controller.
    """
    def __init__(self):
        # Horizontal velocity PIDs (X, Y) - GENTLE
        self.vel_x_pid = SimplePID(
            kp=0.2, ki=0.01, kd=0.1,
            output_limits=(-2.0, 2.0),  # m/s² (reduced)
            integral_limits=(-0.5, 0.5)
        )
        self.vel_y_pid = SimplePID(
            kp=0.2, ki=0.01, kd=0.1,
            output_limits=(-2.0, 2.0),
            integral_limits=(-0.5, 0.5)
        )

        # Vertical velocity PID (Z) - More conservative
        self.vel_z_pid = SimplePID(
            kp=0.8, ki=0.05, kd=0.4,
            output_limits=(-3.0, 3.0),  # m/s²
            integral_limits=(-1.0, 1.0)
        )

        # Limits
        self.max_tilt = np.deg2rad(25)  # Maximum 25° tilt

        # Calculate hover thrust from physics
        # Mass = 1.5 kg, g = 9.81 m/s², 6 motors
        # Total thrust needed = 1.5 * 9.81 = 14.715 N
        # From hal_pybullet_enhanced.py: MAX_THRUST_PER_MOTOR = KF * (MAX_RPM * 2π/60)²
        # KF = 3.16e-5, MAX_RPM = 10000
        # MAX_THRUST_PER_MOTOR = 3.16e-5 * (10000 * 2π/60)² = 3.46 N
        # Total max thrust = 6 * 3.46 = 20.76 N
        # Hover thrust ratio = 14.715 / 20.76 = 0.708
        self.hover_thrust = 0.71  # Normalized thrust to hover

    def update(self, vel_setpoint: np.ndarray, current_vel: np.ndarray,
               current_yaw: float, dt: float) -> Tuple[np.ndarray, float]:
        """
        Args:
            vel_setpoint: [vx, vy, vz] desired velocity in m/s (world frame)
            current_vel: [vx, vy, vz] measured velocity in m/s (world frame)
            current_yaw: Current yaw angle in radians
            dt: Time step in seconds

        Returns:
            att_setpoint: [roll, pitch, yaw] attitude commands in radians
            thrust: Normalized thrust [0, 1]
        """
        # Compute desired accelerations (world frame)
        accel_x = self.vel_x_pid.update(vel_setpoint[0], current_vel[0], dt)
        accel_y = self.vel_y_pid.update(vel_setpoint[1], current_vel[1], dt)
        accel_z = self.vel_z_pid.update(vel_setpoint[2], current_vel[2], dt)

        # Vertical thrust (hover + accel compensation)
        thrust = self.hover_thrust + (accel_z / 9.81) * 0.15
        thrust = np.clip(thrust, 0.1, 0.95)

        # Rotate horizontal acceleration to body frame
        cos_yaw = np.cos(current_yaw)
        sin_yaw = np.sin(current_yaw)

        accel_x_body = accel_x * cos_yaw + accel_y * sin_yaw
        accel_y_body = -accel_x * sin_yaw + accel_y * cos_yaw

        # Desired attitude from acceleration (small angle approximation)
        # pitch (forward) = -accel_x / g
        # roll (right) = accel_y / g
        pitch_desired = np.arctan2(-accel_x_body, 9.81)
        roll_desired = np.arctan2(accel_y_body, 9.81)

        # Apply tilt limits
        pitch_desired = np.clip(pitch_desired, -self.max_tilt, self.max_tilt)
        roll_desired = np.clip(roll_desired, -self.max_tilt, self.max_tilt)

        att_setpoint = np.array([roll_desired, pitch_desired, current_yaw])

        return att_setpoint, thrust

    def reset(self):
        """Reset all PIDs"""
        self.vel_x_pid.reset()
        self.vel_y_pid.reset()
        self.vel_z_pid.reset()


class PositionController:
    """
    Position controller (P-only).
    Converts position errors to velocity setpoints.

    P control provides fast response without overshoot.
    Based on PX4 position controller.
    """
    def __init__(self):
        # P gains: position error (m) → velocity setpoint (m/s)
        self.kp_xy = 0.5  # Horizontal (gentler)
        self.kp_z = 0.8   # Vertical

        # Velocity limits (reduced for stability)
        self.max_vel_xy = 1.0  # m/s horizontal
        self.max_vel_z = 0.8   # m/s vertical

    def update(self, pos_setpoint: np.ndarray, current_pos: np.ndarray) -> np.ndarray:
        """
        Args:
            pos_setpoint: [x, y, z] desired position in meters
            current_pos: [x, y, z] measured position in meters

        Returns:
            vel_setpoint: [vx, vy, vz] velocity commands in m/s
        """
        # Position error
        pos_error = pos_setpoint - current_pos

        # P control: position error → velocity
        vel_setpoint = np.array([
            self.kp_xy * pos_error[0],
            self.kp_xy * pos_error[1],
            self.kp_z * pos_error[2]
        ])

        # Apply velocity limits
        vel_setpoint[0] = np.clip(vel_setpoint[0], -self.max_vel_xy, self.max_vel_xy)
        vel_setpoint[1] = np.clip(vel_setpoint[1], -self.max_vel_xy, self.max_vel_xy)
        vel_setpoint[2] = np.clip(vel_setpoint[2], -self.max_vel_z, self.max_vel_z)

        return vel_setpoint


class CascadedMulticopterController:
    """
    Complete cascaded control system for multicopter.

    Control loops (outermost to innermost):
    1. Position → Velocity (P)
    2. Velocity → Attitude + Thrust (PID)
    3. Attitude → Rate (P)
    4. Rate → Torque (PID)
    """

    def __init__(self, num_motors=6, mass_kg=1.5):
        self.num_motors = num_motors
        self.mass_kg = mass_kg

        # Initialize controllers
        self.pos_controller = PositionController()
        self.vel_controller = VelocityController()
        self.att_controller = AttitudeController()
        self.rate_controller = RateController()

        # Motor mixing matrix for hexacopter (X configuration)
        # [thrust, roll, pitch, yaw] → 6 motors
        self._setup_motor_mixing()

        # Timing
        self.last_time = time.time()

    def _setup_motor_mixing(self):
        """
        Setup motor mixing matrix for hexacopter.

        Motor layout (X configuration):
              0 (front)
           5     1
           4     2
              3 (back)

        Motor directions: 0,2,4 CCW(+yaw), 1,3,5 CW(-yaw)
        """
        # Angles for each motor (60° apart)
        angles = np.array([0, 60, 120, 180, 240, 300]) * np.pi / 180

        # Motor mixing matrix [6 motors x 4 inputs]
        # Columns: [thrust, roll, pitch, yaw]
        self.motor_mix = np.zeros((6, 4))

        for i in range(6):
            self.motor_mix[i, 0] = 1.0  # All motors contribute to thrust
            self.motor_mix[i, 1] = -np.sin(angles[i])  # Roll (Y-axis torque)
            self.motor_mix[i, 2] = np.cos(angles[i])   # Pitch (X-axis torque)
            self.motor_mix[i, 3] = (-1) ** (i + 1)     # Yaw (alternating)

    def compute_control(self, state: dict, setpoint: ControlSetpoint, dt: float = None) -> np.ndarray:
        """
        Main control loop - cascaded computation with quaternion attitude control.

        Args:
            state: Dictionary with keys:
                - 'position': [x, y, z] in meters
                - 'velocity': [vx, vy, vz] in m/s
                - 'euler': [roll, pitch, yaw] in radians (fallback)
                - 'orientation': [x, y, z, w] quaternion (PyBullet format - PREFERRED)
                - 'angular_velocity': [p, q, r] in rad/s
            setpoint: ControlSetpoint object
            dt: Time step (auto-computed if None)

        Returns:
            motor_pwm: Array of 6 motor PWM values [1000-2000]
        """
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            if dt <= 0 or dt > 0.1:  # Sanity check
                dt = 0.01

        # Extract state
        position = state['position']
        velocity = state['velocity']
        rates = state['angular_velocity']  # [p, q, r]

        # Get quaternion from state (preferred) or convert from Euler
        if 'orientation' in state:
            # PyBullet format [x, y, z, w] → our format [w, x, y, z]
            current_quat = pybullet_to_quaternion(state['orientation'])
            current_yaw = quaternion_to_euler(current_quat)[2]
        else:
            # Fallback to Euler angles
            attitude = state['euler']  # [roll, pitch, yaw]
            current_quat = euler_to_quaternion(attitude)
            current_yaw = attitude[2]

        # Cascaded control computation
        # Start from outermost specified setpoint

        if setpoint.position is not None:
            # Level 1: Position → Velocity
            vel_setpoint = self.pos_controller.update(setpoint.position, position)
        elif setpoint.velocity is not None:
            vel_setpoint = setpoint.velocity
        else:
            vel_setpoint = np.zeros(3)

        if setpoint.attitude_quaternion is None and setpoint.attitude is None:
            # Level 2: Velocity → Attitude + Thrust
            att_setpoint_euler, thrust = self.vel_controller.update(
                vel_setpoint, velocity, current_yaw, dt
            )
            # Override yaw if specified
            if setpoint.yaw is not None:
                att_setpoint_euler[2] = setpoint.yaw

            # Convert to quaternion for attitude controller
            att_setpoint_quat = euler_to_quaternion(att_setpoint_euler)
        else:
            # Use provided attitude setpoint
            if setpoint.attitude_quaternion is not None:
                att_setpoint_quat = setpoint.attitude_quaternion
            else:
                att_setpoint_quat = euler_to_quaternion(setpoint.attitude)

            thrust = setpoint.thrust if setpoint.thrust is not None else 0.71

        if setpoint.rate is None:
            # Level 3: Attitude → Rate (using quaternion-based controller)
            rate_setpoint = self.att_controller.update(
                att_setpoint=None,
                current_att_quat=current_quat,
                att_setpoint_quat=att_setpoint_quat
            )
        else:
            rate_setpoint = setpoint.rate

        # Level 4: Rate → Torque (innermost loop)
        torques = self.rate_controller.update(rate_setpoint, rates, dt)

        # Motor mixing: [thrust, torques] → motor commands
        motor_commands = self._mix_motors(thrust, torques)

        # Convert normalized commands to PWM
        motor_pwm = self._commands_to_pwm(motor_commands)

        return motor_pwm

    def _mix_motors(self, thrust: float, torques: np.ndarray) -> np.ndarray:
        """
        Mix thrust and torques to individual motor commands.

        Args:
            thrust: Normalized thrust [0, 1]
            torques: [tau_roll, tau_pitch, tau_yaw] normalized [-1, 1]

        Returns:
            motor_commands: 6 motor commands [0, 1]
        """
        # Create control vector [thrust, roll, pitch, yaw]
        control_vec = np.array([thrust, torques[0], torques[1], torques[2]])

        # Apply mixing matrix
        motor_commands = self.motor_mix @ control_vec

        # Clip to [0, 1]
        motor_commands = np.clip(motor_commands, 0.0, 1.0)

        return motor_commands

    def _commands_to_pwm(self, commands: np.ndarray) -> np.ndarray:
        """
        Convert normalized motor commands [0, 1] to PWM [1000, 2000].

        Args:
            commands: 6 motor commands [0, 1]

        Returns:
            pwm: 6 PWM values [1000, 2000]
        """
        pwm = 1000 + (commands * 1000)
        return pwm.astype(int)

    def reset(self):
        """Reset all controllers"""
        self.vel_controller.reset()
        self.rate_controller.reset()
        self.last_time = time.time()
