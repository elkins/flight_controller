"""
Rate Controller Tuning - Find Stable Gains Empirically

This tests ONLY the rate controller in isolation to find stable gains.
We'll manually command small rate setpoints and measure the response.

This is the correct way to tune cascaded control:
1. Tune rate controller FIRST (innermost loop)
2. Then tune attitude controller
3. Then tune velocity controller
4. Finally tune position controller
"""

import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hal.hal_pybullet_enhanced import EnhancedPyBulletPlatform


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


class RateControllerTest:
    """Minimal rate controller for tuning"""

    def __init__(self, kp=0.005, ki=0.001, kd=0.0001):
        """Start with VERY conservative gains"""
        self.roll_rate_pid = SimplePID(
            kp=kp, ki=ki, kd=kd,
            output_limits=(-0.1, 0.1),
            integral_limits=(-0.02, 0.02)
        )
        self.pitch_rate_pid = SimplePID(
            kp=kp, ki=ki, kd=kd,
            output_limits=(-0.1, 0.1),
            integral_limits=(-0.02, 0.02)
        )
        self.yaw_rate_pid = SimplePID(
            kp=kp*0.8, ki=ki*0.5, kd=0.0,  # Yaw typically needs less gain
            output_limits=(-0.08, 0.08),
            integral_limits=(-0.01, 0.01)
        )

    def update(self, rate_setpoint, current_rates, dt):
        """
        Args:
            rate_setpoint: [roll_rate, pitch_rate, yaw_rate] in rad/s
            current_rates: [p, q, r] measured rates in rad/s
            dt: timestep in seconds

        Returns:
            torques: [roll_torque, pitch_torque, yaw_torque] normalized
        """
        roll_torque = self.roll_rate_pid.update(rate_setpoint[0], current_rates[0], dt)
        pitch_torque = self.pitch_rate_pid.update(rate_setpoint[1], current_rates[1], dt)
        yaw_torque = self.yaw_rate_pid.update(rate_setpoint[2], current_rates[2], dt)

        return np.array([roll_torque, pitch_torque, yaw_torque])


def mix_motors_hexacopter(thrust, torques):
    """Mix thrust + torques into 6 motor commands"""
    angles = np.array([i * 60 for i in range(6)]) * np.pi / 180

    motor_mix = np.zeros((6, 4))
    for i in range(6):
        motor_mix[i, 0] = 1.0  # Thrust
        motor_mix[i, 1] = -np.sin(angles[i])  # Roll
        motor_mix[i, 2] = np.cos(angles[i])   # Pitch
        motor_mix[i, 3] = (-1) ** (i + 1)     # Yaw

    commands = np.array([thrust, torques[0], torques[1], torques[2]])
    motor_thrust = motor_mix @ commands
    motor_thrust = np.clip(motor_thrust, 0.0, 1.0)
    motor_pwm = 1000 + motor_thrust * 1000

    return motor_pwm.astype(int)


def test_rate_step_response(kp=0.005, ki=0.001, kd=0.0001):
    """
    Test rate controller with step input.
    Good rate controller should:
    - Reach setpoint without oscillation
    - No overshoot
    - Settle within 0.5-1.0 seconds
    """
    print("\n" + "="*70)
    print(f"RATE CONTROLLER TUNING TEST")
    print("="*70)
    print(f"\nTesting gains: kp={kp}, ki={ki}, kd={kd}")
    print("\nTest sequence:")
    print("  0-3s:   Startup (disarmed)")
    print("  3-5s:   Hover at 0.5m (establish baseline)")
    print("  5-8s:   Command +10°/s roll rate (step input)")
    print("  8-11s:  Command 0°/s (return to level)")
    print("  11-14s: Command +10°/s pitch rate")
    print("  14-17s: Command 0°/s (return to level)")
    print("  17-20s: Hover and observe stability\n")

    platform = EnhancedPyBulletPlatform(gui=True, start_position=[0, 0, 0.05])
    rate_controller = RateControllerTest(kp=kp, ki=ki, kd=kd)

    # Simple altitude hold (fixed throttle near hover)
    hover_throttle = 0.265

    start_time = time.time()
    last_time = start_time
    loop_count = 0

    print("="*70)
    print("FLIGHT LOG")
    print("="*70)

    try:
        while time.time() - start_time < 20:
            current_time = time.time()
            elapsed = current_time - start_time
            dt = current_time - last_time
            last_time = current_time

            # Get state
            state = platform.get_state()
            pos = state['position']
            euler = state['euler']
            rates = state['angular_velocity']

            # Determine phase and rate setpoint
            if elapsed < 3:
                phase = "STARTUP"
                armed = False
                rate_setpoint = np.zeros(3)
                throttle = 0.0
            elif elapsed < 5:
                phase = "HOVER"
                armed = True
                rate_setpoint = np.zeros(3)
                throttle = hover_throttle
            elif elapsed < 8:
                phase = "ROLL STEP +10°/s"
                armed = True
                rate_setpoint = np.array([np.deg2rad(10), 0, 0])
                throttle = hover_throttle
            elif elapsed < 11:
                phase = "ROLL RETURN 0°/s"
                armed = True
                rate_setpoint = np.zeros(3)
                throttle = hover_throttle
            elif elapsed < 14:
                phase = "PITCH STEP +10°/s"
                armed = True
                rate_setpoint = np.array([0, np.deg2rad(10), 0])
                throttle = hover_throttle
            elif elapsed < 17:
                phase = "PITCH RETURN 0°/s"
                armed = True
                rate_setpoint = np.zeros(3)
                throttle = hover_throttle
            else:
                phase = "OBSERVE"
                armed = True
                rate_setpoint = np.zeros(3)
                throttle = hover_throttle

            # Compute control
            if armed:
                torques = rate_controller.update(rate_setpoint, rates, dt)
            else:
                torques = np.zeros(3)

            # Mix motors
            motor_pwm = mix_motors_hexacopter(throttle, torques)

            # Apply commands
            for i, pwm in enumerate(motor_pwm):
                platform.motors[i].pulse_width_us(int(pwm))

            platform.delay_ms(10)
            loop_count += 1

            # Status every 0.5s
            if loop_count % 50 == 0:
                print(f"t={elapsed:5.1f}s [{phase:20s}] | "
                      f"Alt={pos[2]:5.2f}m | "
                      f"Att=[{np.degrees(euler[0]):6.1f}°,{np.degrees(euler[1]):6.1f}°,{np.degrees(euler[2]):6.1f}°] | "
                      f"Rates=[{np.degrees(rates[0]):6.1f}°/s,{np.degrees(rates[1]):6.1f}°/s,{np.degrees(rates[2]):6.1f}°/s] | "
                      f"Setpoint=[{np.degrees(rate_setpoint[0]):6.1f}°/s,{np.degrees(rate_setpoint[1]):6.1f}°/s]")

    except KeyboardInterrupt:
        print("\n\nTest interrupted")

    platform.disconnect()

    print("\n" + "="*70)
    print("TUNING GUIDELINES")
    print("="*70)
    print("\nObserve the response:")
    print("  ✓ GOOD: Smooth approach to setpoint, no oscillation")
    print("  ✗ BAD:  Oscillation → reduce kp")
    print("  ✗ BAD:  Overshoot → reduce kp or increase kd")
    print("  ✗ BAD:  Slow/sluggish → increase kp")
    print("  ✗ BAD:  Steady-state error → increase ki")
    print("\nNext steps:")
    print("  1. If unstable/oscillating: reduce kp by 50% and re-test")
    print("  2. If stable but slow: increase kp by 20% and re-test")
    print("  3. Once stable, tune attitude controller next")
    print()


if __name__ == "__main__":
    # Start with VERY conservative gains
    test_rate_step_response(kp=0.005, ki=0.001, kd=0.0001)
