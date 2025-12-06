"""
Debug rate controller - detailed logging of PID loop
"""

import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hal.hal_pybullet_enhanced import EnhancedPyBulletPlatform


def test_debug():
    """Test with detailed PID logging"""
    print("\n" + "="*70)
    print("RATE CONTROLLER DEBUG - DETAILED PID LOGGING")
    print("="*70)

    platform = EnhancedPyBulletPlatform(gui=True, start_position=[0, 0, 0.05])

    # Very simple proportional-only controller to isolate sign error
    kp = 0.001  # Very small
    hover_pwm_norm = 0.265

    start_time = time.time()
    last_time = start_time

    print("\nTest: Command +10°/s roll rate, observe response")
    print("If actual rate goes NEGATIVE → control has wrong sign!\n")
    print("="*70)

    try:
        while time.time() - start_time < 8:
            current_time = time.time()
            elapsed = current_time - start_time
            dt = current_time - last_time
            last_time = current_time

            # Get state
            state = platform.get_state()
            pos = state['position']
            euler = state['euler']
            rates = state['angular_velocity']  # [roll_rate, pitch_rate, yaw_rate] in rad/s

            # Phase
            if elapsed < 2:
                phase = "DISARMED"
                armed = False
            elif elapsed < 5:
                phase = "HOVER"
                armed = True
                rate_setpoint_roll = 0.0
            else:
                phase = "ROLL STEP"
                armed = True
                rate_setpoint_roll = np.deg2rad(10)  # +10°/s

            if not armed:
                # Disarmed
                motor_thrust = np.zeros(6)
            else:
                # Simple P controller on roll rate ONLY
                error = rate_setpoint_roll - rates[0]  # setpoint - measurement
                roll_torque = kp * error  # Positive error → positive torque

                # TEST: INVERT roll torque sign to test if mixing is backwards
                roll_torque = -roll_torque  # INVERTED!

                # Apply to motors via mixing
                # Hexacopter X: motors at 0°, 60°, 120°, 180°, 240°, 300°
                angles = np.array([i * 60 for i in range(6)]) * np.pi / 180

                motor_thrust = np.zeros(6)
                for i in range(6):
                    # Roll torque: -sin(angle)
                    motor_thrust[i] = hover_pwm_norm + roll_torque * (-np.sin(angles[i]))

                motor_thrust = np.clip(motor_thrust, 0.0, 1.0)

            # Convert to PWM
            motor_pwm = (1000 + motor_thrust * 1000).astype(int)

            # Apply
            for i, pwm in enumerate(motor_pwm):
                platform.motors[i].pulse_width_us(pwm)

            platform.delay_ms(10)

            # Detailed logging every 0.2s
            if int(elapsed * 50) % 10 == 0:
                if armed and phase == "ROLL STEP":
                    error = rate_setpoint_roll - rates[0]
                    roll_torque = kp * error
                    print(f"t={elapsed:5.2f}s | "
                          f"Setpoint={np.degrees(rate_setpoint_roll):+6.1f}°/s | "
                          f"Actual={np.degrees(rates[0]):+6.1f}°/s | "
                          f"Error={np.degrees(error):+6.1f}°/s | "
                          f"Torque={roll_torque:+.6f} | "
                          f"Motors=[{motor_pwm[0]}, {motor_pwm[1]}, ..., {motor_pwm[5]}]")
                else:
                    print(f"t={elapsed:5.2f}s | {phase:12s}")

    except KeyboardInterrupt:
        print("\n\nTest interrupted")

    platform.disconnect()

    print("\n" + "="*70)
    print("ANALYSIS")
    print("="*70)
    print("\nIf actual rate has OPPOSITE sign to setpoint:")
    print("  → PID sign error or motor mixing sign error")
    print("\nIf actual rate has SAME sign but doesn't reach setpoint:")
    print("  → Gain too low, increase kp")
    print()


if __name__ == "__main__":
    test_debug()
