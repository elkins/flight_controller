"""
Manual Flight Simulator - Human in the Loop

Demonstrates why manual control works when autonomous control fails:
- Only ONE control loop (rate control)
- Human provides attitude/position control
- Much more forgiving of tuning errors
"""

import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hal.hal_pybullet_enhanced import EnhancedPyBulletPlatform


class SimpleRateController:
    """
    Minimal rate controller - this is ALL you need for manual flight.
    Human pilot provides the rate setpoints.
    """
    def __init__(self):
        # Very conservative gains
        self.kp_roll = 0.02
        self.kp_pitch = 0.02
        self.kp_yaw = 0.015

    def update(self, rate_setpoint, current_rate):
        """
        Args:
            rate_setpoint: [roll_rate, pitch_rate, yaw_rate] in rad/s (from pilot stick)
            current_rate: [p, q, r] measured rates

        Returns:
            torques: [roll_torque, pitch_torque, yaw_torque]
        """
        roll_error = rate_setpoint[0] - current_rate[0]
        pitch_error = rate_setpoint[1] - current_rate[1]
        yaw_error = rate_setpoint[2] - current_rate[2]

        torques = np.array([
            self.kp_roll * roll_error,
            self.kp_pitch * pitch_error,
            self.kp_yaw * yaw_error
        ])

        return np.clip(torques, -0.2, 0.2)


def mix_motors_hexacopter(thrust, torques):
    """
    Mix thrust + torques into 6 motor commands.
    Same as cascaded controller, but simplified.
    """
    # Hexacopter X configuration
    angles = np.array([i * 60 for i in range(6)]) * np.pi / 180

    motor_mix = np.zeros((6, 4))
    for i in range(6):
        motor_mix[i, 0] = 1.0  # Thrust
        motor_mix[i, 1] = -np.sin(angles[i])  # Roll
        motor_mix[i, 2] = np.cos(angles[i])   # Pitch
        motor_mix[i, 3] = (-1) ** (i + 1)     # Yaw

    # Combine: [thrust, roll_torque, pitch_torque, yaw_torque]
    commands = np.array([thrust, torques[0], torques[1], torques[2]])
    motor_thrust = motor_mix @ commands

    # Convert to PWM
    motor_thrust = np.clip(motor_thrust, 0.0, 1.0)
    motor_pwm = 1000 + motor_thrust * 1000

    return motor_pwm.astype(int)


def simulate_human_pilot(t, phase, target_alt):
    """
    Simulate what a human pilot would do with the sticks.

    Returns:
        (throttle, rate_setpoint)
    """
    # Phase 1: Takeoff (t < 8s)
    if phase == "TAKEOFF":
        # Human ramps throttle up gradually
        throttle = 0.35 if t < 5 else 0.32
        # Keep attitude level (no stick input)
        rate_setpoint = np.array([0.0, 0.0, 0.0])

    # Phase 2: Hover (8s < t < 20s)
    elif phase == "HOVER":
        # Human adjusts throttle to maintain altitude
        throttle = 0.265  # Close to actual hover
        rate_setpoint = np.array([0.0, 0.0, 0.0])

    # Phase 3: Circle (20s < t < 35s)
    elif phase == "CIRCLE":
        pattern_time = t - 20
        throttle = 0.27  # Slightly more for maneuvering

        # Human banks and yaws smoothly
        # Gentle roll for turn, yaw to coordinate
        roll_rate = 0.15 * np.sin(pattern_time * 0.2)  # Oscillate gently
        yaw_rate = 0.1 * np.sin(pattern_time * 0.2)
        rate_setpoint = np.array([roll_rate, 0.0, yaw_rate])

    # Phase 4: Landing
    elif phase == "LANDING":
        # Human reduces throttle gradually
        throttle = 0.22  # Below hover
        rate_setpoint = np.array([0.0, 0.0, 0.0])

    else:  # DISARMED
        throttle = 0.0
        rate_setpoint = np.array([0.0, 0.0, 0.0])

    return throttle, rate_setpoint


def main():
    """
    Simulate manual flight with human in the loop.
    """
    print("\n" + "="*70)
    print("MANUAL FLIGHT SIMULATION")
    print("="*70)
    print("\nThis simulates what happens when a HUMAN flies with just rate control.")
    print("Note: Only ONE control loop (rate) instead of four cascaded loops!\n")
    print("Control Architecture:")
    print("  Human Pilot → Rate Controller → Motors")
    print("  (Human closes attitude/velocity/position loops in their brain)\n")

    platform = EnhancedPyBulletPlatform(gui=True, start_position=[0, 0, 0.05])
    rate_controller = SimpleRateController()

    print("Flight Plan:")
    print("  0-3s:     Startup (disarmed)")
    print("  3-8s:     Takeoff (human ramps throttle)")
    print("  8-20s:    Hover (human maintains altitude)")
    print("  20-35s:   Gentle maneuvers (human banks/yaws)")
    print("  35-40s:   Landing\n")

    print("="*70)
    print("FLIGHT LOG")
    print("="*70)

    start_time = time.time()
    loop_count = 0

    try:
        while time.time() - start_time < 40:
            elapsed = time.time() - start_time

            # Get state
            state = platform.get_state()
            pos = state['position']
            vel = state['velocity']
            euler = state['euler']
            rates = state['angular_velocity']

            # Determine phase
            if elapsed < 3:
                phase = "DISARMED"
                target_alt = 0.05
            elif elapsed < 8:
                phase = "TAKEOFF"
                target_alt = 2.0
            elif elapsed < 20:
                phase = "HOVER"
                target_alt = 2.0
            elif elapsed < 35:
                phase = "CIRCLE"
                target_alt = 2.0
            else:
                phase = "LANDING"
                target_alt = 0.0

            # Simulate human pilot input
            throttle, rate_setpoint = simulate_human_pilot(elapsed, phase, target_alt)

            # Rate controller (only loop needed!)
            if phase != "DISARMED":
                torques = rate_controller.update(rate_setpoint, rates)
            else:
                torques = np.zeros(3)

            # Mix motors
            motor_pwm = mix_motors_hexacopter(throttle, torques)

            # Apply commands
            for i, pwm in enumerate(motor_pwm):
                platform.motors[i].pulse_width_us(int(pwm))

            platform.delay_ms(10)
            loop_count += 1

            # Status every second
            if loop_count % 100 == 0:
                print(f"t={elapsed:5.1f}s [{phase:10s}] | "
                      f"Alt={pos[2]:5.2f}m | "
                      f"Vel=[{vel[0]:+4.1f},{vel[1]:+4.1f},{vel[2]:+4.1f}]m/s | "
                      f"Att=[{np.degrees(euler[0]):5.1f}°,{np.degrees(euler[1]):5.1f}°,{np.degrees(euler[2]):5.1f}°] | "
                      f"Throttle={throttle:.3f}")

    except KeyboardInterrupt:
        print("\n\nFlight interrupted")

    platform.disconnect()

    print("\n" + "="*70)
    print("KEY INSIGHTS")
    print("="*70)
    print("\n1. SIMPLICITY:")
    print("   Manual flight uses ONE control loop (rate)")
    print("   Autonomous needs FOUR cascaded loops (position→velocity→attitude→rate)")
    print()
    print("2. HUMAN ADVANTAGES:")
    print("   - Visual feedback (see drift, compensate)")
    print("   - Predictive control (anticipate wind, momentum)")
    print("   - Adaptive learning (adjust for conditions)")
    print("   - Fault tolerance (one bad gain doesn't crash everything)")
    print()
    print("3. WHY CASCADED CONTROL IS HARD:")
    print("   - Each loop amplifies errors from inner loops")
    print("   - All gains must be tuned TOGETHER as a system")
    print("   - No visual feedback to guide tuning")
    print("   - Small rate errors → large position errors over time")
    print()
    print("4. THE SOLUTION:")
    print("   Option A: Systematic gain tuning (rate→attitude→velocity→position)")
    print("   Option B: Copy proven gains from gym-pybullet-drones")
    print("   Option C: Add human-like learning (Model Predictive Control, RL)")
    print()

if __name__ == "__main__":
    main()
