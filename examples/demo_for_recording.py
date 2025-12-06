"""
Simple hover demo for manual screen recording.

Run this and use QuickTime Player or screen recording software to capture the window:
  1. Open QuickTime Player
  2. File → New Screen Recording
  3. Click record, then click the PyBullet window
  4. Run this script
  5. Stop recording when done

This avoids PyBullet's broken video recording.
"""

import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hal.hal_pybullet_enhanced import EnhancedPyBulletPlatform
from src.control.cascaded_controller import CascadedMulticopterController, ControlSetpoint


def main():
    print("\n" + "="*70)
    print("CASCADED CONTROLLER HOVER DEMO")
    print("FOR MANUAL SCREEN RECORDING")
    print("="*70)
    print("\n🎥 Please start your screen recording NOW")
    print("   (QuickTime Player → File → New Screen Recording)")
    print("\nWaiting 3 seconds before starting...")

    for i in range(3, 0, -1):
        print(f"   {i}...")
        time.sleep(1)

    print("\n▶️  Starting flight demo!\n")

    # Initialize
    platform = EnhancedPyBulletPlatform(gui=True, start_position=[0, 0, 0.05])
    controller = CascadedMulticopterController()

    print("Flight sequence:")
    print("  0-3s:    Startup (disarmed)")
    print("  3-8s:    Takeoff to 2.0m")
    print("  8-20s:   Hover at 2.0m")
    print("  20-23s:  Land")
    print("\n" + "="*70)

    start_time = time.time()
    loop_count = 0

    try:
        while time.time() - start_time < 23:
            elapsed = time.time() - start_time

            # Get state
            state = platform.get_state()
            pos = state['position']
            vel = state['velocity']
            euler = state['euler']

            # Determine phase and setpoint
            if elapsed < 3:
                # Disarmed
                setpoint = ControlSetpoint(position=np.array([0, 0, 0.05]))
                motor_pwm = np.array([1000] * 6)
                phase = "STARTUP"
            elif elapsed < 8:
                # Takeoff
                setpoint = ControlSetpoint(position=np.array([0, 0, 2.0]))
                motor_pwm = controller.compute_control(state, setpoint, dt=0.01)
                phase = "TAKEOFF"
            elif elapsed < 20:
                # Hover
                setpoint = ControlSetpoint(position=np.array([0, 0, 2.0]))
                motor_pwm = controller.compute_control(state, setpoint, dt=0.01)
                phase = "HOVER"
            else:
                # Land
                setpoint = ControlSetpoint(position=np.array([0, 0, 0.5]))
                motor_pwm = controller.compute_control(state, setpoint, dt=0.01)
                phase = "LANDING"

            # Apply motor commands
            for i, pwm in enumerate(motor_pwm):
                platform.motors[i].pulse_width_us(int(pwm))

            # Step simulation
            platform.delay_ms(10)
            loop_count += 1

            # Status every second
            if loop_count % 100 == 0:
                print(f"t={elapsed:5.1f}s [{phase:10s}] | "
                      f"Alt={pos[2]:5.2f}m | "
                      f"Vel=[{vel[0]:+4.1f},{vel[1]:+4.1f},{vel[2]:+4.1f}]m/s | "
                      f"Att=[{np.degrees(euler[0]):5.1f}°,{np.degrees(euler[1]):5.1f}°,{np.degrees(euler[2]):5.1f}°]")

    except KeyboardInterrupt:
        print("\n\nDemo interrupted")

    platform.disconnect()

    print("\n" + "="*70)
    print("✓ Demo complete!")
    print("\n🛑 You can stop your screen recording now")
    print("\nThe video should show:")
    print("  ✓ Drone on ground (0-3s)")
    print("  ✓ Smooth takeoff (3-8s)")
    print("  ✓ Stable hover at 2m (8-20s)")
    print("  ✓ Controlled descent (20-23s)")
    print("="*70)
    print()


if __name__ == "__main__":
    main()
