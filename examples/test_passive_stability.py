"""
Test passive stability - NO control at all.

Apply equal thrust to all motors and see if PyBullet physics is stable.
If this diverges, the problem is with physics or motor mixing, not PID gains.
"""

import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hal.hal_pybullet_enhanced import EnhancedPyBulletPlatform


def test_passive():
    """Test with NO control - just equal thrust on all motors"""
    print("\n" + "="*70)
    print("PASSIVE STABILITY TEST (NO CONTROL)")
    print("="*70)
    print("\nApplying EQUAL thrust to all motors (no stabilization)")
    print("If drone diverges, problem is physics/mixing, not PID gains.\n")

    platform = EnhancedPyBulletPlatform(gui=True, start_position=[0, 0, 0.05])

    hover_pwm = 1265  # Equal thrust on all motors

    start_time = time.time()
    loop_count = 0

    print("="*70)
    print("FLIGHT LOG")
    print("="*70)

    try:
        while time.time() - start_time < 15:
            elapsed = time.time() - start_time

            # Get state
            state = platform.get_state()
            pos = state['position']
            euler = state['euler']
            rates = state['angular_velocity']

            # Phase
            if elapsed < 2:
                phase = "DISARMED"
                pwm = 1000
            else:
                phase = "EQUAL THRUST"
                pwm = hover_pwm

            # Apply EQUAL PWM to all motors (no control)
            for i in range(6):
                platform.motors[i].pulse_width_us(pwm)

            platform.delay_ms(10)
            loop_count += 1

            # Status every 0.5s
            if loop_count % 50 == 0:
                print(f"t={elapsed:5.1f}s [{phase:15s}] | "
                      f"Alt={pos[2]:5.2f}m | "
                      f"Att=[{np.degrees(euler[0]):6.1f}°,{np.degrees(euler[1]):6.1f}°,{np.degrees(euler[2]):6.1f}°] | "
                      f"Rates=[{np.degrees(rates[0]):6.1f}°/s,{np.degrees(rates[1]):6.1f}°/s,{np.degrees(rates[2]):6.1f}°/s]")

    except KeyboardInterrupt:
        print("\n\nTest interrupted")

    platform.disconnect()

    print("\n" + "="*70)
    print("ANALYSIS")
    print("="*70)
    print("\nIf drone stayed approximately level:")
    print("  ✓ Physics is stable - PID tuning is the issue")
    print("\nIf drone diverged/flipped:")
    print("  ✗ Physics or motor mixing has issues")
    print("  Check: Motor arrangement, thrust coefficients, COM/inertia")
    print()


if __name__ == "__main__":
    test_passive()
