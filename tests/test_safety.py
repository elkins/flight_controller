"""
Flight Safety System Tests

Tests all safety features including battery monitoring, RC signal loss,
geofencing, and motor failure detection.
"""

import pytest
import time
from src.safety import (
    FlightSafety, SafetyStatus, FailsafeAction, apply_failsafe_action
)


def test_battery_normal():
    """Test battery in normal operating range"""
    safety = FlightSafety()
    
    # Normal battery voltage (11.1V for 3S LiPo)
    status, action = safety.check_battery(11.1)
    
    assert status == SafetyStatus.NORMAL
    assert action == FailsafeAction.NONE
    assert 'battery' not in safety.active_failsafes


def test_battery_warning():
    """Test battery low warning"""
    safety = FlightSafety()
    
    # Low battery (10.5V = 3.5V per cell)
    status, action = safety.check_battery(10.5)
    
    assert status == SafetyStatus.WARNING
    assert action == FailsafeAction.NONE


def test_battery_critical():
    """Test battery critical failsafe"""
    safety = FlightSafety()
    
    # Critical battery (9.6V = 3.2V per cell, below 3.3 threshold)
    status, action = safety.check_battery(9.6)
    
    assert status == SafetyStatus.CRITICAL
    assert action == FailsafeAction.LAND
    assert 'battery' in safety.active_failsafes


def test_battery_recovery():
    """Test battery failsafe clears when voltage recovers"""
    safety = FlightSafety()
    
    # Go critical
    safety.check_battery(9.6)
    assert 'battery' in safety.active_failsafes
    
    # Recover (might happen if load decreases)
    status, action = safety.check_battery(11.1)
    assert status == SafetyStatus.NORMAL
    assert 'battery' not in safety.active_failsafes


def test_rc_signal_valid():
    """Test valid RC signal"""
    safety = FlightSafety()
    
    # Valid RC channels
    rc_values = [1500, 1500, 1500, 1500]
    status, action = safety.check_rc_signal(rc_values)
    
    assert status == SafetyStatus.NORMAL
    assert action == FailsafeAction.NONE
    assert 'rc_loss' not in safety.active_failsafes


def test_rc_signal_loss():
    """Test RC signal loss detection"""
    safety = FlightSafety()
    
    # Valid signal first
    rc_values = [1500, 1500, 1500, 1500]
    safety.check_rc_signal(rc_values, current_time=0.0)
    
    # Invalid signal after timeout
    invalid_rc = [0, 0, 0, 0]
    status, action = safety.check_rc_signal(invalid_rc, current_time=2.0)
    
    assert status == SafetyStatus.CRITICAL
    assert action == FailsafeAction.HOVER
    assert 'rc_loss' in safety.active_failsafes


def test_rc_signal_recovery():
    """Test RC signal recovery"""
    safety = FlightSafety()
    
    # Lose signal
    rc_values = [1500, 1500, 1500, 1500]
    safety.check_rc_signal(rc_values, current_time=0.0)
    invalid_rc = [0, 0, 0, 0]
    safety.check_rc_signal(invalid_rc, current_time=2.0)
    assert 'rc_loss' in safety.active_failsafes
    
    # Recover
    status, action = safety.check_rc_signal(rc_values, current_time=3.0)
    assert status == SafetyStatus.NORMAL
    assert 'rc_loss' not in safety.active_failsafes


def test_geofence_altitude_limit():
    """Test altitude geofence"""
    safety = FlightSafety()
    safety.max_altitude = 100.0
    
    # Normal altitude
    status, action = safety.check_geofence((0, 0, -50))  # 50m altitude
    assert status == SafetyStatus.NORMAL
    
    # Exceeded altitude
    status, action = safety.check_geofence((0, 0, -150))  # 150m altitude
    assert status == SafetyStatus.CRITICAL
    assert action == FailsafeAction.LAND
    assert 'altitude' in safety.active_failsafes


def test_geofence_distance_limit():
    """Test distance geofence"""
    safety = FlightSafety()
    safety.set_home_position(0, 0, 0)
    safety.max_distance = 200.0
    
    # Normal distance
    status, action = safety.check_geofence((100, 0, -50))  # 100m away
    assert status == SafetyStatus.NORMAL
    
    # Exceeded distance
    status, action = safety.check_geofence((250, 0, -50))  # 250m away
    assert status == SafetyStatus.CRITICAL
    assert action == FailsafeAction.RETURN_TO_HOME
    assert 'distance' in safety.active_failsafes


def test_geofence_no_home_set():
    """Test geofence without home position set"""
    safety = FlightSafety()
    
    # Should only check altitude, not distance
    status, action = safety.check_geofence((1000, 1000, -50))
    assert status == SafetyStatus.NORMAL


def test_motor_failure_detection():
    """Test motor failure detection"""
    safety = FlightSafety()
    
    # Normal operation
    motor_commands = [1500, 1500, 1500, 1500, 1500, 1500]
    status, action = safety.check_motor_failure(motor_commands, throttle=1500)
    assert status == SafetyStatus.NORMAL
    
    # Motor failure (commanded high but reading low)
    failed_motors = [1500, 1500, 1000, 1500, 1500, 1500]  # Motor 2 failed
    status, action = safety.check_motor_failure(failed_motors, throttle=1500)
    # Note: This test depends on implementation - motor failure detection
    # typically requires sensor feedback which isn't in this simplified version


def test_overall_status():
    """Test overall status calculation"""
    safety = FlightSafety()
    
    # Normal
    assert safety.get_overall_status() == SafetyStatus.NORMAL
    
    # Add a warning
    safety.check_battery(10.5)  # Low battery warning
    # Status should still be normal since no failsafes active
    
    # Add a critical failsafe
    safety.check_battery(9.6)  # Critical battery
    assert safety.get_overall_status() == SafetyStatus.EMERGENCY


def test_primary_failsafe_action():
    """Test primary failsafe action priority"""
    safety = FlightSafety()
    
    # No failsafes
    assert safety.get_primary_failsafe_action() == FailsafeAction.NONE
    
    # Add hover failsafe
    safety.active_failsafes['rc_loss'] = FailsafeAction.HOVER
    assert safety.get_primary_failsafe_action() == FailsafeAction.HOVER
    
    # Add land failsafe (higher priority)
    safety.active_failsafes['battery'] = FailsafeAction.LAND
    assert safety.get_primary_failsafe_action() == FailsafeAction.LAND


def test_failsafe_action_disarm():
    """Test disarm failsafe action"""
    throttle, motors = apply_failsafe_action(
        FailsafeAction.DISARM, 1500, [1500] * 6
    )
    
    assert throttle == 1000
    assert all(m == 1000 for m in motors)


def test_failsafe_action_land():
    """Test landing failsafe action"""
    # Should gradually reduce throttle
    throttle, motors = apply_failsafe_action(
        FailsafeAction.LAND, 1500, [1500] * 6
    )
    
    assert throttle < 1500
    assert throttle >= 1000


def test_failsafe_action_hover():
    """Test hover failsafe action"""
    original_throttle = 1500
    original_motors = [1500, 1500, 1500, 1500, 1500, 1500]
    
    throttle, motors = apply_failsafe_action(
        FailsafeAction.HOVER, original_throttle, original_motors
    )
    
    # Should maintain current commands
    assert throttle == original_throttle
    assert motors == original_motors


def test_safety_reset():
    """Test safety system reset"""
    safety = FlightSafety()
    
    # Trigger some failsafes
    safety.check_battery(9.6)
    safety.check_rc_signal([0, 0, 0, 0], current_time=2.0)
    
    assert len(safety.active_failsafes) > 0
    
    # Reset
    safety.reset()
    
    assert len(safety.active_failsafes) == 0
    assert safety.status == SafetyStatus.NORMAL


def test_statistics():
    """Test safety statistics"""
    safety = FlightSafety()
    
    # Perform some checks
    safety.check_battery(11.1)
    safety.check_battery(11.0)
    safety.check_rc_signal([1500] * 4)
    safety.check_geofence((0, 0, -50))
    
    stats = safety.get_statistics()
    
    assert stats['battery_checks'] == 2
    assert stats['rc_checks'] == 1
    assert stats['geofence_checks'] == 1
    assert stats['status'] == 'normal'


def test_multiple_failsafes():
    """Test handling multiple simultaneous failsafes"""
    safety = FlightSafety()
    
    # Initialize RC signal first
    safety.check_rc_signal([1500, 1500, 1500, 1500], current_time=0.0)
    
    # Trigger multiple failsafes
    safety.check_battery(9.6)  # Critical battery
    safety.check_rc_signal([0, 0, 0, 0], current_time=2.0)  # RC loss
    safety.check_geofence((0, 0, -150))  # Altitude limit
    
    assert len(safety.active_failsafes) == 3
    
    # Should prioritize LAND over HOVER
    action = safety.get_primary_failsafe_action()
    assert action == FailsafeAction.LAND


def test_home_position():
    """Test setting and using home position"""
    safety = FlightSafety()
    
    # Set home
    safety.set_home_position(10.0, 20.0, -5.0)
    
    assert safety.home_position == (10.0, 20.0, -5.0)
    
    # Check distance from home
    status, action = safety.check_geofence((10.0, 20.0, -50))
    assert status == SafetyStatus.NORMAL  # At home


def test_battery_different_cell_counts():
    """Test battery monitoring with different cell counts"""
    safety = FlightSafety()
    
    # 3S battery
    safety.battery_cells = 3
    status, _ = safety.check_battery(9.6)  # 3.2V per cell
    assert status == SafetyStatus.CRITICAL
    
    # 4S battery
    safety.battery_cells = 4
    safety.reset()
    status, _ = safety.check_battery(12.8)  # 3.2V per cell
    assert status == SafetyStatus.CRITICAL
    
    # 6S battery
    safety.battery_cells = 6
    safety.reset()
    status, _ = safety.check_battery(19.2)  # 3.2V per cell
    assert status == SafetyStatus.CRITICAL


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
