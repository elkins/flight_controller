"""
Flight Safety System

Implements critical safety features for drone flight including:
- Battery voltage monitoring and failsafe
- RC signal loss detection
- Geofencing (altitude and distance limits)
- Motor failure detection
- Emergency landing procedures
"""

import time
import logging
from typing import Optional, Tuple, Dict, Any
from enum import Enum

logger = logging.getLogger(__name__)


class SafetyStatus(Enum):
    """Safety status levels"""
    NORMAL = "normal"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class FailsafeAction(Enum):
    """Actions to take when failsafe is triggered"""
    NONE = "none"
    HOVER = "hover"
    LAND = "land"
    RETURN_TO_HOME = "return_to_home"
    DISARM = "disarm"


class FlightSafety:
    """
    Flight safety monitor and failsafe handler.
    
    Monitors critical flight parameters and triggers appropriate
    failsafe actions when safety limits are exceeded.
    """
    
    def __init__(self):
        # Battery settings
        self.battery_cells = 3  # 3S LiPo
        self.battery_voltage_min = 3.3  # Volts per cell (critical)
        self.battery_voltage_warn = 3.5  # Volts per cell (warning)
        self.battery_voltage_nominal = 3.7  # Volts per cell (nominal)
        
        # RC signal settings
        self.rc_timeout_ms = 1000  # Lost signal after 1 second
        self.rc_last_valid_time = time.time()
        
        # Geofence settings
        self.max_altitude = 100.0  # meters (AGL)
        self.max_distance = 200.0  # meters from home
        self.home_position: Optional[Tuple[float, float, float]] = None
        
        # Motor failure detection
        self.motor_command_min_threshold = 1100  # Below this with throttle high = failure
        self.motor_failure_detected = [False] * 6
        
        # State tracking
        self.status = SafetyStatus.NORMAL
        self.active_failsafes: Dict[str, FailsafeAction] = {}
        self.warnings: list = []
        
        # Statistics
        self.battery_checks = 0
        self.rc_checks = 0
        self.geofence_checks = 0
        
    def set_home_position(self, x: float, y: float, z: float):
        """
        Set home position for return-to-home and geofencing.
        
        Args:
            x, y, z: Position in meters (NED frame)
        """
        self.home_position = (x, y, z)
        logger.info(f"Home position set: {x:.2f}, {y:.2f}, {z:.2f}")
    
    def check_battery(self, voltage: float) -> Tuple[SafetyStatus, Optional[FailsafeAction]]:
        """
        Check battery voltage and return safety status.
        
        Args:
            voltage: Battery voltage in volts
            
        Returns:
            Tuple of (safety_status, failsafe_action)
        """
        self.battery_checks += 1
        
        voltage_per_cell = voltage / self.battery_cells
        
        if voltage_per_cell <= self.battery_voltage_min:
            # Critical - land immediately
            self.active_failsafes['battery'] = FailsafeAction.LAND
            logger.critical(f"Battery critical: {voltage:.2f}V ({voltage_per_cell:.2f}V/cell) - LANDING")
            return SafetyStatus.CRITICAL, FailsafeAction.LAND
        
        elif voltage_per_cell <= self.battery_voltage_warn:
            # Warning - prepare to land
            if 'battery' not in self.active_failsafes:
                logger.warning(f"Battery low: {voltage:.2f}V ({voltage_per_cell:.2f}V/cell)")
            return SafetyStatus.WARNING, FailsafeAction.NONE
        
        else:
            # Normal
            if 'battery' in self.active_failsafes:
                del self.active_failsafes['battery']
            return SafetyStatus.NORMAL, FailsafeAction.NONE
    
    def check_rc_signal(self, rc_values: list, current_time: Optional[float] = None) -> Tuple[SafetyStatus, Optional[FailsafeAction]]:
        """
        Check RC signal validity and timeout.
        
        Args:
            rc_values: List of RC channel values (microseconds)
            current_time: Current time (defaults to time.time())
            
        Returns:
            Tuple of (safety_status, failsafe_action)
        """
        self.rc_checks += 1
        
        if current_time is None:
            current_time = time.time()
        
        # Check if RC values are valid (within normal range)
        valid_signal = all(1000 <= val <= 2000 for val in rc_values if val is not None)
        
        if valid_signal:
            self.rc_last_valid_time = current_time
            if 'rc_loss' in self.active_failsafes:
                del self.active_failsafes['rc_loss']
                logger.info("RC signal restored")
            return SafetyStatus.NORMAL, FailsafeAction.NONE
        
        # Calculate time since last valid signal
        time_since_valid = (current_time - self.rc_last_valid_time) * 1000  # milliseconds
        
        if time_since_valid > self.rc_timeout_ms:
            # RC signal lost - hover and prepare to land
            self.active_failsafes['rc_loss'] = FailsafeAction.HOVER
            logger.critical(f"RC signal lost for {time_since_valid:.0f}ms - HOVERING")
            return SafetyStatus.CRITICAL, FailsafeAction.HOVER
        
        return SafetyStatus.WARNING, FailsafeAction.NONE
    
    def check_geofence(self, position: Tuple[float, float, float]) -> Tuple[SafetyStatus, Optional[FailsafeAction]]:
        """
        Check if vehicle is within geofence boundaries.
        
        Args:
            position: Current position (x, y, z) in meters (NED frame)
            
        Returns:
            Tuple of (safety_status, failsafe_action)
        """
        self.geofence_checks += 1
        
        x, y, z = position
        altitude = -z  # Convert NED to altitude (down is negative)
        
        # Check altitude limit
        if altitude > self.max_altitude:
            self.active_failsafes['altitude'] = FailsafeAction.LAND
            logger.critical(f"Altitude limit exceeded: {altitude:.1f}m > {self.max_altitude:.1f}m - LANDING")
            return SafetyStatus.CRITICAL, FailsafeAction.LAND
        
        # Check distance from home (if home is set)
        if self.home_position:
            home_x, home_y, _ = self.home_position
            distance = ((x - home_x)**2 + (y - home_y)**2)**0.5
            
            if distance > self.max_distance:
                self.active_failsafes['distance'] = FailsafeAction.RETURN_TO_HOME
                logger.critical(f"Distance limit exceeded: {distance:.1f}m > {self.max_distance:.1f}m - RTH")
                return SafetyStatus.CRITICAL, FailsafeAction.RETURN_TO_HOME
        
        # Clear geofence failsafes if within limits
        if 'altitude' in self.active_failsafes:
            del self.active_failsafes['altitude']
        if 'distance' in self.active_failsafes:
            del self.active_failsafes['distance']
        
        return SafetyStatus.NORMAL, FailsafeAction.NONE
    
    def check_motor_failure(self, motor_commands: list, throttle: int) -> Tuple[SafetyStatus, Optional[FailsafeAction]]:
        """
        Detect motor failures by comparing commanded vs actual behavior.
        
        Args:
            motor_commands: List of commanded motor values (microseconds)
            throttle: Current throttle input (microseconds)
            
        Returns:
            Tuple of (safety_status, failsafe_action)
        """
        # Only check if throttle is significant
        if throttle < 1200:
            return SafetyStatus.NORMAL, FailsafeAction.NONE
        
        failures_detected = []
        for i, cmd in enumerate(motor_commands):
            # If commanded value is reasonable but motor isn't responding
            if cmd > self.motor_command_min_threshold and cmd < 1100:
                if not self.motor_failure_detected[i]:
                    self.motor_failure_detected[i] = True
                    failures_detected.append(i)
                    logger.critical(f"Motor {i} failure detected - LANDING")
        
        if failures_detected:
            self.active_failsafes['motor_failure'] = FailsafeAction.LAND
            return SafetyStatus.EMERGENCY, FailsafeAction.LAND
        
        return SafetyStatus.NORMAL, FailsafeAction.NONE
    
    def get_overall_status(self) -> SafetyStatus:
        """
        Get overall safety status based on all checks.
        
        Returns:
            Worst safety status from all checks
        """
        if self.active_failsafes:
            # Check if any emergency failsafes are active
            emergency_actions = [FailsafeAction.LAND, FailsafeAction.DISARM]
            if any(action in emergency_actions for action in self.active_failsafes.values()):
                return SafetyStatus.EMERGENCY
            
            # Check if any critical failsafes are active
            if self.active_failsafes:
                return SafetyStatus.CRITICAL
        
        return self.status
    
    def get_primary_failsafe_action(self) -> FailsafeAction:
        """
        Get the most critical failsafe action currently required.
        
        Priority order: DISARM > LAND > RETURN_TO_HOME > HOVER > NONE
        
        Returns:
            The highest priority failsafe action
        """
        if not self.active_failsafes:
            return FailsafeAction.NONE
        
        priority = [
            FailsafeAction.DISARM,
            FailsafeAction.LAND,
            FailsafeAction.RETURN_TO_HOME,
            FailsafeAction.HOVER,
            FailsafeAction.NONE
        ]
        
        for action in priority:
            if action in self.active_failsafes.values():
                return action
        
        return FailsafeAction.NONE
    
    def reset(self):
        """Reset all failsafe states and warnings."""
        self.active_failsafes.clear()
        self.warnings.clear()
        self.motor_failure_detected = [False] * 6
        self.status = SafetyStatus.NORMAL
        logger.info("Safety system reset")
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get safety check statistics.
        
        Returns:
            Dictionary with check counts and current status
        """
        return {
            'battery_checks': self.battery_checks,
            'rc_checks': self.rc_checks,
            'geofence_checks': self.geofence_checks,
            'status': self.status.value,
            'active_failsafes': {k: v.value for k, v in self.active_failsafes.items()},
            'motor_failures': sum(self.motor_failure_detected)
        }


def apply_failsafe_action(action: FailsafeAction, 
                         current_throttle: int,
                         motor_commands: list) -> Tuple[int, list]:
    """
    Apply failsafe action to throttle and motor commands.
    
    Args:
        action: Failsafe action to apply
        current_throttle: Current throttle command
        motor_commands: Current motor commands
        
    Returns:
        Tuple of (modified_throttle, modified_motor_commands)
    """
    if action == FailsafeAction.DISARM:
        # Cut all motors immediately
        return 1000, [1000] * 6
    
    elif action == FailsafeAction.LAND:
        # Gradual descent
        landing_throttle = max(1000, current_throttle - 5)  # Reduce by 5µs per loop
        return landing_throttle, [landing_throttle] * 6
    
    elif action == FailsafeAction.HOVER:
        # Maintain current altitude (keep last known good throttle)
        # In real implementation, this would use altitude hold
        return current_throttle, motor_commands
    
    elif action == FailsafeAction.RETURN_TO_HOME:
        # In real implementation, this would navigate to home
        # For now, just hover
        return current_throttle, motor_commands
    
    else:  # NONE
        return current_throttle, motor_commands
