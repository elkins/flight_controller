"""
PID Controller Module

Implements a PID (Proportional-Integral-Derivative) controller with:
- Integral windup protection
- Derivative filtering using low-pass filter
- Timeout detection for safe initialization
"""

from pyb import millis
from math import pi, isnan


class PID:
    """
    PID Controller with integral windup protection and derivative filtering.
    
    Based on ArduPilot PID implementation with enhancements for:
    - Smooth derivative calculation using RC filter
    - Automatic timeout detection and reset
    - Integral clamping to prevent windup
    """
    
    # Low-pass filter time constant for derivative (20 Hz cutoff)
    DERIVATIVE_RC = 1.0 / (2.0 * pi * 20.0)
    
    # Maximum time gap before reset (ms)
    TIMEOUT_MS = 1000
    
    def __init__(self, p=0.0, i=0.0, d=0.0, imax=0.0):
        """
        Initialize PID controller.
        
        Args:
            p: Proportional gain
            i: Integral gain
            d: Derivative gain
            imax: Maximum integral value (prevents windup)
        """
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        
        # State variables
        self._integrator = 0.0
        self._last_error = 0.0
        self._last_derivative = float('nan')
        self._last_t = 0
    
    def get_pid(self, error, scaler):
        """
        Calculate PID output.
        
        Args:
            error: Error value (setpoint - measurement)
            scaler: Output scaler/gain
            
        Returns:
            PID output value
        """
        # Get current time and calculate delta
        tnow = millis()
        dt = tnow - self._last_t
        
        # Reset on first run or timeout
        if self._last_t == 0 or dt > self.TIMEOUT_MS:
            dt = 0
            self.reset_I()
        
        self._last_t = tnow
        delta_time = float(dt) / 1000.0
        
        # Proportional term
        output = error * self._kp
        
        # Derivative term (with low-pass filtering)
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0.0
                self._last_derivative = 0.0
            else:
                # Calculate instantaneous derivative
                derivative = (error - self._last_error) / delta_time
                
                # Apply low-pass filter to smooth derivative
                alpha = delta_time / (self.DERIVATIVE_RC + delta_time)
                derivative = self._last_derivative + alpha * (derivative - self._last_derivative)
            
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        
        # Apply scaler to P and D terms
        output *= scaler
        
        # Integral term (with anti-windup clamping)
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            
            # Clamp integrator to prevent windup
            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax
            
            output += self._integrator
        
        return output
    
    def reset_I(self):
        """Reset integral and derivative terms."""
        self._integrator = 0.0
        self._last_derivative = float('nan')
    
    def set_gains(self, p=None, i=None, d=None, imax=None):
        """
        Update PID gains dynamically.
        
        Args:
            p: Proportional gain (optional)
            i: Integral gain (optional)
            d: Derivative gain (optional)
            imax: Maximum integral value (optional)
        """
        if p is not None:
            self._kp = float(p)
        if i is not None:
            self._ki = float(i)
        if d is not None:
            self._kd = float(d)
        if imax is not None:
            self._imax = abs(imax)
    
    def get_integrator(self):
        """Get current integrator value."""
        return self._integrator
    
    def __repr__(self):
        """String representation of PID controller."""
        return f"PID(P={self._kp}, I={self._ki}, D={self._kd}, Imax={self._imax})"
