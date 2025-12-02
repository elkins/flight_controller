"""
RC Receiver Module

Reads PWM signals from RC receiver using timer input capture.
Supports standard RC PWM signals (950-1950 microseconds).

Configuration:
- 4 channels: Throttle, Roll, Pitch, Yaw
- Timer input capture mode for accurate pulse width measurement
"""

from pyb import Timer, Pin


# ============================================================================
# CONFIGURATION
# ============================================================================

# Timer assignments for RC receiver channels
RC_TIMERS = [12, 4]

# Timer index for each channel
RC_PINS_TIMERS = [0, 0, 1, 1]

# Pin assignments for each channel
RC_PINS = ['Y8', 'Y7', 'Y4', 'Y3']

# Timer channel assignments
RC_PINS_CHANNELS = [2, 1, 4, 3]

# Initialize timers for input capture
timers = [Timer(k, prescaler=83, period=0x0FFFFFFF) for k in RC_TIMERS]


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def map_range(x, in_min, in_max, out_min, out_max):
    """
    Map a value from one range to another (linear interpolation).
    
    Args:
        x: Input value
        in_min: Input range minimum
        in_max: Input range maximum
        out_min: Output range minimum
        out_max: Output range maximum
        
    Returns:
        Mapped value in output range
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def wrap_180(x):
    """
    Wrap angle to [-180, 180] range.
    
    Args:
        x: Angle in degrees
        
    Returns:
        Angle wrapped to [-180, 180]
    """
    if x < -180:
        return x + 360
    elif x > 180:
        return x - 360
    else:
        return x


# ============================================================================
# RC RECEIVER CLASS
# ============================================================================

class RC:
    """
    RC receiver channel using timer input capture.
    
    Measures PWM pulse width from RC receiver using rising/falling edge detection.
    Provides filtering to reject invalid pulses.
    """
    
    # Valid PWM pulse width range (microseconds)
    PULSE_MIN = 950
    PULSE_MAX = 1950
    
    def __init__(self, index):
        """
        Initialize RC receiver channel.
        
        Args:
            index: Channel index (0=throttle, 1=roll, 2=pitch, 3=yaw)
        """
        if not 0 <= index < len(RC_PINS):
            raise ValueError(f"Invalid channel index: {index}. Must be 0-{len(RC_PINS)-1}")
        
        self.index = index
        self.pin = Pin(RC_PINS[index])
        
        # State variables
        self.start = 0
        self.width = 0
        self.last_width = 1500  # Default center value
        
        # Setup timer input capture
        timer = timers[RC_PINS_TIMERS[index]]
        self.channel = timer.channel(
            RC_PINS_CHANNELS[index],
            Timer.IC,
            pin=self.pin,
            polarity=Timer.BOTH
        )
        
        # Register interrupt callback
        self.channel.callback(self._callback)
    
    def _callback(self, timer):
        """
        Interrupt callback for edge detection.
        
        Called on both rising and falling edges to measure pulse width.
        
        Args:
            timer: Timer object (unused but required by callback signature)
        """
        if self.pin.value():
            # Rising edge - start of pulse
            self.start = self.channel.capture()
        else:
            # Falling edge - end of pulse
            # Calculate width with 32-bit wraparound handling
            self.width = (self.channel.capture() - self.start) & 0x0FFFFFFF
    
    def get_width(self):
        """
        Get current pulse width with validation.
        
        Returns:
            Pulse width in microseconds. Invalid pulses are rejected
            and the last valid width is returned instead.
        """
        w = self.width
        
        # Validate pulse width
        if self.PULSE_MIN < w < self.PULSE_MAX:
            self.last_width = w
        
        return self.last_width
    
    def get_normalized(self, center=1500):
        """
        Get normalized channel value [-1, 1].
        
        Args:
            center: Center point for normalization (default 1500)
            
        Returns:
            Normalized value: -1 (min), 0 (center), +1 (max)
        """
        width = self.get_width()
        if width < center:
            return (width - center) / (center - self.PULSE_MIN)
        else:
            return (width - center) / (self.PULSE_MAX - center)
    
    def __del__(self):
        """Cleanup resources."""
        try:
            self.channel.callback(None)
        except:
            pass
    
    def __repr__(self):
        """String representation."""
        return f"RC(channel={self.index}, pin={RC_PINS[self.index]}, width={self.last_width}us)"
