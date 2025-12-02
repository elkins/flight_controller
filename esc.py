"""
ESC (Electronic Speed Controller) Module

Controls brushless motor ESCs using PWM signals on PyBoard timers.
Standard ESC PWM range: 950-1950 microseconds

Motor Configuration:
- 6 motors (hexacopter configuration)
- Individual trim values for motor balancing
"""

from pyb import Pin, Timer


# ============================================================================
# CONFIGURATION
# ============================================================================

# ESC pin assignments (PyBoard-specific)
# Note: Y9/Y10 reserved for I2C, using Y1/Y2 (TIM8 CH1/CH2) instead
ESC_PINS = ['X1', 'X2', 'X3', 'X6', 'Y1', 'Y2']

# Timer assignments for each pin
ESC_TIMERS = [5, 5, 5, 2, 8, 8]

# Timer channel assignments
ESC_CHANNELS = [1, 2, 3, 1, 1, 2]

# Individual motor trim values (microseconds offset)
# Adjust these to balance motor thrust
ESC_TRIM = [0, 0, 0, 0, 0, 0]


# ============================================================================
# ESC CONTROLLER CLASS
# ============================================================================

class ESC:
    """
    ESC controller using PWM signals.
    
    PWM Configuration:
    - Frequency: 50 Hz (standard servo/ESC frequency)
    - Pulse width: 950-1950 microseconds
    - Timer prescaler: 83 (for 1MHz timer clock)
    - Timer period: 19999 (20ms period)
    """
    
    # PWM pulse width limits (microseconds)
    FREQ_MIN = 950
    FREQ_MAX = 1950
    
    # Timer configuration (PyBoard @ 84 MHz)
    TIMER_PRESCALER = 83  # 84 MHz / (83 + 1) = 1 MHz
    TIMER_PERIOD = 19999  # 1 MHz / 20000 = 50 Hz
    
    def __init__(self, index):
        """
        Initialize ESC controller.
        
        Args:
            index: Motor index (0-5 for hexacopter)
        """
        if not 0 <= index < len(ESC_PINS):
            raise ValueError(f"Invalid motor index: {index}. Must be 0-{len(ESC_PINS)-1}")
        
        self.index = index
        self.trim = ESC_TRIM[index]
        
        # Initialize timer and PWM channel
        self.timer = Timer(
            ESC_TIMERS[index],
            prescaler=self.TIMER_PRESCALER,
            period=self.TIMER_PERIOD
        )
        
        self.channel = self.timer.channel(
            ESC_CHANNELS[index],
            Timer.PWM,
            pin=Pin(ESC_PINS[index])
        )
        
        # Start with minimum throttle
        self.move(self.FREQ_MIN)
    
    def move(self, freq):
        """
        Set motor speed.
        
        Args:
            freq: Throttle value in microseconds (will be clamped to valid range)
        """
        # Apply trim and clamp to valid range
        freq = freq + self.trim
        freq = max(self.FREQ_MIN, min(self.FREQ_MAX, freq))
        
        # Set PWM pulse width
        self.channel.pulse_width(int(freq))
    
    def stop(self):
        """Stop the motor (minimum throttle)."""
        self.move(self.FREQ_MIN)
    
    def __del__(self):
        """Cleanup timer resources."""
        try:
            self.timer.deinit()
        except:
            pass
    
    def __repr__(self):
        """String representation."""
        return f"ESC(motor={self.index}, pin={ESC_PINS[self.index]}, trim={self.trim})"
