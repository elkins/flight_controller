"""
Hardware Abstraction Layer (HAL)

Provides a unified interface for different hardware platforms:
- PyBoard (current implementation)
- Arduino (future)
- Raspberry Pi (future)
- Other microcontrollers (future)

This allows the flight controller logic to remain platform-independent.
"""

import logging
from abc import ABC, abstractmethod
from typing import Callable, Optional

# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class TimerMode:
    """Timer operation modes"""
    PWM = 'PWM'
    INPUT_CAPTURE = 'INPUT_CAPTURE'


class HALTimer(ABC):
    """Abstract timer interface"""
    
    @abstractmethod
    def create_pwm_channel(self, pin: str, frequency: int) -> 'HALPWMChannel':
        """Create a PWM output channel"""
        pass
    
    @abstractmethod
    def create_input_capture(self, pin: str, callback: Callable) -> 'HALInputCapture':
        """Create an input capture channel for measuring pulse width"""
        pass
    
    @abstractmethod
    def deinit(self):
        """Deinitialize the timer"""
        pass


class HALPWMChannel(ABC):
    """Abstract PWM output channel"""
    
    # Safety limits for pulse width
    MIN_PULSE_WIDTH = 500
    MAX_PULSE_WIDTH = 2500
    
    @abstractmethod
    def set_pulse_width(self, width_us: int):
        """Set PWM pulse width in microseconds"""
        pass
    
    @abstractmethod
    def get_pulse_width(self) -> int:
        """Get current PWM pulse width in microseconds"""
        pass
    
    def validate_pulse_width(self, width_us: int):
        """Validate pulse width is within safe limits
        
        Raises:
            ValueError: If pulse width is outside safe range
        """
        if not (self.MIN_PULSE_WIDTH <= width_us <= self.MAX_PULSE_WIDTH):
            raise ValueError(
                f"Pulse width {width_us}us outside safe range "
                f"[{self.MIN_PULSE_WIDTH}, {self.MAX_PULSE_WIDTH}]us"
            )


class HALInputCapture(ABC):
    """Abstract input capture interface"""
    
    @abstractmethod
    def get_pulse_width(self) -> int:
        """Get captured pulse width in microseconds"""
        pass
    
    @abstractmethod
    def set_callback(self, callback: Callable):
        """Set interrupt callback for pulse capture"""
        pass


class HALI2C(ABC):
    """Abstract I2C interface"""
    
    @abstractmethod
    def write_byte(self, device_addr: int, register: int, value: int):
        """Write a byte to I2C device register"""
        pass
    
    @abstractmethod
    def read_byte(self, device_addr: int, register: int) -> int:
        """Read a byte from I2C device register"""
        pass
    
    @abstractmethod
    def read_bytes(self, device_addr: int, register: int, length: int) -> bytes:
        """Read multiple bytes from I2C device"""
        pass
    
    @abstractmethod
    def write_bytes(self, device_addr: int, register: int, data: bytes):
        """Write multiple bytes to I2C device"""
        pass


class HALPlatform(ABC):
    """Abstract hardware platform interface"""
    
    @abstractmethod
    def get_timer(self, timer_id: int) -> HALTimer:
        """Get a timer instance"""
        pass
    
    @abstractmethod
    def get_i2c(self, bus_id: int) -> HALI2C:
        """Get an I2C bus instance"""
        pass
    
    @abstractmethod
    def millis(self) -> int:
        """Get milliseconds since boot"""
        pass
    
    @abstractmethod
    def delay_ms(self, ms: int):
        """Delay for specified milliseconds"""
        pass
    
    @abstractmethod
    def delay_us(self, us: int):
        """Delay for specified microseconds"""
        pass
    
    @property
    @abstractmethod
    def platform_name(self) -> str:
        """Get platform name"""
        pass


# ============================================================================
# PLATFORM FACTORY
# ============================================================================

_current_platform: Optional[HALPlatform] = None


def set_platform(platform: HALPlatform):
    """Set the current hardware platform
    
    Raises:
        TypeError: If platform is not a HALPlatform instance
    """
    global _current_platform
    if not isinstance(platform, HALPlatform):
        raise TypeError(f"Expected HALPlatform, got {type(platform)}")
    logger.info(f"Setting platform to: {platform.platform_name}")
    _current_platform = platform


def get_platform() -> HALPlatform:
    """Get the current hardware platform
    
    Raises:
        RuntimeError: If no platform is detected or set
    """
    if _current_platform is None:
        logger.info("Auto-detecting hardware platform...")
        # Auto-detect platform
        try:
            import pyb
            from hal_pyboard import PyBoardPlatform
            platform = PyBoardPlatform()
            set_platform(platform)
            logger.info("Detected: PyBoard")
        except ImportError:
            try:
                import RPi.GPIO
                from hal_raspberrypi import RaspberryPiPlatform
                platform = RaspberryPiPlatform()
                set_platform(platform)
                logger.info("Detected: Raspberry Pi")
            except ImportError:
                logger.error("No supported hardware platform detected")
                raise RuntimeError(
                    "No supported hardware platform detected. "
                    "Please manually set platform using set_platform()"
                )
    
    return _current_platform


# ============================================================================
# CONVENIENCE FUNCTIONS
# ============================================================================

def create_esc(motor_index: int, pin: str, frequency: int = 50) -> 'ESCController':
    """
    Create an ESC controller for the current platform.
    
    Args:
        motor_index: Motor number (0-5 for hexacopter)
        pin: Pin identifier (platform-specific)
        frequency: PWM frequency in Hz (default 50 for standard ESCs)
    
    Returns:
        ESCController instance
    """
    from esc_hal import ESCController
    return ESCController(motor_index, pin, frequency)


def create_rc_channel(channel_index: int, pin: str) -> 'RCChannelReader':
    """
    Create an RC channel reader for the current platform.
    
    Args:
        channel_index: RC channel number (0-3 for TAER)
        pin: Pin identifier (platform-specific)
    
    Returns:
        RCChannelReader instance
    """
    from rc_hal import RCChannelReader
    return RCChannelReader(channel_index, pin)


def create_imu(i2c_bus: int = 1, address: int = 0x68) -> 'IMUReader':
    """
    Create an IMU reader for the current platform.
    
    Args:
        i2c_bus: I2C bus number
        address: I2C device address
    
    Returns:
        IMUReader instance
    """
    from imu_hal import IMUReader
    return IMUReader(i2c_bus, address)
