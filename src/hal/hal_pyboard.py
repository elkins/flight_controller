"""
PyBoard Hardware Abstraction Layer Implementation

Implements HAL interfaces for PyBoard/STM32 platform.
"""

import logging
from src.hal.hal import (
    HALPlatform, HALTimer, HALPWMChannel, HALInputCapture, HALI2C
)
from pyb import Timer, Pin, I2C, millis, delay
from typing import Callable

# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class PyBoardPWMChannel(HALPWMChannel):
    """PyBoard PWM channel implementation"""
    
    def __init__(self, timer_channel, prescaler: int, period: int):
        self.channel = timer_channel
        self.prescaler = prescaler
        self.period = period
        self._pulse_width = 1000
    
    def set_pulse_width(self, width_us: int):
        """Set PWM pulse width in microseconds
        
        Raises:
            ValueError: If pulse width is outside safe range
        """
        try:
            self.validate_pulse_width(width_us)
            self._pulse_width = width_us
            self.channel.pulse_width(int(width_us))
            logger.debug(f"PWM set to {width_us}us")
        except ValueError as e:
            logger.error(f"Invalid PWM width: {e}")
            raise
        except Exception as e:
            logger.error(f"Failed to set PWM: {e}")
            raise
    
    def get_pulse_width(self) -> int:
        """Get current PWM pulse width in microseconds"""
        return self._pulse_width


class PyBoardInputCapture(HALInputCapture):
    """PyBoard input capture implementation"""
    
    def __init__(self, timer_channel, pin):
        self.channel = timer_channel
        self.pin = pin
        self.start_time = 0
        self.pulse_width = 1500
        self.last_valid_width = 1500
        self._callback = None
        
        # Set up interrupt callback
        self.channel.callback(self._internal_callback)
    
    def _internal_callback(self, timer):
        """Internal interrupt handler"""
        if self.pin.value():
            # Rising edge - start of pulse
            self.start_time = self.channel.capture()
        else:
            # Falling edge - end of pulse
            self.pulse_width = (self.channel.capture() - self.start_time) & 0x0FFFFFFF
        
        if self._callback:
            self._callback(self.pulse_width)
    
    def get_pulse_width(self) -> int:
        """Get captured pulse width in microseconds"""
        # Validate and filter
        if 950 < self.pulse_width < 1950:
            self.last_valid_width = self.pulse_width
        else:
            logger.warning(
                f"RC pulse {self.pulse_width}us outside normal range, "
                f"using last valid: {self.last_valid_width}us"
            )
        return self.last_valid_width
    
    def set_callback(self, callback: Callable):
        """Set user callback for pulse updates"""
        self._callback = callback


class PyBoardTimer(HALTimer):
    """PyBoard timer implementation"""
    
    def __init__(self, timer_id: int):
        self.timer_id = timer_id
        self.timer = None
        self.channels = {}
    
    def create_pwm_channel(self, pin: str, frequency: int) -> HALPWMChannel:
        """Create a PWM output channel
        
        Raises:
            ValueError: If frequency is invalid
        """
        if frequency <= 0:
            raise ValueError(f"Invalid frequency: {frequency}")
        
        # Calculate timer parameters for given frequency
        # Assuming 84 MHz clock
        prescaler = 83  # 84MHz / 84 = 1 MHz
        period = int(1_000_000 / frequency) - 1  # Period in ticks
        
        if self.timer is None:
            self.timer = Timer(self.timer_id, prescaler=prescaler, period=period)
            logger.info(f"Created Timer {self.timer_id} @ {frequency}Hz")
        
        # Determine channel number from pin
        channel_num = self._get_channel_for_pin(pin)
        
        timer_channel = self.timer.channel(
            channel_num,
            Timer.PWM,
            pin=Pin(pin)
        )
        
        pwm_channel = PyBoardPWMChannel(timer_channel, prescaler, period)
        self.channels[pin] = pwm_channel
        logger.debug(f"Created PWM on pin {pin}")
        return pwm_channel
    
    def create_input_capture(self, pin: str, callback: Callable) -> HALInputCapture:
        """Create an input capture channel"""
        if self.timer is None:
            self.timer = Timer(self.timer_id, prescaler=83, period=0x0FFFFFFF)
            logger.info(f"Created Timer {self.timer_id} for input capture")
        
        pin_obj = Pin(pin)
        channel_num = self._get_channel_for_pin(pin)
        
        timer_channel = self.timer.channel(
            channel_num,
            Timer.IC,
            pin=pin_obj,
            polarity=Timer.BOTH
        )
        
        ic_channel = PyBoardInputCapture(timer_channel, pin_obj)
        ic_channel.set_callback(callback)
        self.channels[pin] = ic_channel
        logger.debug(f"Created input capture on pin {pin}")
        return ic_channel
    
    def _get_channel_for_pin(self, pin: str) -> int:
        """Get timer channel number for a pin"""
        # This is a simplified mapping - would need full pin to channel mapping
        # For now, extract from known configurations
        pin_to_channel = {
            'X1': 1, 'X2': 2, 'X3': 3, 'X6': 1,
            'Y1': 1, 'Y2': 2, 'Y3': 3, 'Y4': 4, 'Y7': 1, 'Y8': 2
        }
        return pin_to_channel.get(pin, 1)
    
    def deinit(self):
        """Deinitialize the timer"""
        if self.timer:
            self.timer.deinit()


class PyBoardI2C(HALI2C):
    """PyBoard I2C implementation"""
    
    def __init__(self, bus_id: int):
        self.bus = I2C(bus_id, I2C.MASTER)
    
    def write_byte(self, device_addr: int, register: int, value: int):
        """Write a byte to I2C device register
        
        Raises:
            IOError: If I2C communication fails
        """
        try:
            self.bus.mem_write(value, device_addr, register)
            logger.debug(f"I2C write: addr=0x{device_addr:02X} reg=0x{register:02X} val=0x{value:02X}")
        except Exception as e:
            logger.error(f"I2C write failed: addr=0x{device_addr:02X} - {e}")
            raise IOError(f"I2C write error: {e}")
    
    def read_byte(self, device_addr: int, register: int) -> int:
        """Read a byte from I2C device register
        
        Raises:
            IOError: If I2C communication fails
        """
        try:
            buf = bytearray(1)
            self.bus.mem_read(buf, device_addr, register)
            logger.debug(f"I2C read: addr=0x{device_addr:02X} reg=0x{register:02X} val=0x{buf[0]:02X}")
            return buf[0]
        except Exception as e:
            logger.error(f"I2C read failed: addr=0x{device_addr:02X} - {e}")
            raise IOError(f"I2C read error: {e}")
    
    def read_bytes(self, device_addr: int, register: int, length: int) -> bytes:
        """Read multiple bytes from I2C device
        
        Raises:
            ValueError: If length is invalid
            IOError: If I2C communication fails
        """
        if length <= 0:
            raise ValueError(f"Invalid read length: {length}")
        try:
            buf = bytearray(length)
            self.bus.mem_read(buf, device_addr, register)
            logger.debug(f"I2C read: addr=0x{device_addr:02X} reg=0x{register:02X} len={length}")
            return bytes(buf)
        except Exception as e:
            logger.error(f"I2C read failed: addr=0x{device_addr:02X} - {e}")
            raise IOError(f"I2C read error: {e}")
    
    def write_bytes(self, device_addr: int, register: int, data: bytes):
        """Write multiple bytes to I2C device
        
        Raises:
            ValueError: If data is invalid
            IOError: If I2C communication fails
        """
        if not data:
            raise ValueError("Empty data buffer")
        try:
            self.bus.mem_write(data, device_addr, register)
            logger.debug(f"I2C write: addr=0x{device_addr:02X} reg=0x{register:02X} len={len(data)}")
        except Exception as e:
            logger.error(f"I2C write failed: addr=0x{device_addr:02X} - {e}")
            raise IOError(f"I2C write error: {e}")


class PyBoardPlatform(HALPlatform):
    """PyBoard platform implementation"""
    
    def __init__(self):
        self.timers = {}
        self.i2c_buses = {}
    
    def get_timer(self, timer_id: int) -> HALTimer:
        """Get a timer instance
        
        Raises:
            ValueError: If timer_id is invalid
        """
        if not (1 <= timer_id <= 14):
            raise ValueError(f"Invalid timer ID {timer_id} (must be 1-14 for PyBoard)")
        if timer_id not in self.timers:
            logger.info(f"Creating Timer {timer_id}")
            self.timers[timer_id] = PyBoardTimer(timer_id)
        return self.timers[timer_id]
    
    def get_i2c(self, bus_id: int) -> HALI2C:
        """Get an I2C bus instance
        
        Raises:
            ValueError: If bus_id is invalid
        """
        if bus_id not in (1, 2):
            raise ValueError(f"Invalid I2C bus ID {bus_id} (must be 1 or 2 for PyBoard)")
        if bus_id not in self.i2c_buses:
            logger.info(f"Creating I2C bus {bus_id}")
            self.i2c_buses[bus_id] = PyBoardI2C(bus_id)
        return self.i2c_buses[bus_id]
    
    def millis(self) -> int:
        """Get milliseconds since boot"""
        return millis()
    
    def delay_ms(self, ms: int):
        """Delay for specified milliseconds"""
        delay(ms)
    
    def delay_us(self, us: int):
        """Delay for specified microseconds"""
        delay(us / 1000)
    
    @property
    def platform_name(self) -> str:
        """Get platform name"""
        return "PyBoard"
