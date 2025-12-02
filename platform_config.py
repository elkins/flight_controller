"""
Platform Configuration

Defines pin mappings and hardware configurations for different platforms.
Makes it easy to port to new hardware by just updating this file.
"""

import logging

# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# ============================================================================
# PYBOARD CONFIGURATION
# ============================================================================

PYBOARD_CONFIG = {
    'platform': 'pyboard',
    
    # ESC/Motor pins (PWM output)
    'esc_pins': {
        0: {'pin': 'X1', 'timer': 5, 'channel': 1},
        1: {'pin': 'X2', 'timer': 5, 'channel': 2},
        2: {'pin': 'X3', 'timer': 5, 'channel': 3},
        3: {'pin': 'X6', 'timer': 2, 'channel': 1},
        4: {'pin': 'Y9', 'timer': 2, 'channel': 3},
        5: {'pin': 'Y10', 'timer': 2, 'channel': 4},
    },
    
    # RC receiver pins (Input capture)
    'rc_pins': {
        0: {'pin': 'Y8', 'timer': 12, 'channel': 2},  # Throttle
        1: {'pin': 'Y7', 'timer': 12, 'channel': 1},  # Roll
        2: {'pin': 'Y4', 'timer': 4, 'channel': 4},   # Pitch
        3: {'pin': 'Y3', 'timer': 4, 'channel': 3},   # Yaw
    },
    
    # I2C configuration
    'i2c': {
        'bus': 1,
        'mpu6050_address': 0x68,
    },
    
    # Timer groups (some pins share timers)
    'timer_groups': {
        2: [3, 4, 5],  # ESCs 3, 4, 5 share timer 2
        4: [2, 3],     # RC channels 2, 3 share timer 4
        5: [0, 1, 2],  # ESCs 0, 1, 2 share timer 5
        12: [0, 1],    # RC channels 0, 1 share timer 12
    }
}


# ============================================================================
# ARDUINO/TEENSY CONFIGURATION (Future)
# ============================================================================

ARDUINO_CONFIG = {
    'platform': 'arduino',
    
    # Example configuration for Arduino Mega or Teensy 4.x
    'esc_pins': {
        0: {'pin': 2, 'pwm': True},
        1: {'pin': 3, 'pwm': True},
        2: {'pin': 4, 'pwm': True},
        3: {'pin': 5, 'pwm': True},
        4: {'pin': 6, 'pwm': True},
        5: {'pin': 7, 'pwm': True},
    },
    
    'rc_pins': {
        0: {'pin': 18, 'interrupt': True},  # Throttle
        1: {'pin': 19, 'interrupt': True},  # Roll
        2: {'pin': 20, 'interrupt': True},  # Pitch
        3: {'pin': 21, 'interrupt': True},  # Yaw
    },
    
    'i2c': {
        'sda_pin': 20,
        'scl_pin': 21,
        'mpu6050_address': 0x68,
    }
}


# ============================================================================
# RASPBERRY PI CONFIGURATION (Future)
# ============================================================================

RASPBERRY_PI_CONFIG = {
    'platform': 'raspberry_pi',
    
    # Using BCM pin numbering
    'esc_pins': {
        0: {'pin': 17, 'pwm_channel': 0},
        1: {'pin': 18, 'pwm_channel': 1},
        2: {'pin': 22, 'pwm_channel': 2},
        3: {'pin': 23, 'pwm_channel': 3},
        4: {'pin': 24, 'pwm_channel': 4},
        5: {'pin': 25, 'pwm_channel': 5},
    },
    
    'rc_pins': {
        0: {'pin': 5, 'edge': 'both'},   # Throttle
        1: {'pin': 6, 'edge': 'both'},   # Roll
        2: {'pin': 13, 'edge': 'both'},  # Pitch
        3: {'pin': 19, 'edge': 'both'},  # Yaw
    },
    
    'i2c': {
        'bus': 1,  # /dev/i2c-1
        'mpu6050_address': 0x68,
    }
}


# ============================================================================
# ESP32 CONFIGURATION (Future)
# ============================================================================

ESP32_CONFIG = {
    'platform': 'esp32',
    
    'esc_pins': {
        0: {'pin': 25, 'ledc_channel': 0},
        1: {'pin': 26, 'ledc_channel': 1},
        2: {'pin': 27, 'ledc_channel': 2},
        3: {'pin': 14, 'ledc_channel': 3},
        4: {'pin': 12, 'ledc_channel': 4},
        5: {'pin': 13, 'ledc_channel': 5},
    },
    
    'rc_pins': {
        0: {'pin': 32, 'rmt_channel': 0},  # Throttle
        1: {'pin': 33, 'rmt_channel': 1},  # Roll
        2: {'pin': 34, 'rmt_channel': 2},  # Pitch
        3: {'pin': 35, 'rmt_channel': 3},  # Yaw
    },
    
    'i2c': {
        'sda_pin': 21,
        'scl_pin': 22,
        'mpu6050_address': 0x68,
    }
}


# ============================================================================
# CONFIGURATION LOADER
# ============================================================================

def get_config(platform: str = None):
    """
    Get hardware configuration for specified platform.
    
    Args:
        platform: Platform name ('pyboard', 'arduino', 'raspberry_pi', 'esp32')
                 If None, auto-detect
    
    Returns:
        Configuration dictionary
    """
    if platform is None:
        # Auto-detect platform
        logger.info("Auto-detecting hardware platform...")
        try:
            import pyb
            platform = 'pyboard'
            logger.info("Detected PyBoard")
        except ImportError:
            try:
                import RPi.GPIO
                platform = 'raspberry_pi'
                logger.info("Detected Raspberry Pi")
            except ImportError:
                try:
                    import machine
                    if hasattr(machine, 'Pin'):
                        # Could be ESP32 or other MicroPython board
                        platform = 'esp32'  # Default guess
                        logger.info("Detected ESP32 (MicroPython)")
                except ImportError:
                    platform = 'arduino'  # Final fallback
                    logger.warning("No platform detected, defaulting to Arduino")
    
    configs = {
        'pyboard': PYBOARD_CONFIG,
        'arduino': ARDUINO_CONFIG,
        'raspberry_pi': RASPBERRY_PI_CONFIG,
        'esp32': ESP32_CONFIG,
    }
    
    config = configs.get(platform)
    if config is None:
        available = ', '.join(configs.keys())
        logger.error(f"Invalid platform '{platform}', available: {available}")
        raise ValueError(f"Unknown platform: {platform}")
    
    logger.info(f"Loaded configuration for: {platform}")
    return config


def get_esc_pin(motor_index: int, platform: str = None) -> dict:
    """Get ESC pin configuration for motor index"""
    config = get_config(platform)
    return config['esc_pins'][motor_index]


def get_rc_pin(channel_index: int, platform: str = None) -> dict:
    """Get RC pin configuration for channel index"""
    config = get_config(platform)
    return config['rc_pins'][channel_index]


def get_i2c_config(platform: str = None) -> dict:
    """Get I2C configuration"""
    config = get_config(platform)
    return config['i2c']
