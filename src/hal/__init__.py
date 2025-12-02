"""Hardware Abstraction Layer"""

from .hal import (
    HALPlatform, HALTimer, HALPWMChannel, HALInputCapture, HALI2C
)

__all__ = [
    'HALPlatform', 'HALTimer', 'HALPWMChannel', 'HALInputCapture', 'HALI2C'
]
