"""
pytest configuration for flight controller tests

Adds src directory to Python path so tests can import modules.
"""

import sys
from pathlib import Path

# Add src directory to Python path
src_path = Path(__file__).parent
if str(src_path) not in sys.path:
    sys.path.insert(0, str(src_path))
