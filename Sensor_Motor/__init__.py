"""Motor control package.

Compatibility exports:
- ``motor_guidance`` -> ``guidance``
- ``motor_control``  -> ``control``
"""

from . import control as motor_control
from . import guidance as motor_guidance
from . import motorapp

__all__ = [
    "motorapp",
    "motor_control",
    "motor_guidance",
]

