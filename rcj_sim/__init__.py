"""RoboCupJunior Soccer simulator.

Robot programs import the object names from here::

    from rcj_sim import BALL, BLUE_GOAL, YELLOW_GOAL
"""
from .constants import BALL, BLUE_GOAL, YELLOW_GOAL
from .api import Robot

__all__ = ["Robot", "BALL", "BLUE_GOAL", "YELLOW_GOAL"]
__version__ = "1.0.0"
