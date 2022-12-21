import math
import pytest
from typing import Mapping, List, Tuple

# locals
from .control_model import BodyState, DriveModuleState, Motion, SimpleFourWheelSteeringControlModel
from .drive_module import DriveModule
from .errors import IncompleteTrajectoryException
from .geometry import Point
from .trajectory import BodyMotionTrajectory, DriveModuleProfile, DriveModuleStateTrajectory


def test_controller_should_update_body_state_on_state_update():
    pass

def test_controller_should_update_trajectory_on_desired_state_update():
    pass
