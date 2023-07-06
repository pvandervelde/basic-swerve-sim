import math
import pytest
from typing import Mapping, List, Tuple

# locals
from swerve_controller.control_model import DriveModuleMeasuredValues, BodyMotion, SimpleFourWheelSteeringControlModel
from swerve_controller.drive_module import DriveModule
from swerve_controller.errors import IncompleteTrajectoryException
from swerve_controller.geometry import Point
from swerve_controller.sim_utils import get_intersect
from swerve_controller.states import BodyState
from swerve_controller.control_profile import BodyMotionProfile, DriveModuleStateProfile

def test_intersect_with_parallel_lines():
    intersection_point = get_intersect(
        Point(0.0, 1.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(1.0, 10.0, 0.0),
        Point(1.0, 9.0, 0.0)
    )

    assert math.isinf(intersection_point.x)
    assert math.isinf(intersection_point.y)
    assert math.isclose(intersection_point.z, 0.0, rel_tol=1e-15, abs_tol=1e-15)

def test_intersect_with_vertical_and_horizontal_lines():
    intersection_point = get_intersect(
        Point(0.0, 1.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(1.0, 10.0, 0.0),
        Point(2.0, 10.0, 0.0)
    )

    assert math.isclose(intersection_point.x, 0.0, rel_tol=1e-15, abs_tol=1e-15)
    assert math.isclose(intersection_point.y, 10.0, rel_tol=1e-15, abs_tol=1e-15)
    assert math.isclose(intersection_point.z, 0.0, rel_tol=1e-15, abs_tol=1e-15)

def test_intersect_with_intersecting_lines():
    intersection_point = get_intersect(
        Point(0.0, 1.0, 0.0),
        Point(1.0, 2.0, 0.0),
        Point(0.0, 10.0, 0.0),
        Point(1.0, 9.0, 0.0)
    )

    assert math.isclose(intersection_point.x, 4.5, rel_tol=1e-15, abs_tol=1e-15)
    assert math.isclose(intersection_point.y, 5.5, rel_tol=1e-15, abs_tol=1e-15)
    assert math.isclose(intersection_point.z, 0.0, rel_tol=1e-15, abs_tol=1e-15)

def test_controller_should_update_body_state_on_state_update():
    pass

def test_controller_should_update_trajectory_on_desired_state_update():
    pass
