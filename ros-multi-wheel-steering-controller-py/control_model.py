#!/usr/bin/python3

from math import acos, cos, degrees, isclose, pow, radians, sin, sqrt, tan
from typing import Mapping, List, Tuple

# ROS message types
from geometry_msgs.msg import Point, Twist

#
class BodyState(object):

    def __init__(self) -> None:
        pass

class DriveModuleState(object):

    def __init__(
        self,
        is_singularity: bool,
        steering_angle: float,
        drive_velocity: float
        ):
        self.is_singularity = is_singularity
        self.steering_angle = steering_angle
        self.drive_velocity = drive_velocity

class ControlModelBase(object):

    def __init__(self):
        pass

    # Forward kinematics
    def state_of_body_frame_from_wheel_module_states(self) -> BodyState:
        pass

    # Inverse kinematics
    def state_of_wheel_modules_from_body_state(self, state: BodyState) -> List[DriveModuleState]:
        pass

class SimpleFourWheelSteeringControlModel(ControlModelBase):

    def __init__(self):
        pass

    # Forward kinematics
    def state_of_body_frame_from_wheel_module_states(self) -> BodyState:
        pass

    # Inverse kinematics
    def state_of_wheel_modules_from_body_state(self, state: BodyState) -> List[DriveModuleState]:
        result = []
        for module in self.modules:
            # Kinematics
            # Literature:
            # - https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383/5
            # -
            # For wheel i
            #  - velocity = sqrt( (v_x - omega * y_i)^2 + (v_y + omega * x_i)^2 )
            #  - angle = acos( (v_x - omega * y_i) / (velocity) ) = asin( (v_y + omega * x_i) / (velocity) )
            #
            # Angle: 0 < alpha < Pi
            #  The angle also needs a differentiation if it should go between Pi and 2Pi
            #
            # This assumes that (x_i, y_i) is the coordinate for the steering axis. And that the steering axis is in z-direction.
            # And that the wheel contact point is on that steering axis
            wheel_x_velocity_in_body_coordinates = state.linear.x - state.angular.z * self.steering_axis_xy_position.y
            wheel_y_velocity_in_body_coordinates = state.linear.y + state.angular.z * self.steering_axis_xy_position.x
            drive_velocity = sqrt(pow(wheel_x_velocity_in_body_coordinates, 2.0) + pow(wheel_y_velocity_in_body_coordinates, 2.0))

            is_singularity: bool = False
            if isclose(drive_velocity, 0.0, 1e-9, 1e-9):
                is_singularity = True
                steering_angle = 0
            else:
                steering_angle = acos(wheel_x_velocity_in_body_coordinates / drive_velocity)

            ws = DriveModuleState(is_singularity, steering_angle, drive_velocity)
            result.append(ws)

        return result


# Implement the Seegmiller algorithms in a different controller


def body_state_from_twist(twist: Twist) -> BodyState:
    pass