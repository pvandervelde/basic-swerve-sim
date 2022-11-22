#!/usr/bin/python3

from math import acos, cos, degrees, isclose, pow, radians, sin, sqrt, tan
from typing import Mapping, List, Tuple

# local
from drive_module import DriveModule
from geometry import Orientation, Point, Vector3

class Position(object):
    pass

class Motion(object):

    def __init__(
        self,
        linear_x_velocity_in_meters_per_second: float,
        linear_y_velocity_in_meters_per_second: float,
        angular_z_velocity_in_radians_per_second: float):
        self.linear_velocity = Vector3(linear_x_velocity_in_meters_per_second, linear_y_velocity_in_meters_per_second, 0.0)
        self.angular_velocity = Vector3(0.0, 0.0, angular_z_velocity_in_radians_per_second)

class BodyState(object):

    def __init__(
        self,
        body_x_in_meters: float,
        body_y_in_meters: float,
        body_orientation_in_radians: float,
        body_linear_x_velocity_in_meters_per_second: float,
        body_linear_y_velocity_in_meters_per_second,
        body_angular_z_velocity_in_radians_per_second):

        self.position_in_world_coordinates = Point(body_x_in_meters, body_y_in_meters, 0.0)
        self.orientation_in_world_coordinates = Orientation(0.0, 0.0, body_orientation_in_radians)
        self.motion_in_body_coordinates = Motion(
            body_linear_x_velocity_in_meters_per_second,
            body_linear_y_velocity_in_meters_per_second,
            body_angular_z_velocity_in_radians_per_second)

class DriveModuleState(object):

    def __init__(
        self,
        name: str,
        module_x_in_meters: float,
        module_y_in_meters: float,
        steering_angle: float,
        steering_velocity: float,
        drive_velocity: float,
        drive_acceleration: float,
        ):
        self.name = name
        self.position_in_body_coordinates = Point(module_x_in_meters, module_y_in_meters, 0.0)
        self.orientation_in_body_coordinates = Orientation(0.0, 0.0, steering_angle)
        self.orientation_change_in_body_coordinates = Vector3(0.0, 0.0, steering_velocity)
        self.drive_velocity_in_module_coordinates = Vector3(drive_velocity, 0.0, 0.0)
        self.drive_acceleration_in_module_coordinates = Vector3(drive_acceleration, 0.0, 0.0)

# Abstract class for control models
class ControlModelBase(object):

    def __init__(self):
        pass

    # Forward kinematics
    def body_motion_from_wheel_module_states(self) -> Motion:
        return Motion(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Inverse kinematics
    def state_of_wheel_modules_from_body_motion(self, state: Motion) -> List[DriveModuleState]:
        return []

class SimpleFourWheelSteeringControlModel(ControlModelBase):

    def __init__(self, drive_modules: List[DriveModule]):
        self.modules = drive_modules

    # Forward kinematics
    def body_motion_from_wheel_module_states(self, states: List[DriveModuleState]) -> Motion:
        pass

    # Inverse kinematics
    def state_of_wheel_modules_from_body_motion(self, state: Motion) -> List[DriveModuleState]:
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
            wheel_x_velocity_in_body_coordinates = state.linear_velocity.x - state.angular_velocity.z * module.steering_axis_xy_position.y
            wheel_y_velocity_in_body_coordinates = state.linear_velocity.y + state.angular_velocity.z * module.steering_axis_xy_position.x
            drive_velocity = sqrt(pow(wheel_x_velocity_in_body_coordinates, 2.0) + pow(wheel_y_velocity_in_body_coordinates, 2.0))

            is_singularity: bool = False
            if isclose(drive_velocity, 0.0, 1e-9, 1e-9):
                is_singularity = True
                steering_angle = 0  # THIS IS WRONG. IT SHOULD BE WHAT EVER THE STEERING ANGLE WAS
            else:
                steering_angle = acos(wheel_x_velocity_in_body_coordinates / drive_velocity)

            ws = DriveModuleState(
                module.name,
                module.steering_axis_xy_position.x,
                module.steering_axis_xy_position.y,
                steering_angle,
                steering_velocity,
                drive_velocity,
                drive_acceleration)
            result.append(ws)

        return result


# Implement the Seegmiller algorithms in a different controller
