import math
from typing import Mapping, List, Tuple

# local
from .geometry import Orientation, Point, Vector3

class Wheel(object):

    def __init__(self):
        pass

    # Radius, width

class Motor(object):

    def __init__(self):
        pass

    # Motor capabilities
    # - RPM curve
    # - Torgue

class Encoder(object):

    def __init__(self):
        pass

    # Provide current position and velocity

class DriveModule(object):

    def __init__(
        self,
        name: str,
        steering_link: str,
        drive_link: str,
        steering_axis_xy_position: Point,
        wheel_radius: float,
        steering_motor_maximum_velocity: float,
        steering_motor_minimum_acceleration: float,
        steering_motor_maximum_acceleration: float,
        drive_motor_maximum_velocity: float,
        drive_motor_minimum_acceleration: float,
        drive_motor_maximum_acceleration: float):

        self.name = name

        self.steering_link_name = steering_link
        self.driving_link_name = drive_link

        # Assume a vertical steering axis that goes through the center of the wheel (i.e. no steering offset)
        self.steering_axis_xy_position = steering_axis_xy_position
        self.wheel_radius = wheel_radius

        self.steering_motor_maximum_velocity = steering_motor_maximum_velocity

        self.steering_motor_minimum_acceleration = steering_motor_minimum_acceleration
        self.steering_motor_maximum_acceleration = steering_motor_maximum_acceleration

        self.drive_motor_maximum_velocity = drive_motor_maximum_velocity

        self.drive_motor_minimum_acceleration = drive_motor_minimum_acceleration
        self.drive_motor_maximum_acceleration = drive_motor_maximum_acceleration

    # Motors
    # Wheel
    # Sensors

# Defines the required combination of steering angle and drive velocity for a given drive module in order
# to achieve a given Motion of the robot body.
class DriveModuleProposedState(object):

    def __init__(
        self,
        name: str,
        steering_angle_in_radians: float,
        drive_velocity_in_meters_per_second: float,
        ):
        self.name = name
        self.steering_angle_in_radians = steering_angle_in_radians
        self.drive_velocity_in_meters_per_second = drive_velocity_in_meters_per_second

class DriveModuleState(object):

    def __init__(
        self,
        name: str,
        module_x_in_meters: float,
        module_y_in_meters: float,
        steering_angle: float,
        steering_velocity: float,
        steering_acceleration: float,
        steering_jerk: float,
        drive_velocity: float,
        drive_acceleration: float,
        drive_jerk: float
        ):
        self.name = name
        self.position_in_body_coordinates = Point(module_x_in_meters, module_y_in_meters, 0.0)
        self.orientation_in_body_coordinates = Orientation(0.0, 0.0, steering_angle)

        self.drive_velocity_in_module_coordinates = Vector3(drive_velocity, 0.0, 0.0)
        self.orientation_velocity_in_body_coordinates = Vector3(0.0, 0.0, steering_velocity)

        self.drive_acceleration_in_module_coordinates = Vector3(drive_acceleration, 0.0, 0.0)
        self.orientation_acceleration_in_body_coordinates = Vector3(0.0, 0.0, steering_acceleration)

        self.drive_jerk_in_module_coordinates = Vector3(drive_jerk, 0.0, 0.0)
        self.orientation_jerk_in_body_coordinates = Vector3(0.0, 0.0, steering_jerk)

    def xy_drive_velocity(self) -> Tuple[float, float]:
        v_x = self.drive_velocity_in_module_coordinates.x * math.cos(self.orientation_in_body_coordinates.z)
        v_y = self.drive_velocity_in_module_coordinates.x * math.sin(self.orientation_in_body_coordinates.z)

        return v_x, v_y
