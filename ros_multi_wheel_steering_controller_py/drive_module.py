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

