# local
from geometry import Orientation, Point, Vector3

class Wheel(object):

    def __init__(self):
        pass

    # Radius, width

class Motor(object):

    def __init__(self):
        pass

    # Motor capabilities

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

    def get_current_steering_direction_in_radians(self) -> float:
        return self.steering_direction_in_radians

    def get_current_steering_velocity_in_radians_per_second(self) -> float:
        return self.steering_velocity_in_radians_per_second

    def get_current_steering_acceleration_in_radians_per_second_squared(self) -> float:
        return self.steering_acceleration_in_radians_per_second_squared

    def get_current_drive_velocity_in_meters_per_second(self) -> float:
        return self.drive_velocity_in_meters_per_second

    def get_current_drive_acceleration_in_meters_per_second_squared(self) -> float:
        return self.drive_acceleration_in_meters_per_second_squared

    def set_current_steering_direction_in_radians(self, direction: float):
        self.steering_direction_in_radians = direction

    def set_current_steering_velocity_in_radians_per_second(self, velocity: float):
        self.steering_velocity_in_radians_per_second = velocity

    def set_current_steering_acceleration_in_radians_per_second_squared(self, acceleration: float):
        self.steering_acceleration_in_radians_per_second_squared = acceleration

    def set_current_drive_velocity_in_meters_per_second(self, velocity: float):
        self.drive_velocity_in_meters_per_second = velocity

    def set_current_drive_acceleration_in_meters_per_second_squared(self, acceleration: float):
        self.drive_acceleration_in_meters_per_second_squared = acceleration

    # Motors
    # Wheel
    # Sensors