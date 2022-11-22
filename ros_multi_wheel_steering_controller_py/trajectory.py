from abc import ABC, abstractmethod
from enum import Enum
from typing import Mapping, List, Tuple

from control_model import BodyState, DriveModuleState, Motion
from drive_module import DriveModule

class ProfileType(Enum):
    POLYNOME = 1
    SPLINE = 2
    BSPLINE = 3
    NURBS = 4

class TrajectoryProfile(ABC):

    @abstractmethod
    def acceleration_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def value_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def velocity_at(self, time_fraction: float) -> float:
        pass

class JerkLimitedProfile(TrajectoryProfile):

    def __init__(self, start_position: float, start_velocity: float, start_acceleration: float, end_position: float):
        self.start_position = start_position
        self.start_velocity = start_velocity
        self.start_acceleration = start_acceleration
        self.end_position = end_position

    def acceleration_at(self, time_fraction: float) -> float:
        pass

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start_position

        if time_fraction > 1.0:
            return self.end_position

        return (self.end_position - self.start_position) * time_fraction + self.start_position

    def velocity_at(self, time_fraction: float) -> float:
        pass

def profile_for_type(type: ProfileType, order: int, values: List[float]) -> TrajectoryProfile:
    if isinstance(ProfileType.POLYNOME, type):
        pass
    pass

class DriveModuleProfile(object):

    def __init__(self, steering_profile: TrajectoryProfile, drive_profile: TrajectoryProfile):
        self.steering_profile = steering_profile
        self.drive_profile = drive_profile

    def profile_for_steering(self) -> TrajectoryProfile:
        return self.steering_profile

    def profile_for_drive(self) -> TrajectoryProfile:
        return self.drive_profile

# A collection of position / velocity / acceleration profiles
class BodyMotionTrajectory(object):

    def __init__(self, current: Motion, desired: Motion):
        self.profile = [
            profile_for_type()
        ]

    def value_at(self, time_fraction: float) -> Motion:
        pass

    def time_span(self) -> float:
        # Return the time in seconds which the trajectory spans
        pass

class DriveModuleStateTrajectory(object):

    def __init__(self, drive_modules: List[DriveModule]):
        self.modules = drive_modules

        # Kinda want a constant jerk profile
        self.profiles = []

    def align_module_profiles(self):
        # for each profile adjust it in time such that none of the velocities / accelerations are too high for the motors to handle
        # Then scale the profiles to match in time.
        pass

    def set_current_state(self, states: List[DriveModuleState]):
        pass

    def set_desired_end_state(self, states: List[DriveModuleState]):
        pass

    def time_span(self) -> float:
        # Return the time in seconds which the trajectory spans
        pass

    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleState:
        pass
