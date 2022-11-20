from abc import ABC, abstractmethod
from enum import Enum
from typing import Mapping, List, Tuple

from control_model import BodyState, DriveModuleState
from drive_module import DriveModule

class ProfileType(Enum):
    POLYNOME = 1
    SPLINE = 2
    BSPLINE = 3
    NURBS = 4

class TrajectoryProfile(ABC):

    @abstractmethod
    def order(self) -> int:
        pass

    @abstractmethod
    def value_at(self, location: float) -> float:
        pass

class LinearProfile(TrajectoryProfile):

    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end

    def order(self) -> int:
        return 1

    def value_at(self, location_fraction: float) -> float:
        if location_fraction < 0.0:
            return self.start

        if location_fraction > 1.0:
            return self.end

        return (self.end - self.start) * location_fraction + self.start

# Quadratic model
# Cubic model
# Spline model
# B-spline model
# Nurbs model

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
class BodyStateTrajectory(object):

    def __init__(self, current: BodyState, desired: BodyState):
        self.profile = [
            profile_for_type()
        ]

    def value_at(self, time_fraction: float) -> BodyState:
        pass

    def time_span(self) -> float:
        # Return the time in seconds which the trajectory spans
        pass

class DriveModuleStateTrajectory(object):

    def __init__(self, trajectory_approximation_order: int, drive_modules: List[DriveModule]):
        self.order = trajectory_approximation_order
        self.profiles = []

    def align_module_profiles(self):
        pass

    def required_number_of_intermediate_points(self) -> int:
        return (self.order + 1) - 2

    def set_current_state(self, states: List[DriveModuleState]):
        pass

    def set_desired_end_state(self, states: List[DriveModuleState]):
        pass

    def time_span(self) -> float:
        # Return the time in seconds which the trajectory spans
        pass

    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleState:
        pass
