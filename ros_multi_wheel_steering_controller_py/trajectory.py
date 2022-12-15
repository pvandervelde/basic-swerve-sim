from abc import ABC, abstractmethod
from enum import Enum
from typing import Mapping, List, Tuple

from .control_model import BodyState
from .errors import IncompleteTrajectoryException
from .geometry import Motion
from .drive_module import DriveModule, DriveModuleProposedState, DriveModuleState
from .profile import LinearProfile, ProfilePoint, TransientValueProfile

class DriveModuleProfile(object):

    def __init__(self, steering_profile: TransientValueProfile, drive_profile: TransientValueProfile):
        self.steering_profile = steering_profile
        self.drive_profile = drive_profile

    def profile_for_steering(self) -> TransientValueProfile:
        return self.steering_profile

    def profile_for_drive(self) -> TransientValueProfile:
        return self.drive_profile

# A collection of position / velocity / acceleration profiles
class BodyMotionTrajectory(object):

    def __init__(self, current: Motion, desired: Motion, trajectory_time_in_seconds: float):
        self.start_state = current
        self.end_state = desired
        self.trajectory_time_in_seconds = trajectory_time_in_seconds

        self.profile = [
            LinearProfile(current.linear_velocity.x, desired.linear_velocity.x),
            LinearProfile(current.linear_velocity.y, desired.linear_velocity.y),
            LinearProfile(current.linear_velocity.z, desired.linear_velocity.z),

            LinearProfile(current.angular_velocity.x, desired.angular_velocity.x),
            LinearProfile(current.angular_velocity.y, desired.angular_velocity.y),
            LinearProfile(current.angular_velocity.z, desired.angular_velocity.z),
        ]

    def inflection_points(self) -> List[List[ProfilePoint]]:
        pass

    def value_at(self, time_fraction: float) -> Motion:
        return Motion(
            self.profile[0].value_at(time_fraction),
            self.profile[1].value_at(time_fraction),
            self.profile[5].value_at(time_fraction)
        )

    def time_span(self) -> float:
        return self.trajectory_time_in_seconds

class DriveModuleStateTrajectory(object):

    def __init__(self, drive_modules: List[DriveModule]):
        self.modules = drive_modules
        self.start_states: List[DriveModuleState] = []
        self.end_states: List[DriveModuleProposedState] = []

        # Kinda want a constant jerk profile
        self.profiles: Mapping[str, List[TransientValueProfile]] = {}

    def align_module_profiles(self):
        if len(self.start_states) == 0 or len(self.end_states) == 0:
            raise IncompleteTrajectoryException()

        # for each profile adjust it in time such that none of the velocities / accelerations are too high for the motors to handle
        # Then scale the profiles to match in time.
        pass

    def _create_profiles(self):
        if len(self.start_states) == 0:
            return

        if len(self.end_states) == 0:
            return

        self.profiles.clear()
        for i in range(len(self.modules)):
            start = self.start_states[i]
            end = self.end_states[i]

            module_profiles = [
                # Orientation
                LinearProfile(start.orientation_in_body_coordinates.z, end.steering_angle_in_radians),

                # Drive velocity
                LinearProfile(start.drive_velocity_in_module_coordinates.x, end.drive_velocity_in_meters_per_second),
            ]

            self.profiles[self.modules[i].name] = module_profiles

    def inflection_points(self) -> List[List[ProfilePoint]]:
        pass

    def set_current_state(self, states: List[DriveModuleState]):
        if len(states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(states) }) does not match the number of drive modules.")

        self.start_states = states
        self._create_profiles()

    def set_desired_end_state(self, states: List[DriveModuleProposedState]):
        if len(states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(states) }) does not match the number of drive modules.")

        self.end_states = states
        self._create_profiles()

    def time_span(self) -> float:
        # Return the time in seconds which the trajectory spans
        return 1.0

    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleState:
        if len(self.start_states) == 0 or len(self.end_states) == 0:
            raise IncompleteTrajectoryException()

        if not id in self.profiles:
            raise ValueError(f"There are no profiles for a drive module with name { id }")

        steering_module: DriveModule = None
        for x in self.modules:
            if x.name == id:
                steering_module = x
                break

        profiles = self.profiles[id]

        return DriveModuleState(
            steering_module.name,
            steering_module.steering_axis_xy_position.x,
            steering_module.steering_axis_xy_position.y,
            profiles[0].value_at(time_fraction),
            profiles[0].first_derivative_at(time_fraction),
            profiles[1].value_at(time_fraction),
            profiles[1].first_derivative_at(time_fraction)
        )
