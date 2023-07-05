from abc import ABC, abstractmethod
import math
from typing import Mapping, List

from swerve_controller.control_model import ControlModelBase

from .errors import IncompleteTrajectoryException
from .drive_module import DriveModule
from .profile import SingleVariableLinearProfile, SingleVariableMultiPointLinearProfile, ProfilePoint, SingleVariableTrapezoidalProfile, TransientVariableProfile
from .states import BodyState, DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion

class DriveModuleProfile(object):

    def __init__(self, steering_profile: TransientVariableProfile, drive_profile: TransientVariableProfile):
        self.steering_profile = steering_profile
        self.drive_profile = drive_profile

    def profile_for_steering(self) -> TransientVariableProfile:
        return self.steering_profile

    def profile_for_drive(self) -> TransientVariableProfile:
        return self.drive_profile

# A collection of position / velocity / acceleration profiles
class LinearBodyMotionProfile(object):

    def __init__(self, current: BodyState, desired: BodyMotion, min_trajectory_time_in_seconds: float):
        self.start_state = current
        self.end_state = desired
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds

        self.profile = [
            SingleVariableTrapezoidalProfile(current.motion_in_body_coordinates.linear_velocity.x, desired.linear_velocity.x),
            SingleVariableTrapezoidalProfile(current.motion_in_body_coordinates.linear_velocity.y, desired.linear_velocity.y),
            SingleVariableTrapezoidalProfile(current.motion_in_body_coordinates.linear_velocity.z, desired.linear_velocity.z),

            SingleVariableTrapezoidalProfile(current.motion_in_body_coordinates.angular_velocity.x, desired.angular_velocity.x),
            SingleVariableTrapezoidalProfile(current.motion_in_body_coordinates.angular_velocity.y, desired.angular_velocity.y),
            SingleVariableTrapezoidalProfile(current.motion_in_body_coordinates.angular_velocity.z, desired.angular_velocity.z),
        ]

    def body_motion_at(self, time_fraction: float) -> BodyMotion:
        return BodyMotion(
            self.profile[0].value_at(time_fraction),
            self.profile[1].value_at(time_fraction),
            self.profile[5].value_at(time_fraction),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def time_span(self) -> float:
        return self.min_trajectory_time_in_seconds

class ModuleStateProfile(ABC):

    @abstractmethod
    def align_module_profiles(self):
        pass

    @abstractmethod
    def time_span(self) -> float:
        pass

    @abstractmethod
    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleMeasuredValues:
        pass

class LinearDriveModuleStateProfile(ModuleStateProfile):

    def __init__(self, drive_modules: List[DriveModule], min_trajectory_time_in_seconds: float):
        self.modules = drive_modules
        self.start_states: List[DriveModuleMeasuredValues] = []
        self.end_states: List[DriveModuleDesiredValues] = []
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds

        # Kinda want a constant jerk profile
        self.profiles: Mapping[str, List[TransientVariableProfile]] = {}

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

            end_steering_angle = end.steering_angle_in_radians if not math.isinf(end.steering_angle_in_radians) else start.orientation_in_body_coordinates.z
            module_profiles = [
                # Orientation
                SingleVariableTrapezoidalProfile(start.orientation_in_body_coordinates.z, end_steering_angle),

                # Drive velocity
                SingleVariableTrapezoidalProfile(start.drive_velocity_in_module_coordinates.x, end.drive_velocity_in_meters_per_second),
            ]

            self.profiles[self.modules[i].name] = module_profiles

    def set_current_state(self, states: List[DriveModuleMeasuredValues]):
        if len(states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(states) }) does not match the number of drive modules.")

        self.start_states = states
        self._create_profiles()

    def set_desired_end_state(self, states: List[DriveModuleDesiredValues]):
        if len(states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(states) }) does not match the number of drive modules.")

        self.end_states = states
        self._create_profiles()

    def time_span(self) -> float:
        return self.min_trajectory_time_in_seconds

    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleMeasuredValues:
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

        return DriveModuleMeasuredValues(
            steering_module.name,
            steering_module.steering_axis_xy_position.x,
            steering_module.steering_axis_xy_position.y,
            profiles[0].value_at(time_fraction),
            profiles[0].first_derivative_at(time_fraction),
            profiles[0].second_derivative_at(time_fraction),
            profiles[0].third_derivative_at(time_fraction),
            profiles[1].value_at(time_fraction),
            profiles[1].first_derivative_at(time_fraction),
            profiles[1].second_derivative_at(time_fraction),
        )

class BodyControlledDriveModuleProfile(ModuleStateProfile):
    def __init__(self, drive_modules: List[DriveModule], control_model: ControlModelBase, min_trajectory_time_in_seconds: float, min_body_to_module_resolution_per_second: float):
        self.modules = drive_modules
        self.control_model = control_model

        self.start_state_modules: List[DriveModuleMeasuredValues] = []
        self.end_state_body: BodyState = None

        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds
        self.min_body_to_module_resolution_per_second = min_body_to_module_resolution_per_second

        # Kinda want a constant jerk profile
        self.module_profiles: Mapping[str, List[SingleVariableMultiPointLinearProfile]] = {}

    def align_module_profiles(self):
        if len(self.start_state_modules) == 0 or len(self.end_state_body) == 0:
            raise IncompleteTrajectoryException()

        # for each profile adjust it in time such that none of the velocities / accelerations are too high for the motors to handle
        # Then scale the profiles to match in time.
        pass

    def _create_profiles(self):
        if len(self.start_state_modules) == 0:
            return

        if self.end_state_body is None:
            return

        drive_module_end_states = self.control_model.state_of_wheel_modules_from_body_motion(self.end_state_body)

        # Create the module profiles with start and end states only. The intermediate states will be added as we
        # iterate through the body profile
        self.module_profiles.clear()
        for i in range(len(self.modules)):
            start = self.start_state_modules[i]
            end = drive_module_end_states[i][0]

            end_steering_angle = end.steering_angle_in_radians if not math.isinf(end.steering_angle_in_radians) else start.orientation_in_body_coordinates.z
            module_profiles = [
                # Orientation
                SingleVariableMultiPointLinearProfile(start.orientation_in_body_coordinates.z, end_steering_angle),

                # Drive velocity
                SingleVariableMultiPointLinearProfile(start.drive_velocity_in_module_coordinates.x, end.drive_velocity_in_meters_per_second),
            ]

            self.module_profiles[self.modules[i].name] = module_profiles

        # Compute the body profile
        start_state_body = self.control_model.body_motion_from_wheel_module_states(self.start_state_modules)
        body_profiles = [
            SingleVariableLinearProfile(start_state_body.linear_velocity.x, self.end_state_body.motion_in_body_coordinates.linear_velocity.x),
            SingleVariableLinearProfile(start_state_body.linear_velocity.y, self.end_state_body.motion_in_body_coordinates.linear_velocity.y),
            SingleVariableLinearProfile(start_state_body.linear_velocity.z, self.end_state_body.motion_in_body_coordinates.linear_velocity.z),

            SingleVariableLinearProfile(start_state_body.angular_velocity.x, self.end_state_body.motion_in_body_coordinates.angular_velocity.x),
            SingleVariableLinearProfile(start_state_body.angular_velocity.y, self.end_state_body.motion_in_body_coordinates.angular_velocity.y),
            SingleVariableLinearProfile(start_state_body.angular_velocity.z, self.end_state_body.motion_in_body_coordinates.angular_velocity.z),
        ]

        # Compute intermediate steps for the modules
        number_of_frames = math.ceil(self.min_trajectory_time_in_seconds * self.min_body_to_module_resolution_per_second)

        # We don't include the start and end item because those are already there
        for i in range(1, number_of_frames):
            time_fraction = float(i) / float(number_of_frames)
            body_motion_at_time = BodyMotion(
                body_profiles[0].value_at(time_fraction),
                body_profiles[1].value_at(time_fraction),
                body_profiles[5].value_at(time_fraction)
            )

            drive_module_states = self.control_model.state_of_wheel_modules_from_body_motion(body_motion_at_time)

            for i in range(len(self.modules)):
                # There are two options. For now pick the first one. Realistically we should be picking the one that matches
                # the previous state and the other modules best
                intermediate_state_for_module = drive_module_states[i][0]

                # Orientation
                self.module_profiles[self.modules[i].name][0].add_value(time_fraction, intermediate_state_for_module.steering_angle_in_radians)

                # Drive velocity
                self.module_profiles[self.modules[i].name][1].add_value(time_fraction, intermediate_state_for_module.drive_velocity_in_meters_per_second)


    def set_current_state(self, module_states: List[DriveModuleMeasuredValues]):
        if len(module_states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(module_states) }) does not match the number of drive modules.")

        self.start_state_modules = module_states
        self._create_profiles()

    def set_desired_end_state(self, body_state: BodyState):
        self.end_state_body = body_state
        self._create_profiles()

    def time_span(self) -> float:
        return self.min_trajectory_time_in_seconds

    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleMeasuredValues:
        if len(self.start_state_modules) == 0 or self.end_state_body is None:
            raise IncompleteTrajectoryException()

        if not id in self.module_profiles:
            raise ValueError(f"There are no profiles for a drive module with name { id }")

        steering_module: DriveModule = None
        for x in self.modules:
            if x.name == id:
                steering_module = x
                break

        profiles = self.module_profiles[id]

        return DriveModuleMeasuredValues(
            steering_module.name,
            steering_module.steering_axis_xy_position.x,
            steering_module.steering_axis_xy_position.y,
            profiles[0].value_at(time_fraction),
            profiles[0].first_derivative_at(time_fraction),
            profiles[0].second_derivative_at(time_fraction),
            profiles[0].third_derivative_at(time_fraction),
            profiles[1].value_at(time_fraction),
            profiles[1].first_derivative_at(time_fraction),
            profiles[1].second_derivative_at(time_fraction),
        )


