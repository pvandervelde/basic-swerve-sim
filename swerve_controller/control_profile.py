from abc import ABC, abstractmethod
import math
from typing import Callable, Mapping, List, Tuple

from swerve_controller.control_model import difference_between_angles, ControlModelBase
from swerve_controller.geometry import LinearUnboundedSpace, PeriodicBoundedCircularSpace, RealNumberValueSpace

from .errors import IncompleteTrajectoryException
from .drive_module import DriveModule
from .profile import SingleVariableLinearProfile, SingleVariableMultiPointLinearProfile, ProfilePoint, SingleVariableTrapezoidalProfile, TransientVariableProfile
from .states import BodyState, DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion

# A collection of position / velocity / acceleration profiles
class BodyMotionProfile(object):

    def __init__(
            self,
            current: BodyState,
            desired: BodyMotion,
            min_trajectory_time_in_seconds: float,
            motion_profile_func: Callable[[float, float, RealNumberValueSpace], TransientVariableProfile]):
        self.start_state = current
        self.end_state = desired
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds

        self.profile = [
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.x, desired.linear_velocity.x, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.y, desired.linear_velocity.y, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.z, desired.linear_velocity.z, LinearUnboundedSpace()),

            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.x, desired.angular_velocity.x, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.y, desired.angular_velocity.y, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.z, desired.angular_velocity.z, LinearUnboundedSpace()),
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
    def time_span(self) -> float:
        pass

    @abstractmethod
    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleMeasuredValues:
        pass

class DriveModuleStateProfile(ModuleStateProfile):

    def __init__(
            self,
            drive_modules: List[DriveModule],
            min_trajectory_time_in_seconds: float,
            motion_profile_func: Callable[[float, float, RealNumberValueSpace], TransientVariableProfile]):
        self.modules = drive_modules
        self.motion_profile_func = motion_profile_func
        self.start_states: List[DriveModuleMeasuredValues] = []
        self.end_states: List[DriveModuleDesiredValues] = []
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds

        # Kinda want a constant jerk profile
        self.profiles: Mapping[str, List[TransientVariableProfile]] = {}

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
                self.motion_profile_func(start.orientation_in_body_coordinates.z, end_steering_angle, PeriodicBoundedCircularSpace()),

                # Drive velocity
                self.motion_profile_func(start.drive_velocity_in_module_coordinates.x, end.drive_velocity_in_meters_per_second, LinearUnboundedSpace()),
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

class DriveModuleCalculatedProfilePoint(object):
    def __init__(self, time_from_profile_start_in_seconds:float, steering_angle: float, drive_speed: float):
        self.time_from_profile_start_in_seconds = time_from_profile_start_in_seconds
        self.steering_angle = steering_angle
        self.drive_speed = drive_speed

class BodyControlledDriveModuleProfile(ModuleStateProfile):
    def __init__(
            self,
            drive_modules: List[DriveModule],
            control_model: ControlModelBase,
            min_trajectory_time_in_seconds: float,
            min_body_to_module_resolution_per_second: float,
            motion_profile_func: Callable[[float, float, RealNumberValueSpace], TransientVariableProfile]):
        self.modules = drive_modules
        self.control_model = control_model
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds
        self.motion_profile_func = motion_profile_func

        self.start_state_modules: List[DriveModuleMeasuredValues] = []
        self.end_state_body: BodyMotion = None

        self.min_body_to_module_resolution_per_second = min_body_to_module_resolution_per_second

        self.unbounded_number_space = LinearUnboundedSpace()
        self.steering_number_space = PeriodicBoundedCircularSpace()

        # Kinda want a constant jerk profile
        self.module_profiles: Mapping[str, List[SingleVariableMultiPointLinearProfile]] = {}

    def _create_profiles(self):
        if len(self.start_state_modules) == 0:
            return

        if self.end_state_body is None:
            return

        # Create body trajectory
        drive_module_end_states = self.control_model.state_of_wheel_modules_from_body_motion(self.end_state_body)

        # Create the module profiles with start and end states only. The intermediate states will be added as we
        # iterate through the body profile

        calculated_profiles: Mapping[str, List[DriveModuleCalculatedProfilePoint]] = {}
        for module_index in range(len(self.modules)):
            start = self.start_state_modules[module_index]
            end = drive_module_end_states[module_index][0]

            end_steering_angle = end.steering_angle_in_radians if not math.isinf(end.steering_angle_in_radians) else start.orientation_in_body_coordinates.z
            calculated_profiles[self.modules[module_index].name] = [
                DriveModuleCalculatedProfilePoint(
                    0.0,
                    self.steering_number_space.normalize_value(start.orientation_in_body_coordinates.z),
                    start.drive_velocity_in_module_coordinates.x
                ),
                DriveModuleCalculatedProfilePoint(
                    1.0,
                    self.steering_number_space.normalize_value(end_steering_angle),
                    end.drive_velocity_in_meters_per_second
                )
            ]

        # Compute the body profile
        start_state_body = self.control_model.body_motion_from_wheel_module_states(self.start_state_modules)
        body_profiles = [
            self.motion_profile_func(start_state_body.linear_velocity.x, self.end_state_body.linear_velocity.x, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.linear_velocity.y, self.end_state_body.linear_velocity.y, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.linear_velocity.z, self.end_state_body.linear_velocity.z, self.unbounded_number_space),

            self.motion_profile_func(start_state_body.angular_velocity.x, self.end_state_body.angular_velocity.x, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.angular_velocity.y, self.end_state_body.angular_velocity.y, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.angular_velocity.z, self.end_state_body.angular_velocity.z, self.unbounded_number_space),
        ]

        # Compute intermediate steps for the modules. Assume that the whole profile is 1 second long
        # later on we will change the profile time to ensure that we are within the bounds of the
        # motor capabilities
        number_of_frames = math.ceil(1.0 * self.min_body_to_module_resolution_per_second)

        # We don't include the start and end item because those are already there
        for frame_index in range(1, number_of_frames):
            time_fraction = float(frame_index) / float(number_of_frames)
            body_motion_at_time = BodyMotion(
                body_profiles[0].value_at(time_fraction),
                body_profiles[1].value_at(time_fraction),
                body_profiles[5].value_at(time_fraction),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

            drive_module_states = self.control_model.state_of_wheel_modules_from_body_motion(body_motion_at_time)

            for module_index in range(len(self.modules)):
                # Grab the previous steering angle and velocity as the profile has calculated it. From there we
                # can determine differences, steering velocities/accelerations and drive accelerations.
                previous_steering_angle = self.steering_number_space.normalize_value(calculated_profiles[self.modules[module_index].name][0][frame_index - 1])
                previous_velocity = calculated_profiles[self.modules[module_index].name][1][frame_index - 1]

                next_states_for_module = drive_module_states[module_index]

                first_state_rotation_difference = self.steering_number_space.smallest_distance_between_values(previous_steering_angle, next_states_for_module[0].steering_angle_in_radians)
                second_state_rotation_difference = self.steering_number_space.smallest_distance_between_values(previous_steering_angle, next_states_for_module[1].steering_angle_in_radians)

                first_state_velocity_difference = next_states_for_module[0].drive_velocity_in_meters_per_second - previous_velocity
                second_state_velocity_difference = next_states_for_module[1].drive_velocity_in_meters_per_second - previous_velocity

                # Wheels are moving. We don't know what kind of movement yet though, so figure out if:
                # - The wheel are moving at some significant velocity, in that case pick the state that most
                #   closely matches the current state, i.e. match the drive velocity and the steering angle as
                #   close as possible
                # - The wheel is moving slowly, in that case we may just be close to the moment where the wheel
                #   stops moving (either just before it does that, or just after). This is where we could potentially
                #   flip directions (or we might just have flipped directions)
                #   - If we have just flipped directions then we should probably continue in the same way (but maybe not)

                # Possibilities:
                # - first velocity change and first orientation change are the smallest -> pick the first state
                # - second velocity change and second orientation change are the smallest -> pick the second state
                # - first velocity change is larger and second orientation change is larger -> Bad state. Pick the one with the least relative change?

                desired_state: DriveModuleDesiredValues = None
                if abs(first_state_rotation_difference) <= abs(second_state_rotation_difference):
                    if abs(first_state_velocity_difference) <= abs(second_state_velocity_difference):
                        # first rotation and velocity change are the smallest, so take the first state
                        desired_state = next_states_for_module[0]
                    else:
                        if math.isclose(abs(first_state_rotation_difference), abs(second_state_rotation_difference), rel_tol=1e-7, abs_tol=1e-7):
                            # first rotation is equal to the second rotation
                            # first velocity larger than the second velocity.
                            # pick the second state
                            desired_state = next_states_for_module[1]
                        else:
                            # first rotation is the smallest but second velocity is the smallest
                            desired_state = next_states_for_module[0]
                else:
                    if abs(second_state_velocity_difference) <= abs(first_state_velocity_difference):
                        # second rotation and velocity change are the smallest, so take the second state
                        desired_state = next_states_for_module[1]
                    else:
                        if math.isclose(abs(first_state_rotation_difference), abs(second_state_rotation_difference), rel_tol=1e-7, abs_tol=1e-7):
                            # second rotation is equal to the first rotation
                            # second velocity larger than the first velocity.
                            # pick the first state
                            desired_state = next_states_for_module[0]
                        else:
                            # second rotation is the smallest but first velocity is the smallest
                            desired_state = next_states_for_module[1]

                calculated_profiles[self.modules[module_index].name].insert(
                    frame_index,
                    DriveModuleCalculatedProfilePoint(
                        float(frame_index) / float(number_of_frames),
                        self.steering_number_space.normalize_value(desired_state.steering_angle_in_radians),
                        desired_state.drive_velocity_in_meters_per_second,
                    )
                )

        # TODO apply limits for steering velocity, wheel velocity and accelerations

        profiles: Mapping[str, List[SingleVariableMultiPointLinearProfile]] = {}
        for module_index in range(len(self.modules)):
            point = calculated_profiles[self.modules[module_index].name]

            completed_profiles: List[SingleVariableMultiPointLinearProfile] = [
                # Steering orientation
                SingleVariableMultiPointLinearProfile(point[0].steering_angle, point[-1].steering_angle, PeriodicBoundedCircularSpace()),

                # Drive velocity
                SingleVariableMultiPointLinearProfile(point[0].drive_speed, point[-1].drive_speed)
            ]
            for i in range(1, len(point[0]) - 1):
                completed_profiles[0].add_value(point[i].time_from_profile_start_in_seconds, point[i].steering_angle)
                completed_profiles[1].add_value(point[i].time_from_profile_start_in_seconds, point[i].drive_speed)

            profiles[self.modules[module_index].name] = completed_profiles

        self.module_profiles = profiles

    def set_current_state(self, module_states: List[DriveModuleMeasuredValues]):
        if len(module_states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(module_states) }) does not match the number of drive modules.")

        self.start_state_modules = module_states
        self._create_profiles()

    def set_desired_end_state(self, body_state: BodyMotion):
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
