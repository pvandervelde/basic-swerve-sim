from abc import ABC, abstractmethod
import math
from typing import Callable, Mapping, List, Tuple

from swerve_controller.control_model import ControlModelBase
from swerve_controller.geometry import LinearUnboundedSpace, PeriodicBoundedCircularSpace, RealNumberValueSpace

from .errors import IncompleteTrajectoryException
from .drive_module import DriveModule
from .profile import SingleVariableLinearProfile, SingleVariableMultiPointLinearProfile, ProfilePoint, SingleVariableTrapezoidalProfile, TransientVariableProfile
from .states import BodyState, DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion

# Helper functions

from typing import List, Tuple
from numbers import Real

def select_directions_for_modules(
        drive_modules: List[DriveModule],
        steering_number_space: RealNumberValueSpace,
        previous_steering_angles: List[float],
        previous_drive_velocities: List[float],
        drive_module_states: List[Tuple[DriveModuleDesiredValues, DriveModuleDesiredValues]]) -> Tuple[List[float], List[float]]:
    """
    Selects the desired steering angles and drive velocities for each drive module based on the previous states and possible current states.

    Args:
        drive_modules (List[DriveModule]): List of drive modules.
        steering_number_space (RealNumberValueSpace): Describe the rational value space for the steering angle.
        previous_steering_angles (List[float]): List of previous steering angles for each drive module.
        previous_drive_velocities (List[float]): List of previous drive velocities for each drive module.
        drive_module_states (List[Tuple[DriveModuleDesiredValues, DriveModuleDesiredValues]]): List of tuples representing the two possible next states for each drive module.

    Returns:
        Tuple[List[float], List[float]]: A tuple containing the selected steering angles and drive velocities for each drive module.
    """
    current_steering_orientation: List[float] = []
    current_drive_velocity: List[float] = []

    for module_index in range(len(drive_modules)):
        module_previous_steering_angle = steering_number_space.normalize_value(previous_steering_angles[module_index])
        module_previous_drive_velocity = previous_drive_velocities[module_index]

        next_states_for_module = drive_module_states[module_index]

        if math.isinf(next_states_for_module[0].steering_angle_in_radians):
            next_states_for_module[0].steering_angle_in_radians = module_previous_steering_angle

        if math.isinf(next_states_for_module[1].steering_angle_in_radians):
            next_states_for_module[0].steering_angle_in_radians = steering_number_space.normalize_value(module_previous_steering_angle + math.pi)

        first_state_rotation_difference = steering_number_space.smallest_distance_between_values(module_previous_steering_angle, next_states_for_module[0].steering_angle_in_radians)
        second_state_rotation_difference = steering_number_space.smallest_distance_between_values(module_previous_steering_angle, next_states_for_module[1].steering_angle_in_radians)

        first_state_velocity_difference = next_states_for_module[0].drive_velocity_in_meters_per_second - module_previous_drive_velocity
        second_state_velocity_difference = next_states_for_module[1].drive_velocity_in_meters_per_second - module_previous_drive_velocity

        desired_state: DriveModuleDesiredValues = None
        if abs(first_state_rotation_difference) <= abs(second_state_rotation_difference):
            if abs(first_state_velocity_difference) <= abs(second_state_velocity_difference):
                desired_state = next_states_for_module[0]
            else:
                if math.isclose(abs(first_state_rotation_difference), abs(second_state_rotation_difference), rel_tol=1e-7, abs_tol=1e-7):
                    desired_state = next_states_for_module[1]
                else:
                    desired_state = next_states_for_module[0]
        else:
            if abs(second_state_velocity_difference) <= abs(first_state_velocity_difference):
                desired_state = next_states_for_module[1]
            else:
                if math.isclose(abs(first_state_rotation_difference), abs(second_state_rotation_difference), rel_tol=1e-7, abs_tol=1e-7):
                    desired_state = next_states_for_module[0]
                else:
                    desired_state = next_states_for_module[1]

        current_steering_orientation.append(steering_number_space.normalize_value(desired_state.steering_angle_in_radians))
        current_drive_velocity.append(desired_state.drive_velocity_in_meters_per_second)

    return (current_steering_orientation, current_drive_velocity)

# A collection of position / velocity / acceleration profiles
class BodyMotionProfile(object):

    def __init__(
            self,
            current: BodyState,
            desired: BodyMotion,
            min_trajectory_time_in_seconds: float,
            motion_profile_func: Callable[[float, float, float, RealNumberValueSpace], TransientVariableProfile]):
        self.start_state = current
        self.end_state = desired
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds

        self.profile = [
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.x, desired.linear_velocity.x, min_trajectory_time_in_seconds, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.y, desired.linear_velocity.y, min_trajectory_time_in_seconds, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.z, desired.linear_velocity.z, min_trajectory_time_in_seconds, LinearUnboundedSpace()),

            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.x, desired.angular_velocity.x, min_trajectory_time_in_seconds, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.y, desired.angular_velocity.y, min_trajectory_time_in_seconds, LinearUnboundedSpace()),
            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.z, desired.angular_velocity.z, min_trajectory_time_in_seconds, LinearUnboundedSpace()),
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
            motion_profile_func: Callable[[float, float, float, RealNumberValueSpace], TransientVariableProfile]):
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
                self.motion_profile_func(start.orientation_in_body_coordinates.z, end_steering_angle, self.min_trajectory_time_in_seconds, PeriodicBoundedCircularSpace()),

                # Drive velocity
                self.motion_profile_func(start.drive_velocity_in_module_coordinates.x, end.drive_velocity_in_meters_per_second, self.min_trajectory_time_in_seconds, LinearUnboundedSpace()),
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

    def value_for_module_at(self, id: str, time_since_start_of_profile: float) -> DriveModuleMeasuredValues:
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
            profiles[0].value_at(time_since_start_of_profile),
            profiles[0].first_derivative_at(time_since_start_of_profile),
            profiles[0].second_derivative_at(time_since_start_of_profile),
            profiles[0].third_derivative_at(time_since_start_of_profile),
            profiles[1].value_at(time_since_start_of_profile),
            profiles[1].first_derivative_at(time_since_start_of_profile),
            profiles[1].second_derivative_at(time_since_start_of_profile),
        )

class DriveModuleCalculatedProfilePoint(object):
    def __init__(self, time_for_segment_in_seconds:float, steering_angle: float, drive_speed: float):
        self.time_for_segment_in_seconds = time_for_segment_in_seconds
        self.steering_angle = steering_angle
        self.drive_speed = drive_speed

class ValueDerrivativeSet(object):
    def __init__(self, value: float, first_derivative: float, second_derivative: float, third_derivative: float):
        self.value = value
        self.first_derivative = first_derivative
        self.second_derivative = second_derivative
        self.third_derivative = third_derivative

class TimeStatePair(object):
    def __init__(self, time_fraction: float, state: List[ValueDerrivativeSet]):
        self.time_fraction = time_fraction
        self.state = state

class LimitedDriveModuleProfile(object):
    def __init__(self, drive_modules: List[DriveModule]):
        self.drive_modules = drive_modules

        # Store the steering angles for each point in time for each module
        # The first value is the time fraction, the second value is a list of steering angles for each module
        self.steering_profiles: List[TimeStatePair] = []

        # Store the drive velocities for each point in time for each module
        # The first value is the time fraction, the second value is a list of drive velocities for each module
        self.velocity_profiles: List[TimeStatePair] = []

    def add_profile_point(self, time_step_leading_up_to_value: float, steering_angle: List[float], drive_velocity: List[float]):
        steering_mapping: List[ValueDerrivativeSet] = []
        drive_mapping: List[ValueDerrivativeSet] = []
        for index in range(len(self.drive_modules)):
            steering_mapping.append(ValueDerrivativeSet(steering_angle[index], 0.0, 0.0, 0.0))
            drive_mapping.append(ValueDerrivativeSet(drive_velocity[index], 0.0, 0.0, 0.0))

        self.steering_profiles.append(TimeStatePair(time_step_leading_up_to_value, steering_mapping))
        self.velocity_profiles.append(TimeStatePair(time_step_leading_up_to_value, drive_mapping))

    def calculate_accelerations(self):
        # Steering angle
        for index in range(len(self.steering_profiles)):
            current_points = self.steering_profiles[index]

            if index == 0:
                # First point
                # TODO: If we ever get non-zero velocities / accelerations then we need to plug those in here
                for module_index in range(len(self.drive_modules)):
                    current_points.state[module_index].second_derivative = 0.0
            else:
                # Not the first point or the last point
                previous_points = self.steering_profiles[index - 1]

                time_difference_in_the_past = current_points.time_fraction

                for module_index in range(len(self.drive_modules)):
                    previous_value = previous_points.state[module_index].first_derivative
                    current_value = current_points.state[module_index].first_derivative

                    acceleration_from_past = (current_value - previous_value) / time_difference_in_the_past
                    self.steering_profiles[index].state[module_index].second_derivative = acceleration_from_past

        # Drive velocity
        for index in range(len(self.velocity_profiles)):
            current_points = self.velocity_profiles[index]

            if index == 0:
                # First point
                # TODO: If we ever get non-zero velocities / accelerations then we need to plug those in here
                for module_index in range(len(self.drive_modules)):
                    current_points.state[module_index].first_derivative = 0.0
            elif index == len(self.velocity_profiles) - 1:
                # Last point
                for module_index in range(len(self.drive_modules)):
                    current_points.state[module_index].first_derivative = 0.0
            else:
                # Not the first point or the last point
                previous_points = self.velocity_profiles[index - 1]

                time_difference_in_the_past = current_points.time_fraction

                for module_index in range(len(self.drive_modules)):
                    previous_value = previous_points.state[module_index].value
                    current_value = current_points.state[module_index].value

                    acceleration_from_past = (current_value - previous_value) / time_difference_in_the_past
                    self.velocity_profiles[index].state[module_index].first_derivative = acceleration_from_past

    def calculate_derrivatives(self):
        self.calculate_velocities()
        self.calculate_accelerations()
        self.calculate_jerks()

    def calculate_jerks(self):
        # Steering angle
        for index in range(len(self.steering_profiles)):
            current_points = self.steering_profiles[index]

            if index == 0:
                # First point
                # TODO: If we ever get non-zero velocities / accelerations then we need to plug those in here
                for module_index in range(len(self.drive_modules)):
                    current_points.state[module_index].third_derivative = 0.0
            else:
                # Not the first point or the last point
                previous_points = self.steering_profiles[index - 1]

                time_difference_in_the_past = current_points.time_fraction

                for module_index in range(len(self.drive_modules)):
                    previous_value = previous_points.state[module_index].second_derivative
                    current_value = current_points.state[module_index].second_derivative

                    jerk_from_past = (current_value - previous_value) / time_difference_in_the_past
                    self.steering_profiles[index].state[module_index].third_derivative = jerk_from_past

        # Drive velocity
        for index in range(len(self.velocity_profiles)):
            current_points = self.velocity_profiles[index]

            if index == 0:
                # First point
                # TODO: If we ever get non-zero velocities / accelerations then we need to plug those in here
                for module_index in range(len(self.drive_modules)):
                    current_points.state[module_index].second_derivative = 0.0
            elif index == len(self.velocity_profiles) - 1:
                # Last point
                for module_index in range(len(self.drive_modules)):
                    current_points.state[module_index].second_derivative = 0.0
            else:
                # Not the first point or the last point
                previous_points = self.velocity_profiles[index - 1]

                time_difference_in_the_past = current_points.time_fraction

                for module_index in range(len(self.drive_modules)):
                    previous_value = previous_points.state[module_index].first_derivative
                    current_value = current_points.state[module_index].first_derivative

                    jerk_from_past = (current_value - previous_value) / time_difference_in_the_past
                    self.velocity_profiles[index].state[module_index].second_derivative = jerk_from_past

    def calculate_velocities(self):
        # Only do the steering velocity because there is no need to calculate the drive velocity as it is already calculated
        for index in range(len(self.steering_profiles)):
            current_points = self.steering_profiles[index]

            if index == 0:
                # First point
                # TODO: If we ever get non-zero velocities / accelerations then we need to plug those in here
                for module_index in range(len(self.drive_modules)):
                    current_points.state[module_index].first_derivative = 0.0
            else:
                # Not the first point or the last point
                previous_points = self.steering_profiles[index - 1]

                time_difference_in_the_past = current_points.time_fraction

                for module_index in range(len(self.drive_modules)):
                    previous_value = previous_points.state[module_index].value
                    current_value = current_points.state[module_index].value

                    velocity_from_past = (current_value - previous_value) / time_difference_in_the_past
                    self.steering_profiles[index].state[module_index].first_derivative = velocity_from_past

    def limit_profiles(self):
        self.calculate_derrivatives()

        # When aligning profiles we want to align the steering angle/velocity/acceleration/jerk, and then the drive
        # velocity/acceleration/jerk
        # It seems that the drive velocity alignment doesn't influence the steering angle, however
        # changes to the steering angle change the drive velocity. This assumes that the ratio between the
        # drive velocities for the different drive modules is constant.

        # Limiting approach
        # - Limit the value:
        #       Set value to the max / min and keep it there -> calculate the ratio between desired and actual
        #       and apply to the other values. Additionally increase the size of the timestep by the calculated ratio
        # - Limit the velocity:
        #       Calculate the ratio between the desired and actual velocity and scale the timestep by that ratio
        # - Limit the acceleration:
        #       Calculate the ratio between the desired and actual acceleration and scale the timestep by that ratio squared

        # For each timestep find the biggest values in the steering angle/velocity/acceleration/jerk
        for time_index, time_pair in enumerate(self.steering_profiles):
            max_steering_velocity = 0.0
            max_steering_velocity_index = -1

            for module_index in range(len(self.drive_modules)):
                steering_velocity = abs(time_pair.state[module_index].first_derivative)

                if steering_velocity > max_steering_velocity:
                    max_steering_velocity = steering_velocity
                    max_steering_velocity_index = module_index

            # Limit the steering velocity. Assume a linear change between the previous point and the current one.
            if max_steering_velocity > self.drive_modules[max_steering_velocity_index].steering_motor_maximum_velocity:
                reduction_ratio = self.drive_modules[max_steering_velocity_index].steering_motor_maximum_velocity / max_steering_velocity

                # Increase the timestep so that we end up in the same location
                time_pair.time_fraction = time_pair.time_fraction / reduction_ratio
                self.velocity_profiles[time_index].time_fraction = self.velocity_profiles[time_index].time_fraction / reduction_ratio

                # Reduce all the velocities, accelerations and jerks
                for module_index in range(len(self.drive_modules)):
                    time_pair.state[module_index].first_derivative = time_pair.state[module_index].first_derivative * reduction_ratio

        # limit the steering acceleration
        # For each timestep find the biggest values in the acceleration
        self.calculate_velocities()
        self.calculate_accelerations()
        for time_index, time_pair in enumerate(self.steering_profiles):
            max_steering_acceleration = 0.0
            max_steering_acceleration_index = -1

            for module_index in range(len(self.drive_modules)):
                steering_acceleration = abs(time_pair.state[module_index].second_derivative)

                if steering_acceleration > max_steering_acceleration:
                    max_steering_acceleration = steering_acceleration
                    max_steering_acceleration_index = module_index

            # Limit the steering acceleration. Assume a linear change between the previous point and the current one.
            if max_steering_acceleration > self.drive_modules[max_steering_acceleration_index].steering_motor_maximum_acceleration:
                # The reduction ratio changes the time step, which also changes the velocity so the effect of the reduction ratio is
                # not linear but quadratic. The equation we're trying to solve is:
                #    a_max = (v_curr * ratio - v_prev) / (time_step / ratio) = (v_curr * ratio - v_prev) * ratio / time_step
                # so:
                #    a_max * time_step = (v_curr * ratio - v_prev) * ratio = v_curr * ratio^2 - v_prev * ratio
                # So the equation to solve is
                #    v_curr * ratio^2 - v_prev * ratio - a_max * time_step = 0
                #
                # Make sure that we use the correct maximum acceleration
                max_accel = self.drive_modules[max_steering_acceleration_index].steering_motor_maximum_acceleration * abs(time_pair.state[max_steering_acceleration_index].second_derivative) / time_pair.state[max_steering_acceleration_index].second_derivative
                discriminant = pow(-self.steering_profiles[time_index - 1].state[max_steering_acceleration_index].first_derivative, 2.0) + 4.0 * time_pair.state[max_steering_acceleration_index].first_derivative * max_accel * time_pair.time_fraction

                solution_1 = (self.steering_profiles[time_index - 1].state[max_steering_acceleration_index].first_derivative + math.sqrt(discriminant)) / (2.0 * time_pair.state[max_steering_acceleration_index].first_derivative)
                solution_2 = (self.steering_profiles[time_index - 1].state[max_steering_acceleration_index].first_derivative - math.sqrt(discriminant)) / (2.0 * time_pair.state[max_steering_acceleration_index].first_derivative)

                reduction_ratio = solution_1 if solution_1 > solution_2 else solution_2

                # Increase the timestep so that we end up in the same location
                time_pair.time_fraction = time_pair.time_fraction / reduction_ratio
                self.velocity_profiles[time_index].time_fraction = self.velocity_profiles[time_index].time_fraction / reduction_ratio

                # Reduce all the velocities, accelerations and jerks
                for module_index in range(len(self.drive_modules)):
                    # recalculate the velocity
                    time_pair.state[module_index].first_derivative = time_pair.state[module_index].first_derivative * reduction_ratio

                    # Recalculate the acceleration
                    previous_velocity = self.steering_profiles[time_index - 1].state[module_index].first_derivative
                    time_pair.state[module_index].second_derivative = (time_pair.state[module_index].first_derivative - previous_velocity) / time_pair.time_fraction

                    # recalculate the next acceleration
                    if time_index < len(self.steering_profiles) - 1:
                        next_velocity = self.steering_profiles[time_index + 1].state[module_index].first_derivative
                        next_time_fraction = self.steering_profiles[time_index + 1].time_fraction
                        self.steering_profiles[time_index + 1].state[module_index].second_derivative = (next_velocity - time_pair.state[module_index].first_derivative) / next_time_fraction

        # limit the drive velocity
        # For each timestep find the biggest values in the drive velocity/acceleration/jerk
        self.calculate_velocities()
        self.calculate_accelerations()
        for time_index, time_pair in enumerate(self.velocity_profiles):
            max_drive_velocity = 0.0
            max_drive_velocity_index = -1

            for module_index in range(len(self.drive_modules)):
                drive_velocity = abs(time_pair.state[module_index].value)

                if drive_velocity > max_drive_velocity:
                    max_drive_velocity = drive_velocity
                    max_drive_velocity_index = module_index

            # Limit the drive velocity. Assume a linear change between the previous point and the current one.
            if max_drive_velocity > self.drive_modules[max_drive_velocity_index].drive_motor_maximum_velocity:
                reduction_ratio = self.drive_modules[max_drive_velocity_index].drive_motor_maximum_velocity / max_drive_velocity

                # Reduce all the velocities
                for module_index in range(len(self.drive_modules)):
                    time_pair.state[module_index].value = time_pair.state[module_index].value * reduction_ratio

                # Increase the timestep so that we end up in the same location
                time_pair.time_fraction = time_pair.time_fraction / reduction_ratio
                self.steering_profiles[time_index].time_fraction = self.steering_profiles[time_index].time_fraction / reduction_ratio

                # TODO: Adjust steering velocity etc. etc.

class BodyControlledDriveModuleProfile(ModuleStateProfile):
    def __init__(
            self,
            drive_modules: List[DriveModule],
            control_model: ControlModelBase,
            min_body_to_module_resolution_per_second: float,
            motion_profile_func: Callable[[float, float, float, RealNumberValueSpace], TransientVariableProfile]):
        self.modules = drive_modules
        self.control_model = control_model
        self.min_trajectory_time_in_seconds = 1.0 # Set a default. This will be changed when we calculate the profiles
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

        start_steering_orientation: List[float] = []
        start_drive_velocity: List[float] = []
        for module_index in range(len(self.modules)):
            start = self.start_state_modules[module_index]
            start_steering_orientation.append(self.steering_number_space.normalize_value(start.orientation_in_body_coordinates.z))
            start_drive_velocity.append(start.drive_velocity_in_module_coordinates.x)

        calculated_profiles: LimitedDriveModuleProfile = LimitedDriveModuleProfile(self.modules)
        calculated_profiles.add_profile_point(0.0, start_steering_orientation, start_drive_velocity)

        # Compute the body profile
        start_state_body = self.control_model.body_motion_from_wheel_module_states(self.start_state_modules)
        body_profiles = [
            self.motion_profile_func(start_state_body.linear_velocity.x, self.end_state_body.linear_velocity.x, self.min_trajectory_time_in_seconds, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.linear_velocity.y, self.end_state_body.linear_velocity.y, self.min_trajectory_time_in_seconds, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.linear_velocity.z, self.end_state_body.linear_velocity.z, self.min_trajectory_time_in_seconds, self.unbounded_number_space),

            self.motion_profile_func(start_state_body.angular_velocity.x, self.end_state_body.angular_velocity.x, self.min_trajectory_time_in_seconds, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.angular_velocity.y, self.end_state_body.angular_velocity.y, self.min_trajectory_time_in_seconds, self.unbounded_number_space),
            self.motion_profile_func(start_state_body.angular_velocity.z, self.end_state_body.angular_velocity.z, self.min_trajectory_time_in_seconds, self.unbounded_number_space),
        ]

        # Compute intermediate steps for the modules. Assume that the whole profile is 1 second long
        # later on we will change the profile time to ensure that we are within the bounds of the
        # motor capabilities
        number_of_frames = math.ceil(1.0 * self.min_body_to_module_resolution_per_second)
        time_step_per_frame = 1.0 / float(number_of_frames)

        # We don't include the start that is defined by the actual current state. We also don't
        # add the end
        previous_steering_angles: List[float] = start_steering_orientation
        previous_drive_velocities: List[float] = start_drive_velocity

        # Iterate over all the internal frames and 1 extra to include the end state
        for frame_index in range(1, number_of_frames + 1):
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
            current_steering_orientation, current_drive_velocity = select_directions_for_modules(
                self.modules,
                self.steering_number_space,
                previous_steering_angles,
                previous_drive_velocities,
                drive_module_states)

            calculated_profiles.add_profile_point(time_step_per_frame, current_steering_orientation, current_drive_velocity)
            previous_steering_angles = current_steering_orientation
            previous_drive_velocities = current_drive_velocity

        # apply limits for steering velocity, wheel velocity and accelerations
        #
        # Limits based on: https://journals.sagepub.com/doi/10.5772/51153
        calculated_profiles.limit_profiles()

        profile_total_time = sum([x.time_fraction for x in calculated_profiles.steering_profiles])

        profiles: Mapping[str, List[SingleVariableMultiPointLinearProfile]] = {}
        for module_index in range(len(self.modules)):
            profiles[self.modules[module_index].name] = [
                # Steering orientation
                SingleVariableMultiPointLinearProfile(
                    calculated_profiles.steering_profiles[0].state[module_index].value,
                    calculated_profiles.steering_profiles[-1].state[module_index].value,
                    end_time=profile_total_time,
                    coordinate_space=PeriodicBoundedCircularSpace()),

                # Drive velocity
                SingleVariableMultiPointLinearProfile(
                    calculated_profiles.velocity_profiles[0].state[module_index].value,
                    calculated_profiles.velocity_profiles[-1].state[module_index].value,
                    end_time=profile_total_time)
            ]

        time_to_now = 0.0
        for i in range(1, len(calculated_profiles.steering_profiles)):
            time_to_now += calculated_profiles.steering_profiles[i].time_fraction
            module_steering_values = calculated_profiles.steering_profiles[i].state
            module_drive_values = calculated_profiles.velocity_profiles[i].state

            for module_index in range(len(self.modules)):
                profiles[self.modules[module_index].name][0].add_value(time_to_now, module_steering_values[module_index].value)
                profiles[self.modules[module_index].name][1].add_value(time_to_now, module_drive_values[module_index].value)

        self.module_profiles = profiles
        self.min_trajectory_time_in_seconds = profile_total_time

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

    def value_for_module_at(self, id: str, time_since_start_of_profile: float) -> DriveModuleMeasuredValues:
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
            profiles[0].value_at(time_since_start_of_profile),
            profiles[0].first_derivative_at(time_since_start_of_profile),
            profiles[0].second_derivative_at(time_since_start_of_profile),
            profiles[0].third_derivative_at(time_since_start_of_profile),
            profiles[1].value_at(time_since_start_of_profile),
            profiles[1].first_derivative_at(time_since_start_of_profile),
            profiles[1].second_derivative_at(time_since_start_of_profile),
        )
