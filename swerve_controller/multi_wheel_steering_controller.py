#!/usr/bin/python3

from abc import ABC, abstractmethod
import math
from turtle import forward

import numpy as np
from typing import Mapping, List, Tuple

# local
from .control import BodyMotionCommand, DriveModuleMotionCommand, InvalidMotionCommandException, MotionCommand
from .control_model import ControlModelBase, SimpleFourWheelSteeringControlModel
from .drive_module import DriveModule
from .geometry import Point
from .states import BodyState, DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion
from .trajectory import BodyControlledDriveModuleTrajectory, LinearDriveModuleStateTrajectory, ModuleStateTrajectory

class MultiWheelSteeringController(ABC):

    # Returns the current pose of the robot body, based on the current state of the
    # drive modules.
    @abstractmethod
    def body_state_at_current_time(self) -> BodyState:
        pass

    # Returns the states of the drive modules, as measured at the current time.
    @abstractmethod
    def drive_module_state_at_current_time(self) -> List[DriveModuleMeasuredValues]:
        pass

    # Returns the state of the drive modules to required to match the current trajectory at the given
    # time.
    @abstractmethod
    def drive_module_state_at_future_time(self, future_time_in_seconds:float) -> List[DriveModuleMeasuredValues]:
        pass

    # Gets the control model that is used to determine the state of the body and the drive modules.
    @abstractmethod
    def get_control_model(self) -> ControlModelBase:
        pass

    # Updates the currently stored drive module state
    @abstractmethod
    def on_state_update(self, current_module_states: List[DriveModuleMeasuredValues]):
        pass

    # Updates the currently stored desired body state. On the next time tick the
    # drive module trajectory will be updated to match the new desired end state.
    @abstractmethod
    def on_desired_state_update(self, desired_motion: MotionCommand):
        pass

    # On clock tick, determine if we need to recalculate the trajectories for the drive modules
    @abstractmethod
    def on_tick(self, current_time_in_seconds: float):
        pass

class LinearModuleFirstSteeringController(MultiWheelSteeringController):

    def __init__(self, drive_modules: List[DriveModule]):
        # Get the geometry for the robot
        self.modules = drive_modules

        # Store the current (estimated) state of the body
        self.body_state: BodyState = BodyState(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

        # Store the current (measured) state of the drive modules
        self.module_states: List[DriveModuleMeasuredValues] = [
            DriveModuleMeasuredValues(
                drive_module.name,
                drive_module.steering_axis_xy_position.x,
                drive_module.steering_axis_xy_position.y,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ) for drive_module in drive_modules
        ]

        # Use a simple control model for the time being. Just need something that roughly works
        self.control_model = SimpleFourWheelSteeringControlModel(self.modules)

        # Store the desired body motion, and the last point in time where this value
        # was updated
        self.desired_motion: List[DriveModuleDesiredValues] = []
        self.motion_command_changed_at_time_in_seconds = 0.0

        # Track the current trajectories and update them if necessary
        self.drive_module_trajectory: LinearDriveModuleStateTrajectory = None

        # Track the time at which the trajectories were created
        self.trajectory_created_at_time_in_seconds = 0.0

        # Store the time pointer for where we are on the trajectory
        self.trajectory_current_time_in_seconds = 0.0

        # Keep track of our position in time so that we can figure out where on the current
        # trajectory we should be
        self.current_time_in_seconds = 0.0
        self.trajectory_was_started_at_time_in_seconds = 0.0

        self.min_time_for_trajectory = 1.0

    # Returns the current pose of the robot body, based on the current state of the
    # drive modules.
    def body_state_at_current_time(self) -> BodyState:
        return self.body_state

    # Returns the states of the drive modules, as measured at the current time.
    def drive_module_state_at_current_time(self) -> List[DriveModuleMeasuredValues]:
        return self.module_states

    # Returns the state of the drive modules to required to match the current trajectory at the given
    # time.
    def drive_module_state_at_future_time(self, future_time_in_seconds:float) -> List[DriveModuleMeasuredValues]:
        time_from_start_of_trajectory = future_time_in_seconds - self.trajectory_was_started_at_time_in_seconds
        time_fraction = time_from_start_of_trajectory / self.drive_module_trajectory.time_span()

        result: List[DriveModuleMeasuredValues] = []
        for drive_module in self.modules:
            state = self.drive_module_trajectory.value_for_module_at(drive_module.name, time_fraction)  #### WRONG TYPE
            result.append(state)

        return result

    # Gets the control model that is used to determine the state of the body and the drive modules.
    def get_control_model(self) -> ControlModelBase:
        return self.control_model

    # Updates the currently stored drive module state
    def on_state_update(self, current_module_states: List[DriveModuleMeasuredValues]):
        if current_module_states is None:
            raise TypeError()

        if len(current_module_states) != len(self.modules):
            raise ValueError()

        self.module_states.clear()
        self.module_states.extend(current_module_states)

    # Updates the currently stored desired body state. On the next time tick the
    # drive module trajectory will be updated to match the new desired end state.
    def on_desired_state_update(self, desired_motion: MotionCommand):
        desired_potential_states = desired_motion.to_drive_module_state(self.control_model)

        # Select which state to use, either the forward one or the reverse one.
        # - If there are two directions we need to pick, otherwise pick the only one we have
        # - If the wheels aren't moving then we can pick the one with the closer steering angle change
        # - If the wheels are moving then we use

        desired_states = desired_potential_states[0]
        if len(desired_potential_states[1]) > 0:
            is_stopped = [math.isclose(state.drive_velocity_in_module_coordinates.x, 0.0, rel_tol=1e-7, abs_tol=1e-7) for state in self.module_states]
            if all(is_stopped):
                # wheels aren't moving. Can do any move we like. Limit steering movemement.
                total_first_rotation = 0.0
                total_second_rotation = 0.0
                for i in range(len(self.modules)):
                    current = self.module_states[i].orientation_in_body_coordinates.z
                    # Normalize the steering angle to be between 0 and 2pi
                    if current >= 2 * math.pi:
                        current -= 2 * math.pi

                    if current < 0:
                        current += 2 * math.pi

                    total_first_rotation += abs(desired_potential_states[0][i].steering_angle_in_radians - current)
                    total_second_rotation += abs(desired_potential_states[1][i].steering_angle_in_radians - current)

                if total_second_rotation < total_first_rotation:
                    desired_states = desired_potential_states[1]
            else:
                # Wheels are moving. Pick the first state for now. Not sure how to pick the correct one
                pass

        self.min_time_for_trajectory = desired_motion.time_for_motion()
        self.desired_motion = desired_states
        self.motion_command_changed_at_time_in_seconds = self.current_time_in_seconds

    # On clock tick, determine if we need to recalculate the trajectories for the drive modules
    def on_tick(self, current_time_in_seconds: float):
        self.current_time_in_seconds = current_time_in_seconds

        # Calculate the current body state
        body_motion = self.control_model.body_motion_from_wheel_module_states(self.module_states)
        self.body_state = BodyState(
            self.body_state.position_in_world_coordinates.x,
            self.body_state.position_in_world_coordinates.y,
            self.body_state.orientation_in_world_coordinates.z,
            body_motion.linear_velocity.x,
            body_motion.linear_velocity.y,
            body_motion.angular_velocity.z,
        )

        # If the desired body motion was updated after the trajectory was created, then we need to
        # update the trajectory.
        if (self.motion_command_changed_at_time_in_seconds <= self.trajectory_was_started_at_time_in_seconds) and self.drive_module_trajectory is not None:
            return

        # use the twist trajectory to compute the state for the steering modules for the end state
        # and several intermediate points, i.e. determine the vector [[v_i];[gamma_i]].
        #    Use Seegmiller and Kelly to compute the desired velocities and angles
        #
        #    Keep in mind that our update rate determines the points in time where we can do something
        #
        #    Also keep in mind that steering the wheel effectively changes the velocity of the wheel
        #    if we use a co-axial system
        drive_module_trajectory = LinearDriveModuleStateTrajectory(self.modules, self.min_time_for_trajectory)
        drive_module_trajectory.set_current_state(self.module_states)
        drive_module_trajectory.set_desired_end_state(self.desired_motion)

        # Correct for motor capabilities (max velocity, max acceleration, deadband etc.)
        # and make sure that the trajectories of the different modules all span the same time frame
        drive_module_trajectory.align_module_profiles()

        self.drive_module_trajectory = drive_module_trajectory
        self.trajectory_was_started_at_time_in_seconds = self.current_time_in_seconds

# Linear body first, or even any kind of body first might not be possible because not all module motions
# result in a body motion, but all body motions result in a module motion. That means that for body
# first there might be multiple module movements, e.g. when moving in y-direction the modules
# either need to turn to the y-position during the movement or before the movent.
class LinearBodyFirstSteeringController(MultiWheelSteeringController):

    def __init__(self, drive_modules: List[DriveModule]):
        # Get the geometry for the robot
        self.modules = drive_modules

        # Store the current (estimated) state of the body
        self.body_state: BodyState = BodyState(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

        # Store the current (measured) state of the drive modules
        self.module_states: List[DriveModuleMeasuredValues] = [
            DriveModuleMeasuredValues(
                drive_module.name,
                drive_module.steering_axis_xy_position.x,
                drive_module.steering_axis_xy_position.y,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ) for drive_module in drive_modules
        ]

        # Use a simple control model for the time being. Just need something that roughly works
        self.control_model = SimpleFourWheelSteeringControlModel(self.modules)

        # Store the desired body motion, and the last point in time where this value
        # was updated
        self.desired_body_motion: BodyMotion = BodyMotion(0.0, 0.0, 0.0)

        # All body motions can be translated into drive module motions, however the reverse isn't
        # true, e.g. in the case of a rotation of all the modules without changing the body
        # orientation or velocity.
        # So we deal with that by tracking module motions.
        self.desired_module_motion: List[DriveModuleDesiredValues] = []
        self.desired_motion_changed_at_time_in_seconds = 0.0

        # Track the current trajectories and update them if necessary
        self.module_trajectory: ModuleStateTrajectory = None

        # Track the time at which the trajectories were created
        self.trajectory_created_at_time_in_seconds = 0.0

        # Store the time pointer for where we are on the trajectory
        self.trajectory_current_time_in_seconds = 0.0

        # Keep track of our position in time so that we can figure out where on the current
        # trajectory we should be
        self.current_time_in_seconds = 0.0
        self.trajectory_was_started_at_time_in_seconds = 0.0

        self.min_time_for_trajectory = 1.0

    # Returns the current pose of the robot body, based on the current state of the
    # drive modules.
    def body_state_at_current_time(self) -> BodyState:
        return self.body_state

    # Returns the states of the drive modules, as measured at the current time.
    def drive_module_state_at_current_time(self) -> List[DriveModuleMeasuredValues]:
        return self.module_states

    # Returns the state of the drive modules to required to match the current trajectory at the given
    # time.
    def drive_module_state_at_future_time(self, future_time_in_seconds:float) -> List[DriveModuleMeasuredValues]:
        time_from_start_of_trajectory = future_time_in_seconds - self.trajectory_was_started_at_time_in_seconds

        result: List[DriveModuleMeasuredValues] = []
        time_fraction = time_from_start_of_trajectory / self.module_trajectory.time_span()
        for drive_module in self.modules:
            state = self.module_trajectory.value_for_module_at(drive_module.name, time_fraction)
            result.append(state)

        return result

    # Gets the control model that is used to determine the state of the body and the drive modules.
    def get_control_model(self) -> ControlModelBase:
        return self.control_model

    # Updates the currently stored drive module state
    def on_state_update(self, current_module_states: List[DriveModuleMeasuredValues]):
        if current_module_states is None:
            raise TypeError()

        if len(current_module_states) != len(self.modules):
            raise ValueError()

        self.module_states.clear()
        self.module_states.extend(current_module_states)

    # Updates the currently stored desired body state. On the next time tick the
    # drive module trajectory will be updated to match the new desired end state.
    def on_desired_state_update(self, desired_motion: MotionCommand):
        if isinstance(desired_motion, BodyMotionCommand):
            self.desired_body_motion = desired_motion.to_body_state(self.control_model)
            self.desired_module_motion = []
        else:
            if isinstance(desired_motion, DriveModuleMotionCommand):
                self.desired_module_motion = desired_motion.to_drive_module_state(self.control_model)[0]
                self.desired_body_motion = None
            else:
                raise InvalidMotionCommandException()

        self.desired_motion_changed_at_time_in_seconds = self.current_time_in_seconds
        self.min_time_for_trajectory = desired_motion.time_for_motion()

    # On clock tick, determine if we need to recalculate the trajectories for the drive modules
    def on_tick(self, current_time_in_seconds: float):
        self.current_time_in_seconds = current_time_in_seconds

        # Calculate the current body state
        self.body_state = self.control_model.body_motion_from_wheel_module_states(self.module_states)

        # If the desired body motion was updated after the trajectory was created, then we need to
        # update the trajectory.
        if (self.desired_motion_changed_at_time_in_seconds <= self.trajectory_was_started_at_time_in_seconds) and self.module_trajectory is not None:
            return

        if self.desired_body_motion is not None:
            trajectory = BodyControlledDriveModuleTrajectory(self.modules, self.control_model, self.min_time_for_trajectory, 50.0)
            trajectory.set_current_state(self.module_states)
            trajectory.set_desired_end_state(self.desired_body_motion)
            self.module_trajectory = trajectory
        else:
            if len(self.desired_module_motion) > 0:
                drive_module_trajectory = LinearDriveModuleStateTrajectory(self.modules, self.min_time_for_trajectory)
                drive_module_trajectory.set_current_state(self.module_states)
                drive_module_trajectory.set_desired_end_state(self.desired_module_motion)

                # Correct for motor capabilities (max velocity, max acceleration, deadband etc.)
                # and make sure that the trajectories of the different modules all span the same time frame
                drive_module_trajectory.align_module_profiles()

                self.module_trajectory = drive_module_trajectory
            else:
                # No desired body or module motion is
                return

        self.trajectory_was_started_at_time_in_seconds = self.current_time_in_seconds

class SmoothSteeringController(MultiWheelSteeringController):
    pass
