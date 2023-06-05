#!/usr/bin/python3

from abc import ABC, abstractmethod
import math
from turtle import forward

import numpy as np
from typing import Callable, Mapping, List, Tuple

# local
from .control import BodyMotionCommand, DriveModuleMotionCommand, InvalidMotionCommandException, MotionCommand
from .control_model import ControlModelBase, SimpleFourWheelSteeringControlModel
from .drive_module import DriveModule
from .geometry import Point
from .states import BodyState, DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion
from .trajectory import BodyControlledDriveModuleTrajectory, LinearBodyMotionTrajectory, LinearDriveModuleStateTrajectory, ModuleStateTrajectory

class MultiWheelSteeringController(object):

    def __init__(self, drive_modules: List[DriveModule]):
        # Get the geometry for the robot
        self.modules = drive_modules

        # Use a simple control model for the time being. Just need something that roughly works
        self.control_model = SimpleFourWheelSteeringControlModel(self.modules)

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

        self.previous_module_states: List[DriveModuleMeasuredValues] = [
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

        # trajectories
        self.body_trajectory: LinearBodyMotionTrajectory = None
        self.module_trajectory_from_command: LinearDriveModuleStateTrajectory = None

         # Keep track of our position in time so that we can figure out where on the current
        # trajectory we should be
        self.current_time_in_seconds = 0.0
        self.trajectory_was_started_at_time_in_seconds = 0.0
        self.last_state_update_time = 0.0
        self.min_time_for_trajectory: float = 0.0

        # flags
        self.is_executing_body_trajectory: bool = False
        self.is_executing_module_trajectory: bool = False

    def body_state_at_current_time(self) -> BodyState:
        return self.body_state

    def drive_module_states_at_current_time(self) -> List[DriveModuleMeasuredValues]:
        return self.module_states

    # Returns the state of the drive modules to required to match the current trajectory at the given
    # time.
    def drive_module_state_at_future_time(self, future_time_in_seconds:float) -> List[DriveModuleDesiredValues]:
        time_from_start_of_trajectory = future_time_in_seconds - self.trajectory_was_started_at_time_in_seconds

        trajectory_time = self.body_trajectory.time_span() if self.is_executing_body_trajectory else self.module_trajectory_from_command.time_span()
        time_fraction = time_from_start_of_trajectory / trajectory_time

        result: List[DriveModuleDesiredValues] = []
        if self.is_executing_body_trajectory:
            body_state = self.body_trajectory.body_motion_at(time_fraction)
            drive_module_desired_values = self.control_model.state_of_wheel_modules_from_body_motion(body_state)
            for i in range(len(self.modules)):
                # Wheels are moving. We don't know what kind of movement yet though, so figure out if:
                # - The wheel are moving at some significant velocity, in that case pick the state that most
                #   closely matches the current state, i.e. match the drive velocity and the steering angle as
                #   close as possible
                # - The wheel is moving slowly, in that case we may just be close to the moment where the wheel
                #   stops moving (either just before it does that, or just after). This is where we could potentially
                #   flip directions (or we might just have flipped directions)
                #   - If we have just flipped directions then we should probably continue in the same way (but maybe not)

                #previous_state_for_module = self.previous_module_states[i]

                current_state_for_module = self.module_states[i]
                current_steering_angle = current_state_for_module.orientation_in_body_coordinates.z
                current_velocity = current_state_for_module.drive_velocity_in_module_coordinates.x

                #previous_rotation_difference = current_steering_angle - previous_state_for_module.orientation_in_body_coordinates.z
                #previous_velocity_difference = current_velocity - previous_state_for_module.drive_velocity_in_module_coordinates.x

                # Normalize the steering angle to be between 0 and 2pi
                # if current_steering_angle >= 2 * math.pi:
                #     current_steering_angle -= 2 * math.pi

                # if current_steering_angle < 0:
                #     current_steering_angle += 2 * math.pi

                states_for_module = drive_module_desired_values[i]

                first_state_rotation_difference = states_for_module[0].steering_angle_in_radians - current_steering_angle
                second_state_rotation_difference = states_for_module[1].steering_angle_in_radians - current_steering_angle

                first_state_velocity_difference = states_for_module[0].drive_velocity_in_meters_per_second - current_velocity
                second_state_velocity_difference = states_for_module[1].drive_velocity_in_meters_per_second - current_velocity

                # Possibilities:
                # - first velocity change and first orientation change are the smallest -> pick the first state
                # - second velocity change and second orientation change are the smallest -> pick the second state
                # - first velocity change is larger and second orientation change is larger -> Bad state. Pick the one with the least relative change?

                if abs(first_state_rotation_difference) <= abs(second_state_rotation_difference):
                    if abs(first_state_velocity_difference) <= abs(second_state_velocity_difference):
                        # first rotation and velocity change are the smallest, so take the first state
                        result.append(states_for_module[0])
                    else:
                        if math.isclose(abs(first_state_rotation_difference), abs(second_state_rotation_difference), rel_tol=1e-7, abs_tol=1e-7):
                            # first rotation is equal to the second rotation
                            # first velocity larger than the second velocity.
                            # pick the second state
                            result.append(states_for_module[1])
                        else:
                            # first rotation is the smallest but second velocity is the smallest
                            result.append(states_for_module[0])
                else:
                    if abs(second_state_velocity_difference) <= abs(first_state_velocity_difference):
                        # second rotation and velocity change are the smallest, so take the second state
                        result.append(states_for_module[1])
                    else:
                        if math.isclose(abs(first_state_rotation_difference), abs(second_state_rotation_difference), rel_tol=1e-7, abs_tol=1e-7):
                            # second rotation is equal to the first rotation
                            # second velocity larger than the first velocity.
                            # pick the first state
                            result.append(states_for_module[0])
                        else:
                            # second rotation is the smallest but first velocity is the smallest
                            result.append(states_for_module[1])
        else:
            for drive_module in self.modules:
                state = self.module_trajectory_from_command.value_for_module_at(drive_module.name, time_fraction)
                result.append(DriveModuleDesiredValues(
                    state.name,
                    state.orientation_in_body_coordinates.z,
                    state.drive_velocity_in_module_coordinates.x
                ))

        return result

    # Updates the currently stored desired body state. On the next time tick the
    # drive module trajectory will be updated to match the new desired end state.
    def on_desired_state_update(self, desired_motion: MotionCommand):
        if isinstance(desired_motion, BodyMotionCommand):
            trajectory = LinearBodyMotionTrajectory(self.body_state, desired_motion.to_body_state(self.control_model), desired_motion.time_for_motion())
            self.body_trajectory = trajectory

            self.is_executing_body_trajectory = True
            self.is_executing_module_trajectory = False
        else:
            if isinstance(desired_motion, DriveModuleMotionCommand):
                trajectory = LinearDriveModuleStateTrajectory(self.modules, desired_motion.time_for_motion())
                trajectory.set_current_state(self.module_states)
                trajectory.set_desired_end_state(desired_motion.to_drive_module_state(self.control_model)[0])
                self.module_trajectory_from_command = trajectory

                self.is_executing_body_trajectory = False
                self.is_executing_module_trajectory = True
            else:
                raise InvalidMotionCommandException()

        self.trajectory_was_started_at_time_in_seconds = self.current_time_in_seconds
        self.min_time_for_trajectory = desired_motion.time_for_motion()

    # Updates the currently stored drive module state
    def on_state_update(self, current_module_states: List[DriveModuleMeasuredValues]):
        if current_module_states is None:
            raise TypeError()

        if len(current_module_states) != len(self.modules):
            raise ValueError()

        self.previous_module_states = self.module_states
        self.module_states = current_module_states

         # Calculate the current body state
        body_motion = self.control_model.body_motion_from_wheel_module_states(self.module_states)

        time_step_in_seconds = self.current_time_in_seconds - self.last_state_update_time
        local_x_distance = time_step_in_seconds * 0.5 * (self.body_state.motion_in_body_coordinates.linear_velocity.x + body_motion.linear_velocity.x)
        local_y_distance = time_step_in_seconds * 0.5 * (self.body_state.motion_in_body_coordinates.linear_velocity.y + body_motion.linear_velocity.y)
        global_orientation = self.body_state.orientation_in_world_coordinates.z + time_step_in_seconds * 0.5 * (self.body_state.motion_in_body_coordinates.angular_velocity.z + body_motion.angular_velocity.z)
        self.body_state = BodyState(
            self.body_state.position_in_world_coordinates.x + local_x_distance * math.cos(global_orientation) - local_y_distance * math.sin(global_orientation),
            self. body_state.position_in_world_coordinates.y + local_x_distance * math.sin(global_orientation) + local_y_distance * math.cos(global_orientation),
            global_orientation,
            body_motion.linear_velocity.x,
            body_motion.linear_velocity.y,
            body_motion.angular_velocity.z,
        )

        self.last_state_update_time = self.current_time_in_seconds

    # On clock tick, determine if we need to recalculate the trajectories for the drive modules
    def on_tick(self, current_time_in_seconds: float):
        self.current_time_in_seconds = current_time_in_seconds
