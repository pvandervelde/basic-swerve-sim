#!/usr/bin/python3

from math import acos, cos, degrees, isclose, pow, radians, sin, sqrt, tan
from turtle import forward

import numpy as np
from typing import Mapping, List, Tuple

# local
from .control_model import BodyState, DriveModuleState, Motion, SimpleFourWheelSteeringControlModel
from .drive_module import DriveModule
from .trajectory import BodyMotionTrajectory, DriveModuleStateTrajectory

class MultiWheelSteeringController(object):

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
        self.module_states: List[DriveModuleState] = [
            DriveModuleState(
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
        self.desired_body_motion: Motion = Motion(0.0, 0.0, 0.0)
        self.body_motion_changed_at_time_in_seconds = 0.0

        # Track the current trajectories and update them if necessary
        self.drive_module_trajectory: DriveModuleStateTrajectory = None

        # Track the time at which the trajectories were created
        self.trajectory_created_at_time_in_seconds = 0.0

        # Store the time pointer for where we are on the trajectory
        self.trajectory_current_time_in_seconds = 0.0

        # Keep track of our position in time so that we can figure out where on the current
        # trajectory we should be
        self.current_time_in_seconds = 0.0
        self.trajectory_was_started_at_time_in_seconds = 0.0

    # Returns the current pose of the robot body, based on the current state of the
    # drive modules.
    def current_pose(self) -> BodyState:
        return self.body_state

    # Returns the states of the drive modules, as measured at the current time.
    def drive_module_state_at_current_time(self) -> List[DriveModuleState]:
        return self.module_states

    # Returns the state of the drive modules to required to match the current trajectory at the given
    # time.
    def drive_module_state_at_future_time(self, future_time_in_seconds:float) -> List[DriveModuleState]:
        time_from_start_of_trajectory = future_time_in_seconds - self.trajectory_was_started_at_time_in_seconds
        time_fraction = time_from_start_of_trajectory / self.drive_module_trajectory.time_span

        result: List[DriveModuleState] = []
        for drive_module in self.modules:
            state = self.drive_module_trajectory.value_for_module_at(drive_module.name, time_fraction)
            result.append(state)

        return result

    # Updates the currently stored drive module state
    def on_state_update(self, current_module_states: List[DriveModuleState]):
        if current_module_states is None:
            raise TypeError()

        if len(current_module_states) != len(self.modules):
            raise ValueError()

        self.module_states.clear()
        self.module_states.extend(current_module_states)

    # Updates the currently stored desired body state. On the next time tick the
    # drive module trajectory will be updated to match the new desired end state.
    def on_desired_state_update(self, desired_body_motion: Motion):
        self.desired_body_motion = desired_body_motion
        self.body_motion_changed_at_time_in_seconds = self.current_time_in_seconds

    # On clock tick, determine if we need to recalculate the trajectories for the drive modules
    def on_tick(self, delta_time_in_seconds: float):
        self.current_time_in_seconds += delta_time_in_seconds

        # Calculate the current body state
        self.body_state = self.control_model.body_motion_from_wheel_module_states(self.module_states)

        # If the desired body motion was updated after the trajectory was created, then we need to
        # update the trajectory.
        if (self.body_motion_changed_at_time_in_seconds <= self.trajectory_was_started_at_time_in_seconds):
            return

        # use the twist trajectory to compute the state for the steering modules for the end state
        # and several intermediate points, i.e. determine the vector [[v_i];[gamma_i]].
        #    Use Seegmiller and Kelly to compute the desired velocities and angles
        #
        #    Keep in mind that our update rate determines the points in time where we can do something
        #
        #    Also keep in mind that steering the wheel effectively changes the velocity of the wheel
        #    if we use a co-axial system
        drive_module_trajectory = DriveModuleStateTrajectory(self.modules)
        drive_module_trajectory.set_current_state(self.current_module_states)

        # We may need a pre-trajectory section that aligns the wheel modules to the movement we want
        # to make. This would only be the case if we're currently not moving. If we're moving all
        # modules should be aligned.
        #
        # If we're moving and the modules are not aligned (given a certain tolerance) then we need to
        # rectify that first!
        #
        # --> This almost feels like we need some kind of decision tree structure?
        #
        #

        # We get both the forward and reverse options here. We should see which is the better one
        # For now just use the forward one
        drive_module_potential_states = self.control_model.state_of_wheel_modules_from_body_motion(self.module_states, self.desired_body_motion)
        drive_module_desired_states = [x[0] for x in drive_module_potential_states]

        drive_module_trajectory.set_desired_end_state(drive_module_desired_states)

        # Correct for motor capabilities (max velocity, max acceleration, deadband etc.)
        # and make sure that the trajectories of the different modules all span the same time frame
        drive_module_trajectory.align_module_profiles()

        self.drive_module_trajectory = drive_module_trajectory
        self.trajectory_was_started_at_time_in_seconds = self.current_time_in_seconds
