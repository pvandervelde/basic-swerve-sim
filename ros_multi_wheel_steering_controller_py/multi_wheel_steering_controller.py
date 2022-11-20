#!/usr/bin/python3

from math import acos, cos, degrees, isclose, pow, radians, sin, sqrt, tan
from turtle import forward

import numpy as np
from typing import Mapping, List, Tuple

# local
from control_model import BodyState, DriveModuleState, body_state_from_twist, ControlModelBase, SimpleFourWheelSteeringControlModel
from drive_module import DriveModule
from trajectory import BodyStateTrajectory, DriveModuleStateTrajectory

class MultiWheelSteeringController(object):

    def __init__(self, drive_modules: List[DriveModule]):
        super().__init__('multi_wheel_steering_controller')

        # Get the geometry for the robot
        self.modules = drive_modules

        # Use a simple control model for the time being. Just need something that roughly works
        self.control_model = SimpleFourWheelSteeringControlModel(self.modules)

    def body_trajectory_from_drive_module_states(
        self,
        current_module_states: List[DriveModuleState]) -> BodyStateTrajectory:
        pass

    def drive_module_trajectory_to_achieve_desired_body_state(
        self,
        desired_body_state: BodyState,
        current_module_states: List[DriveModuleState]) -> DriveModuleStateTrajectory:

        # Compute the twist trajectory, i.e. how do we get from our current (v, omega) to the
        # desired (v, omega)
        #   Ideally we want O(1) for jerk, O(2) for acceleration, O(3) for velocity
        #   Can use a spline or b-spline or other trajectory approach
        current_body_state = self.control_model.state_of_body_frame_from_wheel_module_states(current_module_states)

        # Note: We recalculate the trajectory at this stage so that we use the current snapshot of what is
        # going on. If we were to have a trajectory that is re-used we will have to safe guard the updates
        # because the updates would come in on different threads.
        body_trajectory = BodyStateTrajectory()
        body_trajectory.update_current_state(current_body_state)
        body_trajectory.update_desired_state(desired_body_state)

        # use the twist trajectory to compute the state for the steering modules for the end state
        # and several intermediate points, i.e. determine the vector [[v_i];[gamma_i]].
        #    Use Seegmiller and Kelly to compute the desired velocities and angles
        #
        #    Keep in mind that our update rate determines the points in time where we can do something
        #
        #    Also keep in mind that steering the wheel effectively changes the velocity of the wheel
        #    if we use a co-axial system
        drive_module_trajectory = DriveModuleStateTrajectory(self.modules)
        drive_module_trajectory.set_current_state(current_module_states)

        drive_module_trajectory.set_desired_end_state(
            self.control_model.state_of_wheel_modules_from_body_state(body_trajectory.value_at(1.0))
        )



        SET_INTERMEDIATE_POINTS

        # Correct for motor capabilities (max velocity, max acceleration, deadband etc.)
        # and make sure that the trajectories of the different modules all span the same time frame
        drive_module_trajectory.align_module_profiles()

        return drive_module_trajectory

    def drive_module_trajectory_to_achieve_desired_drive_module_orientation(
        self,
        desired_module_states: List[DriveModuleState],
        current_module_states: List[DriveModuleState]) -> DriveModuleStateTrajectory:

        # If the modules are currently stopped, i.e. no forward speed on the wheels, then
        # we can just turn them all at the most efficient rate

        # In this state the ICR for different wheel sets may not be aligned
        pass
