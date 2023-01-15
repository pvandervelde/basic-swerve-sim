#!/usr/bin/python3

from abc import ABC, abstractmethod
from math import acos, cos, degrees, isclose, pow, radians, sin, sqrt, tan
from turtle import forward

import numpy as np
from typing import Mapping, List, Tuple

# local
from .control import MotionCommand
from .control_model import BodyState, ControlModelBase, DriveModuleMeasuredValues, Motion, SimpleFourWheelSteeringControlModel
from .drive_module import DriveModule
from .geometry import Point
from .trajectory import BodyMotionTrajectory, DriveModuleStateTrajectory

# From here: https://stackoverflow.com/a/42727584
def get_intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> Point:
    """
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """

    # s for stacked
    s = np.vstack(
            [
                [a1.x, a1.y],
                [a2.x, a2.y],
                [b1.x, b1.y],
                [b2.x, b2.y],
            ]
        )

    # h for homogeneous
    h = np.hstack((s, np.ones((4, 1))))

    # get first line
    l1 = np.cross(h[0], h[1])

    # get second line
    l2 = np.cross(h[2], h[3])

    # point of intersection
    x, y, z = np.cross(l1, l2)

    # lines are parallel
    if isclose(z, 0.0, abs_tol=1e-15, rel_tol=1e-15):
        return Point(float('inf'), float('inf'), 0.0)

    return Point(x/z, y/z, 0.0)

def instantaneous_center_of_rotation_at_current_time(drive_module_states: List[DriveModuleMeasuredValues]) -> List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]:
    # Get axle lines for each drive module
    #
    # First point is the location of the drive module
    # Second point is a point on the axle line.
    #
    # The axle line is computed by
    # Getting the line perpendicular to the orientation vector.
    # The orientation vector is [cos(orientation.z), sin(orientation.z) ]^T
    # The perpendicular vector of vector [x, y]^T in 2d is [-y, x]^T
    # So the orientation perpendicular vector is [-sin(orientation.z), cos(orientation.z)]^T
    #
    # Thus a point on that line is [-sin(orientation.z) + position.x, cos(orientation.z) + position.y]^T
    axle_lines: List[Tuple[Point, Point]] = []
    for state in drive_module_states:
        line = (
            state.position_in_body_coordinates,
            Point(
                -sin(state.orientation_in_body_coordinates.z) + state.position_in_body_coordinates.x,
                cos(state.orientation_in_body_coordinates.z) + state.position_in_body_coordinates.y,
                0.0,
            )
        )
        axle_lines.append(line)

    # Compute intersections
    result: List[Tuple[DriveModule, DriveModule, Point]] = []

    col: int = 0
    for col in range(len(drive_module_states)):
        set1 = axle_lines[col]

        row: int = 0
        for row in range(col + 1, len(drive_module_states)):
            set2 = axle_lines[row]

            intersect_point = get_intersect(set1[0], set1[1], set2[0], set2[1])
            result.append(
                (
                    drive_module_states[col],
                    drive_module_states[row],
                    intersect_point
                )
            )

    return result

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
    def on_tick(self, delta_time_in_seconds: float):
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
            state = self.drive_module_trajectory.value_for_module_at(drive_module.name, time_fraction)
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
        self.desired_body_motion = desired_body_motion
        self.body_motion_changed_at_time_in_seconds = self.current_time_in_seconds

    # On clock tick, determine if we need to recalculate the trajectories for the drive modules
    def on_tick(self, delta_time_in_seconds: float):
        self.current_time_in_seconds += delta_time_in_seconds

        # Calculate the current body state
        self.body_state = self.control_model.body_motion_from_wheel_module_states(self.module_states)

        # If the desired body motion was updated after the trajectory was created, then we need to
        # update the trajectory.
        if (self.body_motion_changed_at_time_in_seconds <= self.trajectory_was_started_at_time_in_seconds) and self.drive_module_trajectory is not None:
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
        drive_module_trajectory.set_current_state(self.module_states)

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
            state = self.drive_module_trajectory.value_for_module_at(drive_module.name, time_fraction)
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
        self.desired_body_motion = desired_body_motion
        self.body_motion_changed_at_time_in_seconds = self.current_time_in_seconds

    # On clock tick, determine if we need to recalculate the trajectories for the drive modules
    def on_tick(self, delta_time_in_seconds: float):
        self.current_time_in_seconds += delta_time_in_seconds

        # Calculate the current body state
        self.body_state = self.control_model.body_motion_from_wheel_module_states(self.module_states)

        # If the desired body motion was updated after the trajectory was created, then we need to
        # update the trajectory.
        if (self.body_motion_changed_at_time_in_seconds <= self.trajectory_was_started_at_time_in_seconds) and self.drive_module_trajectory is not None:
            return


        # Allow for different methods of calculating the trajectory
        # - Use current state as start, use desired end Motion as end state --> compute module states --> assume linear for module states
        # - current state --> body state; desired end Motion --> body trajectory --> split up into points --> compute module states --> module trajectory







        # use the twist trajectory to compute the state for the steering modules for the end state
        # and several intermediate points, i.e. determine the vector [[v_i];[gamma_i]].
        #    Use Seegmiller and Kelly to compute the desired velocities and angles
        #
        #    Keep in mind that our update rate determines the points in time where we can do something
        #
        #    Also keep in mind that steering the wheel effectively changes the velocity of the wheel
        #    if we use a co-axial system
        drive_module_trajectory = DriveModuleStateTrajectory(self.modules)
        drive_module_trajectory.set_current_state(self.module_states)

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

class SmoothSteeringController(MultiWheelSteeringController):
    pass
