from abc import ABC, abstractmethod
import argparse
from math import isclose, pi
from random import random
from typing import Mapping, List, Tuple

# local
from ros_multi_wheel_steering_controller_py.control_model import BodyState, DriveModuleState, Motion, Orientation, Point
from ros_multi_wheel_steering_controller_py.drive_module import DriveModule
from ros_multi_wheel_steering_controller_py.multi_wheel_steering_controller import MultiWheelSteeringController
from ros_multi_wheel_steering_controller_py.trajectory import BodyMotionTrajectory, DriveModuleStateTrajectory

class SimulatorTrack(object):

    @abstractmethod
    def body_motion_for_time(self, time_in_seconds: float, body_state: BodyState) -> Motion:
        pass

    @abstractmethod
    def has_reached_endpoint(self, time_in_seconds: float, body_state: BodyState) -> bool:
        return True

    # The yaw of the body describes the orientation of the body, i.e. in which direction it is pointing
    @abstractmethod
    def initial_yaw(self) -> Orientation:
        pass

    # The heading of the body is the tangent of the movement path, i.e. where the robot is moving to
    @abstractmethod
    def initial_heading(self) -> Orientation:
        pass

# Straight line track in x-direction only.
class StraightLineTrack(SimulatorTrack):

    def body_motion_for_time(self, time_in_seconds: float, body_state: BodyState) -> Motion:
        if body_state.position_in_world_coordinates.x > 1.0:
            return Motion(0.0, 0.0, 0.0)
        else:
            return Motion(0.1, 0.0, 0.0)

    def has_reached_endpoint(self, time_in_seconds: float, body_state: BodyState) -> bool:
        return body_state.position_in_world_coordinates.x > 1.0 and isclose(body_state.motion_in_body_coordinates.linear_velocity.x, 0.0, 1e-9, 1e-9)

    # The yaw of the body describes the orientation of the body, i.e. in which direction it is pointing
    def initial_yaw(self) -> Orientation:
        return Orientation(0.0, 0.0, 0.0)

    # The heading of the body is the tangent of the movement path, i.e. where the robot is moving to
    def initial_heading(self) -> Orientation:
        return Orientation(0.0, 0.0, 0.0)

# Different simulator tracks
#
# 1. Straight line
#       x(t) = a * t; 0 < t < 1.0
#       y(t) = b * t; 0 < t < 1.0
#   a. facing forwards
#   b. facing backwards
#   c. facing left
#   d. facing right
#   e. facing at an angle
#   f. rotating
#
# 2. Square - facing forward
#
# 3. Circle
#
# 3. Flower (aka hypotrochoid)
#       x(t) = (R - r) * cos(t) + rho * cos( ((R - r) / r) * t )
#       y(t) = (R - r) * sin(t) - rho * sin( ((R - r) / r) * t )

def get_controller(drive_modules: List[DriveModule]) -> MultiWheelSteeringController:
    return MultiWheelSteeringController(drive_modules)

def get_drive_module_info() -> List[DriveModule]:
    drive_modules: List[DriveModule] = []
    left_front = DriveModule(
        name="left-front",
        steering_link="joint_steering_left_front",
        drive_link="joint_drive_left_front",
        steering_axis_xy_position=Point(0.5, 0.5, 0.0),
        wheel_radius=0.1,
        steering_motor_maximum_velocity=1.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=1.0,
        drive_motor_minimum_acceleration=0.1,
        drive_motor_maximum_acceleration=1.0
    )
    drive_modules.append(left_front)

    left_rear = DriveModule(
        name="left_rear",
        steering_link="joint_steering_left_rear",
        drive_link="joint_drive_left_rear",
        steering_axis_xy_position=Point(-0.5, 0.5, 0.0),
        wheel_radius=0.1,
        steering_motor_maximum_velocity=1.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=1.0,
        drive_motor_minimum_acceleration=0.1,
        drive_motor_maximum_acceleration=1.0
    )
    drive_modules.append(left_rear)

    right_rear = DriveModule(
        name="right-rear",
        steering_link="joint_steering_right_rear",
        drive_link="joint_drive_right_rear",
        steering_axis_xy_position=Point(-0.5, -0.5, 0.0),
        wheel_radius=0.1,
        steering_motor_maximum_velocity=1.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=1.0,
        drive_motor_minimum_acceleration=0.1,
        drive_motor_maximum_acceleration=1.0
    )
    drive_modules.append(right_rear)

    right_front = DriveModule(
        name="right-front",
        steering_link="joint_steering_right_front",
        drive_link="joint_drive_right_front",
        steering_axis_xy_position=Point(0.5, -0.5, 0.0),
        wheel_radius=0.1,
        steering_motor_maximum_velocity=1.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=1.0,
        drive_motor_minimum_acceleration=0.1,
        drive_motor_maximum_acceleration=1.0
    )
    drive_modules.append(right_front)

    return drive_modules

def get_simulation_track(name_of_path: str) -> SimulatorTrack:
    return StraightLineTrack()

def initialize_drive_modules(drive_modules: List[DriveModule], align_modules: bool) -> List[DriveModuleState]:
    states: List[DriveModuleState] = []
    for drive_module in drive_modules:
        steering_angle = 0.0
        if not align_modules:
            steering_angle = 2 * pi * random()

        state = DriveModuleState(
            drive_module.name,
            drive_module.steering_axis_xy_position.x,
            drive_module.steering_axis_xy_position.y,
            steering_angle,
            0.0,
            0.0,
            0.0,
        )
        states.append(state)

    return states

def initialize_state_file(file_path: str, number_of_modules: int):
    with open(file_path, mode='w') as file_:
        file_.write("Time (s),")
        file_.write("x-body [wc] (m),y-body [wc] (m),z-body [wc] (m),")
        file_.write("x-rot-body [wc] (rad),y-rot-body [wc] (rad),z-rot-body [wc] (rad),")
        file_.write("x-vel-body [bc] (m/s), y-vel-body [bc] (m/s), z-vel-body [bc] (m/s),")
        file_.write("x-vel-body [bc] (rad/s), y-vel-body [bc] (rad/s), z-vel-body [bc] (rad/s),")

        file_.write("number of modules (-),")
        for i in range(number_of_modules):
            file_.write(f"x-module-{i} [bc] (m), y-module-{i} [bc] (m), z-module-{i} [bc] (m),")
            file_.write(f"x-rot-module-{i} [bc] (rad), y-rot-module-{i} [bc] (rad), z-rot-module-{i} [bc] (rad),")
            file_.write(f"x-rot-module-{i} [bc] (rad), y-rot-module-{i} [bc] (rad), z-rot-module-{i} [bc] (rad),")
            file_.write(f"x-vel-module-{i} [mc] (m/s), y-vel-module-{i} [mc] (m/s), y-vel-module-{i} [mc] (m/s),")

def read_arguments() -> Mapping[str, any]:
    parser = argparse.ArgumentParser(
        description="Simulate a 4 wheel steering robot in 2D",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        "-a",
        "--align_modules",
        action="store_true",
        default=True,
        required=False,
        help="Indicates if the modules should be aligned at the start, or if the modules should be set to a random state.")
    parser.add_argument(
        "-p",
        "--path_to_follow",
        action="store",
        default="straight-line-forwards",
        required=False,
        help="The name of the path that should be simulated.")
    parser.add_argument(
        "-o",
        "--output",
        action="store",
        required=True,
        help="The file path for the output file")
    args = parser.parse_args()
    return vars(args)

def record_state_at_time(file_path: str, current_time_in_seconds: float, body_state: BodyState, drive_module_states: List[DriveModuleState], drive_module_trajectory: DriveModuleStateTrajectory):

    # Create a CSV with the following layout
    # body pose in wc, body twist, module count, module 1 pose
    # body_x_wc, body_y_wc, body_gamma_wc, body_v_x, body_v_y, body_v_omega, number_of_modules, module_1_x_bc, module_1_y_bc, module_1_gamma_bc, module_1_v, module_2_x_bc, ...
    with open(file_path, mode='a') as file_:
        # Write the current time
        file_.write("{},".format(current_time_in_seconds))

        # Write the body state
        body_pos = body_state.position_in_world_coordinates
        file_.write("{},{},{},".format(body_pos.x, body_pos.y, body_pos.z))

        body_orient = body_state.orientation_in_world_coordinates
        file_.write("{},{},{},".format(body_orient.x, body_orient.y, body_orient.z))

        body_linear_vel = body_state.linear_velocity_body_coordinates
        file_.write("{},{},{},".format(body_linear_vel.x, body_linear_vel.y, body_linear_vel.z))

        body_angular_vel = body_state.angular_velocity_body_coordinates
        file_.write("{},{},{},".format(body_angular_vel.x, body_angular_vel.y, body_angular_vel.z))

        # Write the number of modules
        file_.write("{},".format(len(drive_module_states)))

        # Write the module states
        for drive_module in drive_module_states:
            module_pos = drive_module.position_in_body_coordinates
            file_.write("{},{},{},".format(module_pos.x, module_pos.y, module_pos.z))

            module_orient = drive_module.orientation_in_body_coordinates
            file_.write("{},{},{},".format(module_orient.x, module_orient.y, module_orient.z))

            module_vel = drive_module.drive_velocity_in_module_coordinates
            file_.write("{},{},{},".format(module_vel.x, module_vel.y, module_vel.z))

        # Record the trajectory

        file_.write("\n")  # Next line.

def simulation_align_drive_modules(
    sim_time_in_seconds: float,
    time_step_in_seconds: float,
    drive_modules: List[DriveModule],
    drive_module_states: List[DriveModuleState],
    body_state: BodyState,
    controller: MultiWheelSteeringController
    ) -> float:

    # Update this --> Incorrect desired state
    drive_module_trajectory = controller.drive_module_trajectory_to_achieve_desired_drive_module_orientation(drive_module_states, drive_module_states)
    are_modules_aligned = True
    while not are_modules_aligned:


        # DO STUFF HERE

        pass
    return sim_time_in_seconds

def simulation_orient_body(
    sim_time_in_seconds: float,
    time_step_in_seconds: float,
    ) -> float:


    # DO STUFF HERE


    return sim_time_in_seconds

def simulation_process_module_trajectory(
    state_file_path: str,
    sim_time_in_seconds: float,
    time_step_in_seconds: float,
    drive_modules: List[DriveModule],
    drive_module_trajectory: DriveModuleStateTrajectory,
    current_body_state: BodyState,
    controller: MultiWheelSteeringController
    ) -> Tuple[List[DriveModuleState], BodyState]:

    # For each drive module get the state for the steering and the drive
    # send the values to the drawing / storage API
    time_span_for_trajectory = drive_module_trajectory.time_span()
    time_fraction = time_step_in_seconds / time_span_for_trajectory

    drive_module_states: List[DriveModuleState] = []
    for drive_module in drive_modules:
        drive_module_state = drive_module_trajectory.value_for_module_at(drive_module.name, time_fraction)
        drive_module_states.append(drive_module_state)

    body_motion = controller.body_motion_from_drive_module_states(drive_module_states)

    # This is probably incorrect. The rotation should come into the position some how ..??????
    updated_body_state = BodyState(
        current_body_state.position_in_world_coordinates.x + body_motion.linear_velocity.x * time_step_in_seconds,
        current_body_state.position_in_world_coordinates.y + body_motion.linear_velocity.y * time_step_in_seconds,
        current_body_state.orientation_in_world_coordinates.z + body_motion.angular_velocity.z * time_step_in_seconds,
        body_motion.linear_velocity.x,
        body_motion.linear_velocity.y,
        body_motion.angular_velocity.z
    )

    record_state_at_time(
        state_file_path,
        sim_time_in_seconds,
        updated_body_state,
        drive_module_states)

    return drive_module_states, updated_body_state

def main(args=None):
    arg_dict = read_arguments()
    align_modules: bool = arg_dict["align_modules"] if "align_modules" in arg_dict else False
    name_of_path_to_follow: str = arg_dict["path_to_follow"]
    state_file_path: str = arg_dict["output"]

    # Initialize drive module state
    drive_modules = get_drive_module_info()
    drive_module_states: List[DriveModuleState] = initialize_drive_modules(drive_modules, align_modules)
    body_state: BodyState = BodyState(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        )

    initialize_state_file(state_file_path, len(drive_modules))

    controller = get_controller(drive_modules)

    time_step_in_seconds = 0.01
    current_sim_time_in_seconds = 0.0

    # Ensure that all the drive modules point in the same direction
    current_sim_time_in_seconds = simulation_align_drive_modules(
        current_sim_time_in_seconds,
        time_step_in_seconds,
        drive_modules,
        drive_module_states,
        body_state,
        controller)

    # Point the body in the direction that the simulation path wants it to point
    track = get_simulation_track(name_of_path_to_follow)
    current_sim_time_in_seconds = simulation_orient_body(
        current_sim_time_in_seconds,
        time_step_in_seconds)

    # Follow the track
    simulation_finished: bool = track.has_reached_endpoint(current_sim_time_in_seconds, body_state)
    while not simulation_finished:
        current_sim_time_in_seconds = current_sim_time_in_seconds + time_step_in_seconds

        # For current time and position calculate velocity commands
        desired_body_motion = track.body_motion_for_time(current_sim_time_in_seconds, body_state)

        drive_module_trajectory = controller.drive_module_trajectory_to_achieve_desired_body_motion(desired_body_motion, drive_module_states)
        drive_module_states, body_state = simulation_process_module_trajectory(
            state_file_path,
            current_sim_time_in_seconds,
            time_step_in_seconds,
            drive_modules,
            drive_module_states,
            drive_module_trajectory,
            body_state,
            controller
        )

        record_state_at_time(
            state_file_path,
            current_sim_time_in_seconds,
            body_state,
            drive_module_states,
            drive_module_trajectory)

        simulation_finished = track.has_reached_endpoint(current_sim_time_in_seconds, body_state)

if __name__ == '__main__':
    main()