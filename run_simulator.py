import argparse
from math import pi
from random import random
from typing import Mapping, List, Tuple

# local
from ros_multi_wheel_steering_controller_py.control_model import BodyState, DriveModuleState, Orientation, Point
from ros_multi_wheel_steering_controller_py.drive_module import DriveModule
from ros_multi_wheel_steering_controller_py.multi_wheel_steering_controller import MultiWheelSteeringController
from ros_multi_wheel_steering_controller_py.trajectory import BodyStateTrajectory, DriveModuleStateTrajectory

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
#
# 2. Square - facing forward
#
# 3. Circle
#
# 3. Flower (aka hypotrochoid)
#       x(t) = (R - r) * cos(t) + rho * cos( ((R - r) / r) * t )
#       y(t) = (R - r) * sin(t) - rho * sin( ((R - r) / r) * t )
class SimulatorTrack(object):

    def __init__(self):
        pass

    # The yaw of the body describes the orientation of the body, i.e. in which direction it is pointing
    def initial_yaw(self) -> Orientation:
        pass

    # The heading of the body is the tangent of the movement path, i.e. where the robot is moving to
    def initial_heading(self) -> Orientation:
        pass

def calculate_body_state_from_current_position(path_name: str, current_time: float, current_body_state: BodyState) -> BodyState:

    pass

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

def get_path(name_of_path: str) -> SimulatorTrack:
    pass

def has_reached_endpoint(path_name: str, current_time: float, current_body_state: BodyState) -> bool:
    return False

def initialize_drive_modules(drive_modules: List[DriveModule], align_modules: bool) -> List[DriveModuleState]:
    states: List[DriveModuleState] = []
    for drive_module in drive_modules:
        steering_angle = drive_module.steering_direction_in_radians
        if not align_modules:
            steering_angle = 2 * pi * random()

        state = DriveModuleState(
            drive_module.steering_axis_xy_position.x,
            drive_module.steering_axis_xy_position.y,
            steering_angle,
            0.0
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
            file_.write("x-module-{} [bc] (m), y-module-{} [bc] (m), z-module-{} [bc] (m),".format(i))
            file_.write("x-rot-module-{} [bc] (rad), y-rot-module-{} [bc] (rad), z-rot-module-{} [bc] (rad),".format(i))
            file_.write("x-rot-module-{} [bc] (rad), y-rot-module-{} [bc] (rad), z-rot-module-{} [bc] (rad),".format(i))
            file_.write("x-vel-module-{} [mc] (m/s), y-vel-module-{} [mc] (m/s), y-vel-module-{} [mc] (m/s),".format(i))

def read_arguments() -> dict[str, any]:
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

def record_state_at_time(file_path: str, current_time_in_seconds: float, body_state: BodyState, drive_module_states: List[DriveModuleState]):

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

        file_.write("\n")  # Next line.

def main(args=None):
    arg_dict = read_arguments()
    align_modules: bool = arg_dict["align_modules"] if "align_modules" in arg_dict else False
    name_of_path_to_follow: str = arg_dict["path_to_follow"]
    state_file_path: str = arg_dict["output"]

    # Initialize drive module state
    drive_modules = get_drive_module_info()
    drive_module_states: List[DriveModuleState] = initialize_drive_modules(drive_modules, align_modules)
    body_state: BodyState = None # DETERMINE THE BODY STATE ONCE THE WHEELS ARE IN PLACE

    initialize_state_file(state_file_path, len(drive_modules))

    controller = get_controller(drive_modules)

    time_step_in_seconds = 0.01
    current_sim_time_in_seconds = 0.0

    # Start the simulation
    path_to_follow = get_path(name_of_path_to_follow)

    is_in_correct_position = False
    while not is_in_correct_position:

        # Rotate the body in the correct orientation

        # Rotate the drive modules into the right position before we start
        drive_module_trajectory = controller.drive_module_trajectory_to_achieve_desired_drive_module_orientation(stuff, drive_module_states)

        record_state_at_time(
            state_file_path,
            current_sim_time_in_seconds,
            body_state,
            drive_module_states)
        pass

    simulation_finished = False
    while not simulation_finished:
        current_sim_time_in_seconds = current_sim_time_in_seconds + time_step_in_seconds

        # For current time and position calculate velocity commands
        desired_body_state = calculate_body_state_from_current_position(name_of_path_to_follow, current_sim_time_in_seconds, body_state)

        # Send velocity commands to controller
        # NEED THE CURRENT DRIVE MODULE STATE
        drive_module_trajectory = controller.drive_module_trajectory_to_achieve_desired_body_state(desired_body_state, drive_module_states)

        # For each drive module get the state for the steering and the drive
        # send the values to the drawing / storage API
        time_span_for_trajectory = drive_module_trajectory.time_span()
        time_fraction = time_step_in_seconds / time_span_for_trajectory

        drive_module_states.clear()
        for drive_module in drive_modules:
            drive_module_state = drive_module_trajectory.value_for_module_at(drive_module.name, time_fraction)
            drive_module_states.append(drive_module_state)

        body_trajectory = controller.body_trajectory_from_drive_module_states(drive_module_states)
        body_state = body_trajectory.value_at(time_fraction)

        record_state_at_time(
            state_file_path,
            current_sim_time_in_seconds,
            body_state,
            drive_module_states)

        simulation_finished = has_reached_endpoint(name_of_path_to_follow, current_sim_time_in_seconds, body_state.position_in_world_coordinates)


if __name__ == '__main__':
    main()