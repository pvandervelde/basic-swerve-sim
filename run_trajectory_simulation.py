import argparse
from os import makedirs, path
from pathlib import Path

from typing import Callable, List, Mapping, NamedTuple, Tuple
from swerve_controller.geometry import RealNumberValueSpace
from swerve_controller.profile import SingleVariableLinearProfile, SingleVariableSCurveProfile, SingleVariableTrapezoidalProfile, TransientVariableProfile
import yaml
from yaml.loader import SafeLoader

# local
from sim_output.plots import plot_trajectories

from swerve_controller.control import BodyMotionCommand, DriveModuleMotionCommand, MotionCommand
from swerve_controller.control_model import DriveModuleDesiredValues, DriveModuleMeasuredValues, Point
from swerve_controller.drive_module import DriveModule
from swerve_controller.multi_wheel_steering_controller import (
    LimitedModuleFollowsBodySteeringController,
    ModuleFirstSteeringController,
    ModuleFollowsBodySteeringController,
)
from swerve_controller.sim_utils import instantaneous_center_of_rotation_at_current_time
from swerve_controller.states import BodyState

class MotionPlan(NamedTuple):
    description: str
    name: str
    body_state: BodyState
    initial_drive_module_states: List[DriveModuleDesiredValues]
    motions: List[MotionCommand]

def get_drive_module_info(robot_length: float = 1.2, robot_width: float = 1.1, wheel_radius: float = 0.1, wheel_width=0.1) -> List[DriveModule]:
    steering_motor_maximum_velocity = 2.0
    steering_motor_minimum_acceleration = 0.1
    steering_motor_maximum_acceleration = 10.0
    steering_motor_minimum_jerk = 0.1
    steering_motor_maximum_jerk = 75.0

    drive_motor_maximum_velocity = 1.0
    drive_motor_minimum_acceleration = 0.1
    drive_motor_maximum_acceleration = 5.0
    drive_motor_minimum_jerk = 0.1
    drive_motor_maximum_jerk = 10.0

    drive_modules: List[DriveModule] = []
    left_front = DriveModule(
        name="left-front",
        steering_link="joint_steering_left_front",
        drive_link="joint_drive_left_front",
        steering_axis_xy_position=Point(0.5 * (robot_length - 2 * wheel_radius), 0.5 * (robot_width - wheel_width), 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_motor_maximum_velocity,
        steering_motor_minimum_acceleration=steering_motor_minimum_acceleration,
        steering_motor_maximum_acceleration=steering_motor_maximum_acceleration,
        steering_motor_minimum_jerk=steering_motor_minimum_jerk,
        steering_motor_maximum_jerk=steering_motor_maximum_jerk,
        drive_motor_maximum_velocity=drive_motor_maximum_velocity,
        drive_motor_minimum_acceleration=drive_motor_minimum_acceleration,
        drive_motor_maximum_acceleration=drive_motor_maximum_acceleration,
        drive_motor_minimum_jerk=drive_motor_minimum_jerk,
        drive_motor_maximum_jerk=drive_motor_maximum_jerk
    )
    drive_modules.append(left_front)

    left_rear = DriveModule(
        name="left_rear",
        steering_link="joint_steering_left_rear",
        drive_link="joint_drive_left_rear",
        steering_axis_xy_position=Point(-0.5 * (robot_length - 2 * wheel_radius), 0.5 * (robot_width - wheel_width), 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_motor_maximum_velocity,
        steering_motor_minimum_acceleration=steering_motor_minimum_acceleration,
        steering_motor_maximum_acceleration=steering_motor_maximum_acceleration,
        steering_motor_minimum_jerk=steering_motor_minimum_jerk,
        steering_motor_maximum_jerk=steering_motor_maximum_jerk,
        drive_motor_maximum_velocity=drive_motor_maximum_velocity,
        drive_motor_minimum_acceleration=drive_motor_minimum_acceleration,
        drive_motor_maximum_acceleration=drive_motor_maximum_acceleration,
        drive_motor_minimum_jerk=drive_motor_minimum_jerk,
        drive_motor_maximum_jerk=drive_motor_maximum_jerk
    )
    drive_modules.append(left_rear)

    right_rear = DriveModule(
        name="right-rear",
        steering_link="joint_steering_right_rear",
        drive_link="joint_drive_right_rear",
        steering_axis_xy_position=Point(-0.5 * (robot_length - 2 * wheel_radius), -0.5 * (robot_width - wheel_width), 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_motor_maximum_velocity,
        steering_motor_minimum_acceleration=steering_motor_minimum_acceleration,
        steering_motor_maximum_acceleration=steering_motor_maximum_acceleration,
        steering_motor_minimum_jerk=steering_motor_minimum_jerk,
        steering_motor_maximum_jerk=steering_motor_maximum_jerk,
        drive_motor_maximum_velocity=drive_motor_maximum_velocity,
        drive_motor_minimum_acceleration=drive_motor_minimum_acceleration,
        drive_motor_maximum_acceleration=drive_motor_maximum_acceleration,
        drive_motor_minimum_jerk=drive_motor_minimum_jerk,
        drive_motor_maximum_jerk=drive_motor_maximum_jerk
    )
    drive_modules.append(right_rear)

    right_front = DriveModule(
        name="right-front",
        steering_link="joint_steering_right_front",
        drive_link="joint_drive_right_front",
        steering_axis_xy_position=Point(0.5 * (robot_length - 2 * wheel_radius), -0.5 * (robot_width - wheel_width), 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_motor_maximum_velocity,
        steering_motor_minimum_acceleration=steering_motor_minimum_acceleration,
        steering_motor_maximum_acceleration=steering_motor_maximum_acceleration,
        steering_motor_minimum_jerk=steering_motor_minimum_jerk,
        steering_motor_maximum_jerk=steering_motor_maximum_jerk,
        drive_motor_maximum_velocity=drive_motor_maximum_velocity,
        drive_motor_minimum_acceleration=drive_motor_minimum_acceleration,
        drive_motor_maximum_acceleration=drive_motor_maximum_acceleration,
        drive_motor_minimum_jerk=drive_motor_minimum_jerk,
        drive_motor_maximum_jerk=drive_motor_maximum_jerk
    )
    drive_modules.append(right_front)

    return drive_modules

def get_linear_motion_profile(start: float, end: float, end_time: float, number_space: RealNumberValueSpace) -> TransientVariableProfile:
    return SingleVariableLinearProfile(start, end, end_time, number_space)

def get_motions(input_files: List[str]) -> List[MotionPlan]:
    result: List[MotionPlan] = []

    for input_file in input_files:
        relative = Path(input_file)

        with open(relative.absolute()) as f:
            print("Reading {} ...".format(f.name))
            data = yaml.load(f, Loader=SafeLoader)
            data_plan = data["plan"]

            data_initial_body_state = data_plan["start_state"]["body"]
            initial_body_state: BodyState = BodyState(
                body_x_in_meters=data_initial_body_state["position_in_meters_relative_to_world"]["x"],
                body_y_in_meters=data_initial_body_state["position_in_meters_relative_to_world"]["y"],
                body_orientation_in_radians=data_initial_body_state["orientation_in_radians_relative_to_world"]["z"],
                body_angular_z_velocity_in_radians_per_second=data_initial_body_state["angular_velocity_in_radians_per_second"]["z"],
                body_linear_x_velocity_in_meters_per_second=data_initial_body_state["linear_velocity_in_meters_per_second"]["x"],
                body_linear_y_velocity_in_meters_per_second=data_initial_body_state["linear_velocity_in_meters_per_second"]["y"],
                body_angular_z_acceleration_in_radians_per_second_quared=0.0,
                body_linear_x_acceleration_in_meters_per_second_quared=0.0,
                body_linear_y_acceleration_in_meters_per_second_quared=0.0,
                body_angular_z_jerk_in_radians_per_second_cubed=0.0,
                body_linear_x_jerk_in_meters_per_second_cubed=0.0,
                body_linear_y_jerk_in_meters_per_second_cubed=0.0
            )

            data_initial_module_state = data_plan["start_state"]["modules"]
            initial_module_state: List[DriveModuleDesiredValues] = []
            for module_initial_state in data_initial_module_state:
                state = DriveModuleDesiredValues(
                    name=module_initial_state["name"],
                    steering_angle_in_radians=module_initial_state["orientation_in_radians_relative_to_body"],
                    drive_velocity_in_meters_per_second=module_initial_state["velocity_in_meters_per_second"],
                )

                initial_module_state.append(state)

            data_commands = data_plan["commands"]
            commands: List[MotionCommand] = []
            for data_command in data_commands:

                time_span = data_command["time_span"]
                if "modules" in data_command:
                    data_command_module = data_command["modules"]

                    command_module_state: List[DriveModuleDesiredValues] = []
                    for module_command_state in data_command_module:
                        state = DriveModuleDesiredValues(
                            name=module_command_state["name"],
                            steering_angle_in_radians=module_command_state["orientation_in_radians_relative_to_body"],
                            drive_velocity_in_meters_per_second=module_command_state["velocity_in_meters_per_second"],
                        )

                        command_module_state.append(state)

                    command = DriveModuleMotionCommand(time_span, command_module_state)

                    commands.append(command)
                else:
                    if "body" in data_command:
                        data_command_body = data_command["body"]
                        command = BodyMotionCommand(
                            time_span,
                            data_command_body["linear_velocity_in_meters_per_second"]["x"],
                            data_command_body["linear_velocity_in_meters_per_second"]["y"],
                            data_command_body["angular_velocity_in_radians_per_second"]["z"],
                        )

                        commands.append(command)

            plan = MotionPlan(
                description=data_plan["description"],
                name=data_plan["name"],
                body_state=initial_body_state,
                initial_drive_module_states=initial_module_state,
                motions=commands,
            )

            result.append(plan)

    return result

def get_scurve_profile(start: float, end: float, end_time: float, number_space: RealNumberValueSpace) -> TransientVariableProfile:
    return SingleVariableSCurveProfile(start, end, end_time, number_space)

def get_trapezoidal_profile(start: float, end: float, end_time: float, number_space: RealNumberValueSpace) -> TransientVariableProfile:
    return SingleVariableTrapezoidalProfile(start, end, end_time, number_space)

def initialize_drive_modules(drive_modules: List[DriveModule], module_states: List[DriveModuleDesiredValues]) -> List[DriveModuleMeasuredValues]:
    states: List[DriveModuleMeasuredValues] = []

    index = 0
    for drive_module in drive_modules:

        state = DriveModuleMeasuredValues(
            drive_module.name,
            drive_module.steering_axis_xy_position.x,
            drive_module.steering_axis_xy_position.y,
            module_states[index].steering_angle_in_radians,
            0.0,
            0.0,
            0.0,
            module_states[index].drive_velocity_in_meters_per_second,
            0.0,
            0.0,
        )
        states.append(state)

        index += 1

    return states

def initialize_state_file(file_path: str, number_of_modules: int):
    with open(file_path, mode='w') as file_:
        file_.write("Time (s),")
        file_.write("x-body [wc] (m),y-body [wc] (m),z-body [wc] (m),")
        file_.write("x-rot-body [wc] (rad),y-rot-body [wc] (rad),z-rot-body [wc] (rad),")

        file_.write("x-vel-body [bc] (m/s), y-vel-body [bc] (m/s), z-vel-body [bc] (m/s),")
        file_.write("x-rotvel-body [bc] (rad/s), y-rotvel-body [bc] (rad/s), z-rotvel-body [bc] (rad/s),")

        file_.write("x-acc-body [bc] (m/s^2), y-acc-body [bc] (m/s^2), z-acc-body [bc] (m/s^2),")
        file_.write("x-rotacc-body [bc] (rad/s^2), y-rotacc-body [bc] (rad/s^2), z-rotacc-body [bc] (rad/s^2),")

        file_.write("x-jerk-body [bc] (m/s^3), y-jerk-body [bc] (m/s^3), z-jerk-body [bc] (m/s^3),")
        file_.write("x-rotjerk-body [bc] (rad/s^3), y-rotjerk-body [bc] (rad/s^3), z-rotjerk-body [bc] (rad/s^3),")

        file_.write("number of modules (-),")
        for i in range(number_of_modules):
            file_.write(f"x-module-{i} [bc] (m), y-module-{i} [bc] (m), z-module-{i} [bc] (m),")
            file_.write(f"x-rot-module-{i} [bc] (rad), y-rot-module-{i} [bc] (rad), z-rot-module-{i} [bc] (rad),")

            file_.write(f"x-vel-module-{i} [mc] (m/s), y-vel-module-{i} [mc] (m/s), z-vel-module-{i} [mc] (m/s),")
            file_.write(f"x-rotvel-module-{i} [bc] (rad/s), y-rotvel-module-{i} [bc] (rad/s), z-rotvel-module-{i} [bc] (rad/s),")

            file_.write(f"x-acc-module-{i} [mc] (m/s^2), y-acc-module-{i} [mc] (m/s^2), z-acc-module-{i} [mc] (m/s^2),")
            file_.write(f"x-rotacc-module-{i} [bc] (rad/s^2), y-rotacc-module-{i} [bc] (rad/s^2), z-rotacc-module-{i} [bc] (rad/s^2),")

            file_.write(f"x-jerk-module-{i} [mc] (m/s^3), y-jerk-module-{i} [mc] (m/s^3), z-jerk-module-{i} [mc] (m/s^3),")
            file_.write(f"x-rotjerk-module-{i} [bc] (rad/s^3), y-rotjerk-module-{i} [bc] (rad/s^3), z-rotjerk-module-{i} [bc] (rad/s^3),")

        file_.write("\n")  # Next line.

def read_arguments() -> Mapping[str, any]:
    parser = argparse.ArgumentParser(
        description="Simulate a 4 wheel steering robot in 2D",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        "-f",
        "--file",
        action="append",
        required=True,
        type=str,
        help="The file path for the input file which contains the desired motions to be executed. Can be provided multiple times.")

    parser.add_argument(
        "-o",
        "--output",
        action="store",
        required=True,
        type=str,
        help="The directory path for the output files")

    parser.add_argument(
        "-ng",
        "--no-graphs",
        action="store_true",
        required=False,
        help="Indicates if graphs should be generated or not. If not specified graphs will be created."
    )

    parser.add_argument(
        "-c",
        "--control-level",
        action="store",
        choices=['module', 'body', 'limited'],
        default='module',
        required=False,
        help="The name of the controller that should be used for the simulation. Current options are: 'module', 'body', 'limited'"
    )

    parser.add_argument(
        "-mp",
        "--motion-profile",
        action="store",
        choices=['linear', 'trapezoidal', 'scurve'],
        default='module',
        required=False,
        help="The name of the motion profile that controls the velocity and acceleration profiles for the drive module motors. Current options are: 'linear', 'trapezoidal', 'scurve'"
    )

    args = parser.parse_args()

    return vars(args)

def record_state_at_time(file_path: str, current_time_in_seconds: float, body_state: BodyState, drive_module_states: List[DriveModuleMeasuredValues]):

    # Create a CSV with the following layout
    # body pose in wc, body twist, module count, module 1 info, .. , module N info
    # with:
    #     body_x_wc, body_y_wc, body_gamma_wc, body_v_x, body_v_y, body_v_omega, number_of_modules, module_1_x_bc, module_1_y_bc, module_1_gamma_bc, module_1_v, module_2_x_bc, ...
    with open(file_path, mode='a') as file_:
        # Write the current time
        file_.write("{},".format(current_time_in_seconds))

        # Write the body state
        body_pos = body_state.position_in_world_coordinates
        file_.write("{},{},{},".format(body_pos.x, body_pos.y, body_pos.z))

        body_orient = body_state.orientation_in_world_coordinates
        file_.write("{},{},{},".format(body_orient.x, body_orient.y, body_orient.z))

        body_linear_vel = body_state.motion_in_body_coordinates.linear_velocity
        file_.write("{},{},{},".format(body_linear_vel.x, body_linear_vel.y, body_linear_vel.z))

        body_angular_vel = body_state.motion_in_body_coordinates.angular_velocity
        file_.write("{},{},{},".format(body_angular_vel.x, body_angular_vel.y, body_angular_vel.z))

        file_.write("{},{},{},".format(0.0, 0.0, 0.0))

        file_.write("{},{},{},".format(0.0, 0.0, 0.0))

        file_.write("{},{},{},".format(0.0, 0.0, 0.0))

        file_.write("{},{},{},".format(0.0, 0.0, 0.0))

        # Write the number of modules
        file_.write("{},".format(len(drive_module_states)))

        # Write the module states
        for drive_module in drive_module_states:
            module_pos = drive_module.position_in_body_coordinates
            file_.write("{},{},{},".format(module_pos.x, module_pos.y, module_pos.z))
            module_orient = drive_module.orientation_in_body_coordinates
            file_.write("{},{},{},".format(module_orient.x, module_orient.y, module_orient.z))

            module_lin_vel = drive_module.drive_velocity_in_module_coordinates
            file_.write("{},{},{},".format(module_lin_vel.x, module_lin_vel.y, module_lin_vel.z))
            module_rot_vel = drive_module.orientation_velocity_in_body_coordinates
            file_.write("{},{},{},".format(module_rot_vel.x, module_rot_vel.y, module_rot_vel.z))

            module_lin_acc = drive_module.drive_acceleration_in_module_coordinates
            file_.write("{},{},{},".format(module_lin_acc.x, module_lin_acc.y, module_lin_acc.z))
            module_rot_acc = drive_module.orientation_acceleration_in_body_coordinates
            file_.write("{},{},{},".format(module_rot_acc.x, module_rot_acc.y, module_rot_acc.z))

            module_lin_jerk = drive_module.drive_jerk_in_module_coordinates
            file_.write("{},{},{},".format(module_lin_jerk.x, module_lin_jerk.y, module_lin_jerk.z))
            module_rot_jerk = drive_module.orientation_jerk_in_body_coordinates
            file_.write("{},{},{},".format(module_rot_jerk.x, module_rot_jerk.y, module_rot_jerk.z))

        file_.write("\n")  # Next line.

def simulation_run_trajectories(arg_dict: Mapping[str, any]):
    input_files: List[str] = arg_dict["file"]
    output_directory: str = arg_dict["output"]
    do_not_draw_graphs: bool = arg_dict["no_graphs"]
    controller: str = arg_dict["control_level"]
    motion_profile: str = arg_dict["motion_profile"]
    print("Running trajectory simulation")
    print("Simulating motion for the following files:")
    for input_file in input_files:
        print("    {}".format(input_file))

    print("Outputting to {}".format(output_directory))

    drive_modules = get_drive_module_info()
    motions = get_motions(input_files)
    for motion_set in motions:
        simulation_run_trajectory(output_directory, drive_modules, motion_set, controller, motion_profile, do_not_draw_graphs)

def simulation_run_trajectory(
    output_directory: str,
    drive_modules: List[DriveModule],
    motion_set: MotionPlan,
    controller_name: str,
    motion_profile:str,
    do_not_draw_graphs: bool,
    ):

    if motion_profile == 'linear':
        motion_profile_func = get_linear_motion_profile

    if motion_profile == 'trapezoidal':
        motion_profile_func = get_trapezoidal_profile

    if motion_profile == 'scurve':
        motion_profile_func = get_scurve_profile

    if controller_name == 'module':
        controller = ModuleFirstSteeringController(drive_modules, motion_profile_func)

    if controller_name == 'body':
        controller = ModuleFollowsBodySteeringController(drive_modules, motion_profile_func)

    if controller_name == 'limited':
        controller = LimitedModuleFollowsBodySteeringController(drive_modules, motion_profile_func, 100)

    motion_directory = path.join(output_directory, motion_set.name, controller_name, motion_profile)

    state_file_path = path.join(motion_directory, "sim_results.csv")
    if not path.isdir(motion_directory):
        print("Output directory {} does not exist. Creating directory ...".format(motion_directory))
        makedirs(motion_directory)

    print("Initializing state file at {}".format(state_file_path))
    initialize_state_file(state_file_path, len(drive_modules))

    initial_module_states: List[DriveModuleMeasuredValues] = initialize_drive_modules(
        drive_modules,
        motion_set.initial_drive_module_states)

    controller.on_state_update(initial_module_states)

    simulation_rate_in_hz = 100
    current_sim_time_in_seconds = 0.0

    # The motion set should be a command 'trajectory', i.e. a collection of ControlCommands with the
    # time span over which the command state should be achieved

    points_in_time: List[float] = [ ]
    body_states: List[BodyState] = []
    drive_states: List[List[DriveModuleMeasuredValues]] = []
    icr_map: List[Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]]] = []

    body_state = motion_set.body_state

    for motion in motion_set.motions:
        print("Processing motion set at {} ...".format(current_sim_time_in_seconds))

        controller.on_tick(current_sim_time_in_seconds)
        controller.on_desired_state_update(motion)

        time_for_motion = controller.time_for_current_movement()

        step_count = int(time_for_motion * simulation_rate_in_hz)
        time_step_in_seconds =  1.0 / float(simulation_rate_in_hz)

        for time_index in range(1, step_count + 1):
            controller.on_tick(current_sim_time_in_seconds)

            points_in_time.append(current_sim_time_in_seconds)

            # Record the current state of the system
            current_drive_module_states = controller.drive_module_states_at_current_time()
            icr_coordinate_map = instantaneous_center_of_rotation_at_current_time(current_drive_module_states)
            icr_map.append(
                    (
                        current_sim_time_in_seconds,
                        icr_coordinate_map
                    )
                )

            body_state = controller.body_state_at_current_time()
            body_states.append(body_state)

            drive_states.append(current_drive_module_states)

            record_state_at_time(
                state_file_path,
                current_sim_time_in_seconds,
                body_state,
                current_drive_module_states)

            current_sim_time_in_seconds += time_step_in_seconds

            print("Processing step at {} ...".format(current_sim_time_in_seconds))

            # Predict what the next state is going to be
            desired_drive_module_states = controller.drive_module_state_at_future_time(current_sim_time_in_seconds)
            predicted_drive_states: List[DriveModuleMeasuredValues] = []
            for module_index in range(len(drive_modules)):

                orientation_velocity = (desired_drive_module_states[module_index].steering_angle_in_radians - current_drive_module_states[module_index].orientation_in_body_coordinates.z) / time_step_in_seconds
                orientation_acceleration = (orientation_velocity - current_drive_module_states[module_index].orientation_velocity_in_body_coordinates.z) / time_step_in_seconds
                orientation_jerk = (orientation_acceleration - current_drive_module_states[module_index].orientation_acceleration_in_body_coordinates.z) / time_step_in_seconds

                drive_acceleration = (desired_drive_module_states[module_index].drive_velocity_in_meters_per_second - current_drive_module_states[module_index].drive_velocity_in_module_coordinates.x) / time_step_in_seconds
                drive_jerk = (drive_acceleration - current_drive_module_states[module_index].drive_acceleration_in_module_coordinates.x) / time_step_in_seconds

                predicted_drive_states.append(
                    DriveModuleMeasuredValues(
                        drive_modules[module_index].name,
                        drive_modules[module_index].steering_axis_xy_position.x,
                        drive_modules[module_index].steering_axis_xy_position.y,
                        desired_drive_module_states[module_index].steering_angle_in_radians,
                        orientation_velocity,
                        orientation_acceleration,
                        orientation_jerk,
                        desired_drive_module_states[module_index].drive_velocity_in_meters_per_second,
                        drive_acceleration,
                        drive_jerk,
                    )
                )

            # Set the predicted state as the next state
            controller.on_state_update(predicted_drive_states)

    # Now draw all the graphs
    if not do_not_draw_graphs:
        plot_trajectories(
            motion_set.description,
            motion_set.name,
            motion_directory,
            points_in_time,
            body_states,
            drive_modules,
            drive_states,
            icr_map,
            "blue")

def main(args=None):
    arg_dict = read_arguments()

    simulation_run_trajectories(arg_dict)

if __name__ == '__main__':
    main()