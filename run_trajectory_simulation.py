from abc import ABC, abstractmethod
import argparse
from math import cos, isclose, isinf, pi, radians, sin, sqrt
from os import makedirs, path
from pathlib import Path
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.offline as py
from random import random
from typing import List, Mapping, NamedTuple, Tuple
import yaml
from yaml.loader import SafeLoader

# local
from swerve_controller.control import BodyMotionCommand, DriveModuleMotionCommand, MotionCommand
from swerve_controller.control_model import DriveModuleDesiredValues, DriveModuleMeasuredValues, Orientation, Point
from swerve_controller.drive_module import DriveModule
from swerve_controller.multi_wheel_steering_controller import (
    LinearBodyFirstSteeringController,
    LinearModuleFirstSteeringController,
    MultiWheelSteeringController,
)
from swerve_controller.sim_utils import instantaneous_center_of_rotation_at_current_time
from swerve_controller.states import BodyState, BodyMotion
from swerve_controller.trajectory import BodyMotionTrajectory, DriveModuleStateTrajectory

class ProfilePlotValues(NamedTuple):
    name: str
    markers: Mapping[str, int]
    x_values: List[float]
    y_values: List[float]
    annotations: List[str] = []

class MotionPlan(NamedTuple):
    description: str
    name: str
    body_state: BodyState
    initial_drive_module_states: List[DriveModuleDesiredValues]
    motions: List[MotionCommand]

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

def generate_plot_information(
    points_in_time: List[float],
    body_states: List[BodyState],
    drive_modules: List[DriveModule],
    drive_states: List[List[DriveModuleMeasuredValues]],
    icr_coordinate_map: List[Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]]],
    color: str,
    ) -> List[List[ProfilePlotValues]]:
    default_size = 6

    plots: List[List[ProfilePlotValues]] = []

    markers: List[dict] = [
        dict(color = "darkblue", symbol="circle-open", size=default_size),
        dict(color = "darkgoldenrod", symbol="square-open", size=default_size),
        dict(color = "darkgreen", symbol="diamond-open", size=default_size),
        dict(color = "darkmagenta", symbol="cross-open", size=default_size),
        dict(color = "darksalmon", symbol="triangle-up-open", size=default_size),
        dict(color = "darkturquoise", symbol="pentagon-open", size=default_size),
        dict(color = "darkorange", symbol="hexagram-open", size=default_size),
        dict(color = "darkviolet", symbol="hourglass-open", size=default_size),
        dict(color = "darkgray", symbol="arrow-left-open", size=default_size),
        dict(color = "darkkhaki", symbol="x-open", size=default_size),
    ]

    # Body position
    plots.append(
        [
            ProfilePlotValues(
                name="body position",
                markers=markers[0],
                x_values=[b.position_in_world_coordinates.x for b in body_states],
                y_values=[b.position_in_world_coordinates.y for b in body_states],
                annotations=points_in_time,
            )
        ]
    )

    plots.append(
        [
            ProfilePlotValues(
                name="body x-position",
                markers=markers[0],
                x_values=points_in_time,
                y_values=[b.position_in_world_coordinates.x for b in body_states],
            ),
            ProfilePlotValues(
                name="body y-position",
                markers=markers[1],
                x_values=points_in_time,
                y_values=[b.position_in_world_coordinates.y for b in body_states]
            ),
            ProfilePlotValues(
                name="body orientation",
                markers=markers[2],
                x_values=points_in_time,
                y_values=[b.orientation_in_world_coordinates.z for b in body_states]
            )
        ]
    )

    # Body velocity
    plots.append(
        [
            ProfilePlotValues(
                name="body x-velocity",
                markers=markers[0],
                x_values=points_in_time,
                y_values=[b.motion_in_body_coordinates.linear_velocity.x for b in body_states]
            ),
            ProfilePlotValues(
                name="body y-velocity",
                markers=markers[1],
                x_values=points_in_time,
                y_values=[b.motion_in_body_coordinates.linear_velocity.y for b in body_states]
            ),
            ProfilePlotValues(
                name="body rotation-velocity",
                markers=markers[2],
                x_values=points_in_time,
                y_values=[b.motion_in_body_coordinates.angular_velocity.z for b in body_states]
            ),
        ]
    )

    drive_velocities: List[ProfilePlotValues] = []
    plots.append(drive_velocities)

    drive_orientations: List[ProfilePlotValues] = []
    plots.append(drive_orientations)

    module_index = 0
    for drive_module in drive_modules:

        drive_velocities.append(
            ProfilePlotValues(
                name="{} drive velocity".format(drive_module.name),
                markers=markers[module_index],
                x_values=points_in_time,
                y_values=[d[module_index].drive_velocity_in_module_coordinates.x for d in drive_states]
            )
        )

        # drive_velocities.append(
        #     ProfilePlotValues(
        #         name="{} drive acceleration".format(drive_module.name),
        #         markers=markers[0]
        #         x_values=points_in_time,
        #         y_values=[d[module_index].drive_acceleration_in_module_coordinates.x for d in drive_states]
        #     )
        # )

        # drive_velocities.append(
        #     ProfilePlotValues(
        #         name="{} drive jerk".format(drive_module.name),
        #         markers=markers[0]
        #         x_values=points_in_time,
        #         y_values=[d[module_index].drive_jerk_in_module_coordinates.x for d in drive_states]
        #     )
        # )

        drive_orientations.append(
            ProfilePlotValues(
                name="{} drive orientation".format(drive_module.name),
                markers=markers[module_index],
                x_values=points_in_time,
                y_values=[d[module_index].orientation_in_body_coordinates.z for d in drive_states]
            )
        )

        # wheel_orientation.append(
        #     ProfilePlotValues(
        #         name="{} drive orientation velocity".format(drive_module.name),
        #         markers=markers[0]
        #         x_values=points_in_time,
        #         y_values=[d[module_index].orientation_velocity_in_body_coordinates.z for d in drive_states]
        #     )
        # )

        # wheel_orientation.append(
        #     ProfilePlotValues(
        #         name="{} drive orientation acceleration".format(drive_module.name),
        #         markers=markers[0]
        #         x_values=points_in_time,
        #         y_values=[d[module_index].orientation_acceleration_in_body_coordinates.z for d in drive_states]
        #     )
        # )

        # wheel_orientation.append(
        #     ProfilePlotValues(
        #         name="{} drive orientation jerk".format(drive_module.name),
        #         markers=markers[0]
        #         x_values=points_in_time,
        #         y_values=[d[module_index].orientation_jerk_in_body_coordinates.z for d in drive_states]
        #     )
        # )

        module_index += 1

    # Plot the ICR
    icr_module_names: List[Tuple[str, str]] = [ (i[0].name, i[1].name) for i in icr_coordinate_map[0][1] ]
    x_values: List[List[float]] = [
        [],
        [],
        [],
        [],
        [],
        [],
    ]
    y_values: List[List[float]] = [
        [],
        [],
        [],
        [],
        [],
        [],
    ]
    for icrs_at_time in icr_coordinate_map:
        index = 0
        for icr in icrs_at_time[1]:

            if not isinf(icr[2].x) and not isinf(icr[2].y):

                if abs(icr[2].x) < 25 and abs(icr[2].y) < 25:
                    if (not isclose(icr[0].drive_velocity_in_module_coordinates.x, 0.0, rel_tol=1e-7, abs_tol=1e-7)) or (not isclose(icr[1].drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-7, abs_tol=1e-7)):
                        x_values[index].append(icr[2].x)
                        y_values[index].append(icr[2].y)

            index += 1

    icr_plots: List[ProfilePlotValues] = []
    plots.append(icr_plots)
    for icr_index in range(len(x_values)):
        names = icr_module_names[icr_index]
        plot_name = "icr - {}-{}".format(names[0], names[1])
        icr_plots.append(
            ProfilePlotValues(
                name=plot_name,
                markers=markers[icr_index],
                x_values=x_values[icr_index],
                y_values=y_values[icr_index],
            )
        )

    return plots

def generate_plot_traces(plots: List[List[ProfilePlotValues]]) -> List[go.Figure]:
    figures: List[go.Figure] = []

    for lists in plots:

        fig = get_plot(1, 1)
        figures.append(fig)

        for values in lists:
            print("Creating plot with title [{}] ...".format(values.name))
            fig.append_trace(
                    go.Scatter(
                        x=values.x_values,
                        y=values.y_values,
                        mode='markers',
                        marker=values.markers,
                        name=values.name,
                        showlegend=True,
                        text=values.annotations,
                        marker_colorscale="Rainbow",
                    ),
                    row = 1,
                    col = 1)
            fig.update_yaxes(title_text=values.name)

    return figures

def get_controller(drive_modules: List[DriveModule]) -> Mapping[str, MultiWheelSteeringController]:
    return {
        "LinearModuleFirstSteeringController": LinearModuleFirstSteeringController(drive_modules),
        #"LinearBodyFirstSteeringController": LinearBodyFirstSteeringController(drive_modules),
    }

def get_drive_module_info() -> List[DriveModule]:
    drive_modules: List[DriveModule] = []
    left_front = DriveModule(
        name="left-front",
        steering_link="joint_steering_left_front",
        drive_link="joint_drive_left_front",
        steering_axis_xy_position=Point(0.5, 0.5, 0.0),
        wheel_radius=0.1,
        steering_motor_maximum_velocity=10.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=10.0,
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
        steering_motor_maximum_velocity=10.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=10.0,
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
        steering_motor_maximum_velocity=10.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=10.0,
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
        steering_motor_maximum_velocity=10.0,
        steering_motor_minimum_acceleration=0.1,
        steering_motor_maximum_acceleration=1.0,
        drive_motor_maximum_velocity=10.0,
        drive_motor_minimum_acceleration=0.1,
        drive_motor_maximum_acceleration=1.0
    )
    drive_modules.append(right_front)

    return drive_modules

def get_plot(rows: int, cols: int) -> go.Figure:
    fig = make_subplots(
        rows=rows,
        cols=cols,
        shared_xaxes=False,
        shared_yaxes=True,
        )

    return fig

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

def plot_trajectories(
    set_name: str,
    short_name: str,
    output_directory: str,
    points_in_time: List[float],
    body_states: List[BodyState],
    drive_modules: List[DriveModule],
    drive_states: List[List[DriveModuleMeasuredValues]],
    icr_coordinate_map: List[Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]]],
    color: str,
    ):

    plots = generate_plot_information(points_in_time, body_states, drive_modules, drive_states, icr_coordinate_map, color)
    figs = generate_plot_traces(plots)

    index = 0
    for fig in figs:
        # fig.update_traces(
        #     marker=dict(
        #         size=3,
        #         line=dict(
        #             width=1,
        #             color='DarkSlateGrey')),
        #     selector=dict(mode='markers'))
        fig.update_layout(
            template='ggplot2',
            showlegend=True,
            legend= {'itemsizing': 'constant'}
            )

        plot_file_path = path.join(output_directory, "{}-{}.html".format(short_name, index))
        py.plot(fig, filename = plot_file_path)

        figure_file_path = path.join(output_directory, "{}-{}.png".format(short_name, index))
        fig.write_image(figure_file_path, width=2100, height=1500)

        index += 1

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
    print("Running trajectory simulation")
    print("Simulating motion for the following files:")
    for input_file in input_files:
        print("    {}".format(input_file))

    print("Outputting to {}".format(output_directory))

    drive_modules = get_drive_module_info()
    motions = get_motions(input_files)
    for motion_set in motions:
        motion_directory = path.join(output_directory, motion_set.name)
        simulation_run_trajectory(motion_directory, drive_modules, motion_set)

def simulation_run_trajectory(
    output_directory: str,
    drive_modules: List[DriveModule],
    motion_set: MotionPlan,
    ):

    state_file_path = path.join(output_directory, "{}.csv".format(motion_set.name))
    if not path.isdir(output_directory):
        print("Output directory {} does not exist. Creating directory ...".format(output_directory))
        makedirs(output_directory)

    print("Initializing state file at {}".format(state_file_path))
    initialize_state_file(state_file_path, len(drive_modules))

    drive_module_states: List[DriveModuleMeasuredValues] = initialize_drive_modules(
        drive_modules,
        motion_set.initial_drive_module_states)

    controller = (list(get_controller(drive_modules).values()))[0]
    controller.on_state_update(drive_module_states)

    simulation_rate_in_hz = 100
    current_sim_time_in_seconds = 0.0
    time_step_in_seconds = 1.0 / simulation_rate_in_hz

    # The motion set should be a command 'trajectory', i.e. a collection of ControlCommands with the
    # time span over which the command state should be achieved

    points_in_time: List[float] = [ current_sim_time_in_seconds ]
    body_states: List[BodyState] = []
    drive_states: List[List[DriveModuleMeasuredValues]] = []
    icr_map: List[Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]]] = []

    body_state = motion_set.body_state

    for motion in motion_set.motions:
        controller.on_desired_state_update(motion)

        step_count = int(motion.time_for_motion() * simulation_rate_in_hz)

        for i in range(1, step_count + 1):
            controller.on_tick(current_sim_time_in_seconds)

            current_sim_time_in_seconds += time_step_in_seconds

            print("Processing step at {} ...".format(current_sim_time_in_seconds))

            points_in_time.append(current_sim_time_in_seconds)

            drive_module_states = controller.drive_module_state_at_future_time(current_sim_time_in_seconds)
            drive_states.append(drive_module_states)

            icr_coordinate_map = instantaneous_center_of_rotation_at_current_time(drive_module_states)
            icr_map.append(
                    (
                        current_sim_time_in_seconds,
                        icr_coordinate_map
                    )
                )

            body_motion = controller.get_control_model().body_motion_from_wheel_module_states(drive_module_states)

            local_x_distance = time_step_in_seconds * body_motion.linear_velocity.x
            local_y_distance = time_step_in_seconds * body_motion.linear_velocity.y
            global_orientation = body_state.orientation_in_world_coordinates.z + time_step_in_seconds * body_motion.angular_velocity.z
            body_state = BodyState(
                body_state.position_in_world_coordinates.x + local_x_distance * cos(global_orientation) - local_y_distance * sin(global_orientation),
                body_state.position_in_world_coordinates.y + local_x_distance * sin(global_orientation) + local_y_distance * cos(global_orientation),
                global_orientation,
                body_motion.linear_velocity.x,
                body_motion.linear_velocity.y,
                body_motion.angular_velocity.z,
            )

            body_states.append(body_state)

            record_state_at_time(
                state_file_path,
                current_sim_time_in_seconds,
                body_state,
                drive_module_states)

            current_drive_states: List[DriveModuleMeasuredValues] = []
            for desired_state in drive_module_states:
                current_drive_states.append(
                    DriveModuleMeasuredValues(
                        desired_state.name,
                        desired_state.position_in_body_coordinates.x,
                        desired_state.position_in_body_coordinates.y,
                        desired_state.orientation_in_body_coordinates.z,
                        desired_state.orientation_velocity_in_body_coordinates.z,
                        desired_state.orientation_acceleration_in_body_coordinates.z,
                        desired_state.orientation_jerk_in_body_coordinates.z,
                        desired_state.drive_velocity_in_module_coordinates.x,
                        desired_state.drive_acceleration_in_module_coordinates.x,
                        desired_state.drive_jerk_in_module_coordinates.x,
                    )
                )

            controller.on_state_update(current_drive_states)

    # Now draw all the graphs
    plot_trajectories(
        motion_set.description,
        motion_set.name,
        output_directory,
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