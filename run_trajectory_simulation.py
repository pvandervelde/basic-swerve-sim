from abc import ABC, abstractmethod
import argparse
from math import cos, isclose, isinf, pi, radians, sin, sqrt
from os import makedirs, path
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.offline as py
from random import random
from typing import List, Mapping, NamedTuple, Tuple

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
    x_pos: int = -1
    y_pos: int = -1

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
    icr_coordinate_map: List[List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]],
    color: str,
    ) -> List[List[ProfilePlotValues]]:
    default_size = 2

    plots: List[List[ProfilePlotValues]] = []

    module_index = 0
    for drive_module in drive_modules:
        sub_plots: List[ProfilePlotValues] = []
        plots.append(sub_plots)

        sub_plots.append(
            ProfilePlotValues(
                name="{} drive velocity".format(drive_module.name),
                markers=dict(color = color, size=default_size),
                x_values=points_in_time,
                y_values=[d[module_index].drive_velocity_in_module_coordinates.x for d in drive_states]
            )
        )

        # sub_plots.append(
        #     ProfilePlotValues(
        #         name="{} drive acceleration".format(drive_module.name),
        #         markers=dict(color = color, size=default_size),
        #         x_values=points_in_time,
        #         y_values=[d[module_index].drive_acceleration_in_module_coordinates.x for d in drive_states]
        #     )
        # )

        # sub_plots.append(
        #     ProfilePlotValues(
        #         name="{} drive jerk".format(drive_module.name),
        #         markers=dict(color = color, size=default_size),
        #         x_values=points_in_time,
        #         y_values=[d[module_index].drive_jerk_in_module_coordinates.x for d in drive_states]
        #     )
        # )

        sub_plots.append(
            ProfilePlotValues(
                name="{} drive orientation".format(drive_module.name),
                markers=dict(color = color, size=default_size),
                x_values=points_in_time,
                y_values=[d[module_index].orientation_in_body_coordinates.z for d in drive_states]
            )
        )

        # sub_plots.append(
        #     ProfilePlotValues(
        #         name="{} drive orientation velocity".format(drive_module.name),
        #         markers=dict(color = color, size=default_size),
        #         x_values=points_in_time,
        #         y_values=[d[module_index].orientation_velocity_in_body_coordinates.z for d in drive_states]
        #     )
        # )

        # sub_plots.append(
        #     ProfilePlotValues(
        #         name="{} drive orientation acceleration".format(drive_module.name),
        #         markers=dict(color = color, size=default_size),
        #         x_values=points_in_time,
        #         y_values=[d[module_index].orientation_acceleration_in_body_coordinates.z for d in drive_states]
        #     )
        # )

        # sub_plots.append(
        #     ProfilePlotValues(
        #         name="{} drive orientation jerk".format(drive_module.name),
        #         markers=dict(color = color, size=default_size),
        #         x_values=points_in_time,
        #         y_values=[d[module_index].orientation_jerk_in_body_coordinates.z for d in drive_states]
        #     )
        # )

        # Plot the ICR
        # There are 4 wheels, so there are 3 ICR combinations for each wheel.
        x_values: List[List[float]] = [
            [],
            [],
            [],
        ]
        y_values: List[List[float]] = [
            [],
            [],
            [],
        ]
        for icrs_at_time in icr_coordinate_map:
            index = 0
            for icr in icrs_at_time:
                should_plot = False

                if icr[0].name == drive_modules[module_index].name:
                    if not isclose(icr[0].drive_velocity_in_module_coordinates.x, 0.0, abs_tol=1e-6, rel_tol=1e-6):
                        should_plot = True
                if icr[1].name == drive_modules[module_index].name:
                    if not isclose(icr[1].drive_velocity_in_module_coordinates.x, 0.0, abs_tol=1e-6, rel_tol=1e-6):
                        should_plot = True

                if should_plot:
                    if not isinf(icr[2].x) and not isinf(icr[2].y):

                        if abs(icr[2].x) < 25 and abs(icr[2].y) < 25:
                            x_values[index].append(icr[2].x)
                            y_values[index].append(icr[2].y)

                    index += 1

        if len(x_values) > 0:
            sub_plots.append(
                ProfilePlotValues(
                    name="icr position",
                    markers=dict(color = color, size=default_size),
                    x_values=x_values[0],
                    y_values=y_values[0],
                    x_pos=module_index + 1,
                    y_pos=5
                )
            )

            sub_plots.append(
                ProfilePlotValues(
                    name="icr position",
                    markers=dict(color = "light{}".format(color), size=default_size),
                    x_values=x_values[1],
                    y_values=y_values[1],
                    x_pos=module_index + 1,
                    y_pos=5
                )
            )

            sub_plots.append(
                ProfilePlotValues(
                    name="icr position",
                    markers=dict(color = "dark{}".format(color), size=default_size),
                    x_values=x_values[2],
                    y_values=y_values[2],
                    x_pos=module_index + 1,
                    y_pos=5
                )
            )
        else:
            sub_plots.append(
                ProfilePlotValues(
                    name="icr position",
                    markers=dict(color = color, size=default_size),
                    x_values=[],
                    y_values=[]
                )
            )

        module_index += 1

    # Body position
    plots[0].insert(
        0,
        ProfilePlotValues(
            name="body position",
            markers=dict(color = color, size=default_size),
            x_values=[b.position_in_world_coordinates.x for b in body_states],
            y_values=[b.position_in_world_coordinates.y for b in body_states]
        )
    )

    plots[1].insert(
        0,
        ProfilePlotValues(
            name="body x-position",
            markers=dict(color = color, size=default_size),
            x_values=points_in_time,
            y_values=[b.position_in_world_coordinates.x for b in body_states]
        )
    )

    plots[2].insert(
        0,
        ProfilePlotValues(
            name="body y-position",
            markers=dict(color = color, size=default_size),
            x_values=points_in_time,
            y_values=[b.position_in_world_coordinates.y for b in body_states]
        )
    )

    plots[3].insert(
        0,
        ProfilePlotValues(
            name="body orientation",
            markers=dict(color = color, size=default_size),
            x_values=points_in_time,
            y_values=[b.orientation_in_world_coordinates.z for b in body_states]
        )
    )

    # Body velocity
    plots[0].insert(
        1,
        ProfilePlotValues(
            name="body x-velocity",
            markers=dict(color = color, size=default_size),
            x_values=points_in_time,
            y_values=[b.motion_in_body_coordinates.linear_velocity.x for b in body_states]
        )
    )

    plots[1].insert(
        1,
        ProfilePlotValues(
            name="body y-velocity",
            markers=dict(color = color, size=default_size),
            x_values=points_in_time,
            y_values=[b.motion_in_body_coordinates.linear_velocity.y for b in body_states]
        )
    )

    plots[2].insert(
        1,
        ProfilePlotValues(
            name="body rotation-velocity",
            markers=dict(color = color, size=default_size),
            x_values=points_in_time,
            y_values=[b.motion_in_body_coordinates.angular_velocity.z for b in body_states]
        )
    )

    plots[3].insert(
        1,
        ProfilePlotValues(
            name="empty-on-purpose",
            markers=dict(color = color, size=default_size),
            x_values=points_in_time,
            y_values=[]
        )
    )

    return plots

def generate_plot_traces(fig: go.Figure, plots: List[List[ProfilePlotValues]]):
    col_index = 1
    for lists in plots:

        row_index = 1
        for values in lists:
            print("Appending plot [{}, {}] with title [{}] ...".format(row_index, col_index, values.name))
            col = values.x_pos if values.x_pos > -1 else col_index
            row = values.y_pos if values.y_pos > -1 else row_index
            fig.append_trace(
                    go.Scatter(
                        x=values.x_values,
                        y=values.y_values,
                        legendgroup="a",
                        legendgrouptitle_text="a-title",
                        mode='markers',
                        marker=values.markers,
                        name=values.name,
                        showlegend=True,
                    ),
                    row=row,
                    col=col)
            fig.update_yaxes(title_text=values.name, row=row, col=col)
            row_index += 1

        col_index += 1

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

def get_plot(rows: int, cols: int) -> go.Figure:
    fig = make_subplots(
        rows=rows,
        cols=cols,
        shared_xaxes=False,
        shared_yaxes=True,
        )

    return fig

def get_motions(drive_modules: List[DriveModule]) -> List[Tuple[str, BodyState, List[DriveModuleDesiredValues], List[MotionCommand]]]:
    result: List[Tuple[str, BodyState, List[DriveModuleDesiredValues], List[MotionCommand]]] = []

    # result.append(
    #     (
    #         "0-degree forward from stand still",
    #         BodyState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #         [
    #             DriveModuleDesiredValues(drive_modules[0].name, 0.0,  0.0),
    #             DriveModuleDesiredValues(drive_modules[1].name, 0.0,  0.0),
    #             DriveModuleDesiredValues(drive_modules[2].name, 0.0,  0.0),
    #             DriveModuleDesiredValues(drive_modules[3].name, 0.0,  0.0),
    #         ],
    #         [
    #             BodyMotionCommand(1.0, 0.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "90-degree forward from stand still with rotation first",
    #         BodyState(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    #         [
    #             DriveModuleDesiredValues(drive_modules[0].name, 0.0,  0.0),
    #             DriveModuleDesiredValues(drive_modules[1].name, 0.0,  0.0),
    #             DriveModuleDesiredValues(drive_modules[2].name, 0.0,  0.0),
    #             DriveModuleDesiredValues(drive_modules[3].name, 0.0,  0.0),
    #         ],
    #         [
    #             DriveModuleMotionCommand([
    #                 DriveModuleDesiredValues(drive_modules[0].name, radians(90),  0.0),
    #                 DriveModuleDesiredValues(drive_modules[1].name, radians(90),  0.0),
    #                 DriveModuleDesiredValues(drive_modules[2].name, radians(90),  0.0),
    #                 DriveModuleDesiredValues(drive_modules[3].name, radians(90),  0.0)
    #             ]),
    #             BodyMotionCommand(0.0, 1.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "0-degree forwards to 90 degree forwards, without changing orientation",
    #         BodyState(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    #         [
    #             DriveModuleDesiredValues(drive_modules[0].name, 0.0,  1.0),
    #             DriveModuleDesiredValues(drive_modules[1].name, 0.0,  1.0),
    #             DriveModuleDesiredValues(drive_modules[2].name, 0.0,  1.0),
    #             DriveModuleDesiredValues(drive_modules[3].name, 0.0,  1.0),
    #         ],
    #         [
    #             BodyMotionCommand(0.0, 1.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "45-degree forward from stand still",
    #         BodyState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #         [
    #             DriveModuleProposedState(drive_modules[0].name, 0.0,  0.0),
    #             DriveModuleProposedState(drive_modules[1].name, 0.0,  0.0),
    #             DriveModuleProposedState(drive_modules[2].name, 0.0,  0.0),
    #             DriveModuleProposedState(drive_modules[3].name, 0.0,  0.0),
    #         ],
    #         [
    #             BodyMotionCommand(1.0, 1.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "0-degree stop from moving",
    #         BodyState(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    #         [
    #             DriveModuleProposedState(drive_modules[0].name, 0.0,  1.0),
    #             DriveModuleProposedState(drive_modules[1].name, 0.0,  1.0),
    #             DriveModuleProposedState(drive_modules[2].name, 0.0,  1.0),
    #             DriveModuleProposedState(drive_modules[3].name, 0.0,  1.0),
    #         ],
    #         [
    #             BodyMotionCommand(0.0, 0.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "90-degree stop from moving",
    #         BodyState(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    #         [
    #             DriveModuleProposedState(drive_modules[0].name, radians(90),  1.0),
    #             DriveModuleProposedState(drive_modules[1].name, radians(90),  1.0),
    #             DriveModuleProposedState(drive_modules[2].name, radians(90),  1.0),
    #             DriveModuleProposedState(drive_modules[3].name, radians(90),  1.0),
    #         ],
    #         [
    #             BodyMotionCommand(0.0, 0.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "45-degree stop from moving",
    #         BodyState(0.0, 0.0, 0.0, 1.0, 1.0, 0.0),
    #         [
    #             DriveModuleProposedState(drive_modules[0].name, radians(45),  sqrt(2)),
    #             DriveModuleProposedState(drive_modules[1].name, radians(45),  sqrt(2)),
    #             DriveModuleProposedState(drive_modules[2].name, radians(45),  sqrt(2)),
    #             DriveModuleProposedState(drive_modules[3].name, radians(45),  sqrt(2)),
    #         ],
    #         [
    #             BodyMotionCommand(0.0, 0.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "90-degree forwards to 0 degree forwards, without changing orientation",
    #         BodyState(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    #         [
    #             DriveModuleProposedState(drive_modules[0].name, radians(90),  1.0),
    #             DriveModuleProposedState(drive_modules[1].name, radians(90),  1.0),
    #             DriveModuleProposedState(drive_modules[2].name, radians(90),  1.0),
    #             DriveModuleProposedState(drive_modules[3].name, radians(90),  1.0),
    #         ],
    #         [
    #             BodyMotionCommand(1.0, 0.0, 0.0),
    #         ]
    #     )
    # )

    result.append(
        (
            "in place rotation from stand still",
            BodyState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            [
                DriveModuleDesiredValues(drive_modules[0].name, radians(0),  0.0),
                DriveModuleDesiredValues(drive_modules[1].name, radians(0),  0.0),
                DriveModuleDesiredValues(drive_modules[2].name, radians(0),  0.0),
                DriveModuleDesiredValues(drive_modules[3].name, radians(0),  0.0),
            ],
            [
                DriveModuleMotionCommand([
                    DriveModuleDesiredValues(drive_modules[0].name, radians(135),  0.0),
                    DriveModuleDesiredValues(drive_modules[1].name, radians(225),  0.0),
                    DriveModuleDesiredValues(drive_modules[2].name, radians(315),  0.0),
                    DriveModuleDesiredValues(drive_modules[3].name, radians(45),  0.0)
                ]),
                BodyMotionCommand(0.0, 0.0, 1.0),
            ]
        )
    )

    result.append(
        (
            "in place rotation from 45-degree forwards",
            BodyState(0.0, 0.0, 0.0, sqrt(0.5), sqrt(0.5), 0.0),
            [
                DriveModuleDesiredValues(drive_modules[0].name, radians(0),  0.0),
                DriveModuleDesiredValues(drive_modules[1].name, radians(0),  0.0),
                DriveModuleDesiredValues(drive_modules[2].name, radians(0),  0.0),
                DriveModuleDesiredValues(drive_modules[3].name, radians(0),  0.0),
            ],
            [
                DriveModuleMotionCommand([
                    DriveModuleDesiredValues(drive_modules[0].name, radians(45),  0.0),
                    DriveModuleDesiredValues(drive_modules[1].name, radians(45),  0.0),
                    DriveModuleDesiredValues(drive_modules[2].name, radians(45),  0.0),
                    DriveModuleDesiredValues(drive_modules[3].name, radians(45),  0.0)
                ]),
                BodyMotionCommand(sqrt(0.5), sqrt(0.5), 0.0),
                BodyMotionCommand(0.0, 0.0, 1.0),
            ]
        )
    )

    # result.append(
    #     (
    #         "circle without changing orientation from moving forwards at 0 degrees",
    #         BodyState(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    #         [
    #             DriveModuleDesiredValues(drive_modules[0].name, radians(0),  1.0),
    #             DriveModuleDesiredValues(drive_modules[1].name, radians(0),  1.0),
    #             DriveModuleDesiredValues(drive_modules[2].name, radians(0),  1.0),
    #             DriveModuleDesiredValues(drive_modules[3].name, radians(0),  1.0),
    #         ],
    #         [
    #             BodyMotionCommand(0.0, 1.0, 0.0),
    #             BodyMotionCommand(-1.0, 0.0, 0.0),
    #             BodyMotionCommand(0.0, -1.0, 0.0),
    #             BodyMotionCommand(1.0, 0.0, 0.0),
    #         ]
    #     )
    # )

    # result.append(
    #     (
    #         "circle while keeping the orientation tangentially to movement",
    #         BodyState(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    #         [
    #             DriveModuleDesiredValues(drive_modules[0].name, radians(0),  1.0),
    #             DriveModuleDesiredValues(drive_modules[1].name, radians(0),  1.0),
    #             DriveModuleDesiredValues(drive_modules[2].name, radians(0),  1.0),
    #             DriveModuleDesiredValues(drive_modules[3].name, radians(0),  1.0),
    #         ],
    #         [
    #             BodyMotionCommand(0.0, 1.0, 1.0),
    #             BodyMotionCommand(-1.0, 0.0, 1.0),
    #             BodyMotionCommand(0.0, -1.0, 1.0),
    #             BodyMotionCommand(1.0, 0.0, 1.0),
    #         ]
    #     )
    # )

    # # circle with orientation rotation counter circle

    # result.append(
    #     (
    #         "0-degrees forwards while rotating around center",
    #         BodyState(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    #         [
    #             DriveModuleProposedState(drive_modules[0].name, radians(0),  1.0),
    #             DriveModuleProposedState(drive_modules[1].name, radians(0),  1.0),
    #             DriveModuleProposedState(drive_modules[2].name, radians(0),  1.0),
    #             DriveModuleProposedState(drive_modules[3].name, radians(0),  1.0),
    #         ],
    #         [
    #             BodyMotionCommand(0.0, 1.0, 1.0),
    #             BodyMotionCommand(-1.0, 0.0, 1.0),
    #             BodyMotionCommand(0.0, -1.0, 1.0),
    #             BodyMotionCommand(1.0, 0.0, 1.0),
    #         ]
    #     )
    # )

    # forward + rotation
    # forward to in place rotation
    # Diagnoal to in place rotation


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

def plot_trajectories(
    fig: go.Figure,
    set_name: str,
    output_directory: str,
    points_in_time: List[float],
    body_states: List[BodyState],
    drive_modules: List[DriveModule],
    drive_states: List[List[DriveModuleMeasuredValues]],
    icr_coordinate_map: List[List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]],
    color: str,
    ):

    plots = generate_plot_information(points_in_time, body_states, drive_modules, drive_states, icr_coordinate_map, color)
    generate_plot_traces(fig, plots)

    # fig.update_traces(
    #     marker=dict(
    #         size=3,
    #         line=dict(
    #             width=1,
    #             color='DarkSlateGrey')),
    #     selector=dict(mode='markers'))
    fig.update_layout(
        template='ggplot2',
        title=set_name,
        width=2800,
        height=500 * len(plots[0]),
        showlegend=True,
        legend= {'itemsizing': 'constant'}
        )

def read_arguments() -> Mapping[str, any]:
    parser = argparse.ArgumentParser(
        description="Simulate a 4 wheel steering robot in 2D",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        "-o",
        "--output",
        action="store",
        required=True,
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

def simulation_align_drive_modules(
    sim_time_in_seconds: float,
    time_step_in_seconds: float,
    drive_modules: List[DriveModule],
    drive_module_states: List[DriveModuleMeasuredValues],
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

def simulation_run_trajectories(arg_dict: Mapping[str, any]):
    output_directory: str = arg_dict["output"]
    print("Running trajectory simulation")
    print("Outputting to {}".format(output_directory))

    drive_modules = get_drive_module_info()
    motions = get_motions(drive_modules)
    for motion_set in motions:
        simulation_run_trajectory(output_directory, motion_set[0], motion_set[1], drive_modules, motion_set[2], motion_set[3])

def simulation_run_trajectory(
    output_directory: str,
    set_name:str,
    body_state: BodyState,
    drive_modules: List[DriveModule],
    drive_module_proposed_states: List[DriveModuleDesiredValues],
    motion_set: List[MotionCommand]
    ):

    state_file_path = path.join(output_directory, "{}.csv".format(set_name))
    if not path.isdir(output_directory):
        print("Output directory {} does not exist. Creating directory ...".format(output_directory))
        makedirs(output_directory)

    print("Initializing state file at {}".format(state_file_path))
    initialize_state_file(state_file_path, len(drive_modules))

    # There are 10 variables we want to show (at the moment)
    # - body x-velocity
    # - body y-velocity
    # - body rotational velocity
    #
    # And for each module
    #
    # - module drive velocity
    # - module drive acceleration
    # - module drive jerk
    # - module drive orientation
    # - module drive orientation velocity
    # - module drive orientation acceleration
    # - module drive orientation jerk
    # - ICR
    plot_count = 3 + 5 # 7
    print("Creating figure with {} plots".format(plot_count))
    fig = get_plot(plot_count, 4)

    drive_module_states: List[DriveModuleMeasuredValues] = initialize_drive_modules(drive_modules, drive_module_proposed_states)

    controller = (list(get_controller(drive_modules).values()))[0]
    controller.on_state_update(drive_module_states)

    time_step_in_seconds = 0.01
    current_sim_time_in_seconds = 0.0

    # The motion set should be a command 'trajectory', i.e. a collection of ControlCommands with the
    # time span over which the command state should be achieved

    points_in_time: List[float] = [ 0.0 ]
    body_states: List[BodyState] = []
    drive_states: List[List[DriveModuleMeasuredValues]] = []
    icr_map: List[ List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]] = []

    for motion in motion_set:
        controller.on_desired_state_update(motion)

        for i in range(1, 101):
            controller.on_tick(current_sim_time_in_seconds)

            current_sim_time_in_seconds += time_step_in_seconds

            print("Processing step at {} ...".format(current_sim_time_in_seconds))

            points_in_time.append(current_sim_time_in_seconds)

            drive_module_states = controller.drive_module_state_at_future_time(current_sim_time_in_seconds)
            drive_states.append(drive_module_states)

            icr_coordinate_map = instantaneous_center_of_rotation_at_current_time(drive_module_states)
            icr_map.append(icr_coordinate_map)

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
    plot_trajectories(fig, set_name, output_directory, points_in_time, body_states, drive_modules, drive_states, icr_map, "blue")

    plot_file_path = path.join(output_directory, "{}.html".format(set_name))
    py.plot(fig, filename = plot_file_path)

def main(args=None):
    arg_dict = read_arguments()

    simulation_run_trajectories(arg_dict)

if __name__ == '__main__':
    main()