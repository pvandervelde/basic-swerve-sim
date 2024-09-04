import math
from typing import List, Tuple

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation, HTMLWriter
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D

# local
from swerve_controller.control_model import DriveModuleMeasuredValues
from swerve_controller.drive_module import DriveModule
from swerve_controller.geometry import Point
from swerve_controller.states import BodyState

plt.rcParams["animation.ffmpeg_path"] = "ffmpeg"


class AnimationData(object):
    def __init__(
        self,
        ax_robot: Axes,
        ax_body_velocity: Axes,
        ax_body_acceleration: Axes,
        ax_body_jerk: Axes,
        ax_body_angular_velocity: Axes,
        ax_body_angular_acceleration: Axes,
        ax_body_angular_jerk: Axes,
        ax_module_orientation: Axes,
        ax_module_angular_velocity: Axes,
        ax_module_velocity: Axes,
        ax_module_acceleration: Axes,
        points_in_time: List[float],
        drive_modules: List[DriveModule],
        body_states: List[BodyState],
        drive_module_states: List[List[DriveModuleMeasuredValues]],
        icr_coordinate_map: List[
            Tuple[
                float,
                List[
                    Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]
                ],
            ]
        ],
    ):
        self.ax_robot = ax_robot
        self.ax_body_velocity = ax_body_velocity
        self.ax_body_acceleration = ax_body_acceleration
        self.ax_body_jerk = ax_body_jerk
        self.ax_body_angular_velocity = ax_body_angular_velocity
        self.ax_body_angular_acceleration = ax_body_angular_acceleration
        self.ax_body_angular_jerk = ax_body_angular_jerk
        self.ax_module_orientation = ax_module_orientation
        self.ax_module_angular_velocity = ax_module_angular_velocity
        self.ax_module_velocity = ax_module_velocity
        self.ax_module_acceleration = ax_module_acceleration
        self.points_in_time = points_in_time
        self.drive_modules = drive_modules
        self.body_states = body_states
        self.drive_module_states = drive_module_states
        self.icr_coordinate_map = icr_coordinate_map


class AnimatedRobot(object):
    def __init__(self, ax: Axes):
        self.robot_body: Line2D = ax.plot([], [], color=body_colors[0])[0]
        self.wheels: List[Line2D] = [
            ax.plot([], [], color=drive_module_colors[0])[0],
            ax.plot([], [], color=drive_module_colors[1])[0],
            ax.plot([], [], color=drive_module_colors[2])[0],
            ax.plot([], [], color=drive_module_colors[3])[0],
        ]
        self.icr_lines: List[Line2D] = [
            ax.plot(
                [],
                [],
                color=drive_module_colors[0],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
            ax.plot(
                [],
                [],
                color=drive_module_colors[0],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
            ax.plot(
                [],
                [],
                color=drive_module_colors[1],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
            ax.plot(
                [],
                [],
                color=drive_module_colors[1],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
            ax.plot(
                [],
                [],
                color=drive_module_colors[2],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
            ax.plot(
                [],
                [],
                color=drive_module_colors[2],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
            ax.plot(
                [],
                [],
                color=drive_module_colors[3],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
            ax.plot(
                [],
                [],
                color=drive_module_colors[3],
                dashes=[10, 5, 10, 5],
                linewidth=0.75,
            )[0],
        ]
        self.icr_points: List[Line2D] = [
            ax.plot([], [], "-ro")[0],
            ax.plot([], [], "-ro")[0],
            ax.plot([], [], "-ro")[0],
            ax.plot([], [], "-ro")[0],
            ax.plot([], [], "-ro")[0],
            ax.plot([], [], "-ro")[0],
        ]
        self.position: Line2D = ax.plot([], [], marker="*", markersize=2)[0]


class AnimatedPlots(object):
    def __init__(
        self,
        body_velocity: Axes,
        body_acceleration: Axes,
        body_jerk: Axes,
        body_angular_velocity: Axes,
        body_angular_acceleration: Axes,
        body_angular_jerk: Axes,
        module_orientation: Axes,
        module_orientation_velocity: Axes,
        module_orientation_acceleration: Axes,
        module_orientation_jerk: Axes,
        module_velocity: Axes,
        module_acceleration: Axes,
        module_jerk: Axes,
        drive_modules: List[DriveModule],
    ):
        self.ax_body_velocity = body_velocity
        self.ax_body_acceleration = body_acceleration
        self.ax_body_jerk = body_jerk

        self.ax_body_angular_velocity = body_angular_velocity
        self.ax_body_angular_acceleration = body_angular_acceleration
        self.ax_body_angular_jerk = body_angular_jerk

        self.ax_module_orientation = module_orientation
        self.ax_module_orientation_velocity = module_orientation_velocity
        self.ax_module_orientation_acceleration = module_orientation_acceleration
        self.ax_module_orientation_jerk = module_orientation_jerk
        self.ax_module_velocity = module_velocity
        self.ax_module_acceleration = module_acceleration
        self.ax_module_jerk = module_jerk

        (self.body_x_velocity,) = body_velocity.plot(
            [], [], lw=2.5, color=body_colors[1], label="x-velocity"
        )
        (self.body_y_velocity,) = body_velocity.plot(
            [], [], lw=2.5, color=body_colors[2], label="y-velocity"
        )

        (self.body_x_acceleration,) = body_acceleration.plot(
            [], [], lw=2.5, color=body_colors[1], label="x-acceleration"
        )
        (self.body_y_acceleration,) = body_acceleration.plot(
            [], [], lw=2.5, color=body_colors[2], label="y-acceleration"
        )

        (self.body_x_jerk,) = body_jerk.plot(
            [], [], lw=2.5, color=body_colors[1], label="x-jerk"
        )
        (self.body_y_jerk,) = body_jerk.plot(
            [], [], lw=2.5, color=body_colors[2], label="y-jerk"
        )

        (self.body_angular_velocity,) = body_angular_velocity.plot(
            [], [], lw=2.5, color=body_colors[0], label="rotation-velocity"
        )
        (self.body_angular_acceleration,) = body_angular_acceleration.plot(
            [], [], lw=2.5, color=body_colors[0], label="rotation-acceleration"
        )
        (self.body_angular_jerk,) = body_angular_jerk.plot(
            [], [], lw=2.5, color=body_colors[0], label="rotation-jerk"
        )

        self.module_orientation: List[Line2D] = []
        self.module_orientation_velocity: List[Line2D] = []
        self.module_orientation_acceleration: List[Line2D] = []
        self.module_orientation_jerk: List[Line2D] = []
        self.module_velocity: List[Line2D] = []
        self.module_acceleration: List[Line2D] = []
        self.module_jerk: List[Line2D] = []

        i: int = 0
        for drive_module in drive_modules:
            name = drive_module.name
            color_name = drive_module_colors[i]

            self.module_orientation.append(
                module_orientation.plot(
                    [0.0], [0.1], lw=2.5, color=color_name, label=name
                )[0]
            )
            self.module_orientation_velocity.append(
                module_orientation_velocity.plot(
                    [], [], lw=2.5, color=color_name, label=name
                )[0]
            )
            self.module_orientation_acceleration.append(
                module_orientation_acceleration.plot(
                    [], [], lw=2.5, color=color_name, label=name
                )[0]
            )
            self.module_orientation_jerk.append(
                module_orientation_jerk.plot(
                    [], [], lw=2.5, color=color_name, label=name
                )[0]
            )

            self.module_velocity.append(
                module_velocity.plot([], [], lw=2.5, color=color_name, label=name)[0]
            )
            self.module_acceleration.append(
                module_acceleration.plot([], [], lw=2.5, color=color_name, label=name)[
                    0
                ]
            )
            self.module_jerk.append(
                module_jerk.plot([], [], lw=2.5, color=color_name, label=name)[0]
            )

            i += 1

    def legend_refresh(self):
        self.ax_body_velocity.legend(loc="upper right")
        self.ax_body_acceleration.legend(loc="upper right")
        self.ax_body_jerk.legend(loc="upper right")
        self.ax_body_angular_velocity.legend(loc="upper right")
        self.ax_body_angular_acceleration.legend(loc="upper right")
        self.ax_body_angular_jerk.legend(loc="upper right")
        self.ax_module_orientation.legend(loc="upper right")
        self.ax_module_orientation_velocity.legend(loc="upper right")
        self.ax_module_orientation_acceleration.legend(loc="upper right")
        self.ax_module_orientation_jerk.legend(loc="upper right")
        self.ax_module_velocity.legend(loc="upper right")
        self.ax_module_acceleration.legend(loc="upper right")
        self.ax_module_jerk.legend(loc="upper right")


ANIMATION_FRAME_DIVIDER: int = 1
PLOT_TITLE_FONT_SIZE: int = 10
PLOT_AXIS_FONT_SIZE: int = 8

animation_data: AnimationData = None
animated_robot: AnimatedRobot = None
animated_plots: AnimatedPlots = None

body_colors: List[str] = [
    "orchid",
    "deepskyblue",
    "yellowgreen",
    "sandybrown",
]

drive_module_colors: List[str] = ["darkorange", "green", "blue", "purple"]

icr_colors: List[str] = ["orange", "lightgreen", "lightblue", "violet"]


def animate(time_index: int) -> List[Line2D]:
    drive_modules = animation_data.drive_modules
    body_states = animation_data.body_states
    drive_module_states = animation_data.drive_module_states
    icr_coordinate_map = animation_data.icr_coordinate_map
    current_time = animation_data.points_in_time[time_index * ANIMATION_FRAME_DIVIDER]

    frames: List[Line2D] = []

    robot_frames = create_robot_movement_frame(
        drive_modules,
        body_states[time_index * ANIMATION_FRAME_DIVIDER],
        drive_module_states[time_index * ANIMATION_FRAME_DIVIDER],
        icr_coordinate_map[time_index * ANIMATION_FRAME_DIVIDER],
    )
    frames.extend(robot_frames)

    graph_frames = create_graph_frames(
        current_time,
        drive_modules,
        body_states[time_index * ANIMATION_FRAME_DIVIDER],
        drive_module_states[time_index * ANIMATION_FRAME_DIVIDER],
    )
    frames.extend(graph_frames)

    return frames


def create_body_acceleration_plot(
    body_states: List[BodyState], fig: Figure, grid: GridSpec, time_max: float
) -> Axes:
    ax = fig.add_subplot(grid[0, 12:16])  ####

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.linear_acceleration.x < y_min:
            y_min = state.motion_in_body_coordinates.linear_acceleration.x

        if state.motion_in_body_coordinates.linear_acceleration.y < y_min:
            y_min = state.motion_in_body_coordinates.linear_acceleration.y

        if state.motion_in_body_coordinates.linear_acceleration.x > y_max:
            y_max = state.motion_in_body_coordinates.linear_acceleration.x

        if state.motion_in_body_coordinates.linear_acceleration.y > y_max:
            y_max = state.motion_in_body_coordinates.linear_acceleration.y

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Body acceleration", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Acceleration (m/s^2)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_body_jerk_plot(
    body_states: List[BodyState], fig: Figure, grid: GridSpec, time_max: float
) -> Axes:
    ax = fig.add_subplot(grid[0, 16:20])  ####

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.linear_jerk.x < y_min:
            y_min = state.motion_in_body_coordinates.linear_jerk.x

        if state.motion_in_body_coordinates.linear_jerk.y < y_min:
            y_min = state.motion_in_body_coordinates.linear_jerk.y

        if state.motion_in_body_coordinates.linear_jerk.x > y_max:
            y_max = state.motion_in_body_coordinates.linear_jerk.x

        if state.motion_in_body_coordinates.linear_jerk.y > y_max:
            y_max = state.motion_in_body_coordinates.linear_jerk.y

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Body jerk", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Jerk (m/s^3)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_body_angular_acceleration_plot(
    body_states: List[BodyState], fig: Figure, grid: GridSpec, time_max: float
) -> Axes:
    ax = fig.add_subplot(grid[1, 12:16])

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.angular_acceleration.z < y_min:
            y_min = state.motion_in_body_coordinates.angular_acceleration.z

        if state.motion_in_body_coordinates.angular_acceleration.z > y_max:
            y_max = state.motion_in_body_coordinates.angular_acceleration.z

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Body rotation acceleration", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Acceleration (rad/s^2)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_body_angular_jerk_plot(
    body_states: List[BodyState], fig: Figure, grid: GridSpec, time_max: float
) -> Axes:
    ax = fig.add_subplot(grid[1, 16:20])  ####

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.angular_jerk.z < y_min:
            y_min = state.motion_in_body_coordinates.angular_jerk.z

        if state.motion_in_body_coordinates.angular_jerk.z > y_max:
            y_max = state.motion_in_body_coordinates.angular_jerk.z

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Body rotation jerk", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Jerk (rad/s^3)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_body_angular_velocity_plot(
    body_states: List[BodyState], fig: Figure, grid: GridSpec, time_max: float
) -> Axes:
    ax = fig.add_subplot(grid[1, 8:12])

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.angular_velocity.z < y_min:
            y_min = state.motion_in_body_coordinates.angular_velocity.z

        if state.motion_in_body_coordinates.angular_velocity.z > y_max:
            y_max = state.motion_in_body_coordinates.angular_velocity.z

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Body angular velocity", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Velocity (rad/s)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_body_velocity_plot(
    body_states: List[BodyState], fig: Figure, grid: GridSpec, time_max: float
) -> Axes:
    ax = fig.add_subplot(grid[0, 8:12])

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.linear_velocity.x < y_min:
            y_min = state.motion_in_body_coordinates.linear_velocity.x

        if state.motion_in_body_coordinates.linear_velocity.y < y_min:
            y_min = state.motion_in_body_coordinates.linear_velocity.y

        if state.motion_in_body_coordinates.linear_velocity.x > y_max:
            y_max = state.motion_in_body_coordinates.linear_velocity.x

        if state.motion_in_body_coordinates.linear_velocity.y > y_max:
            y_max = state.motion_in_body_coordinates.linear_velocity.y

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Body velocity", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Velocity (m/s)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_graph_frames(
    current_time: float,
    drive_modules: List[DriveModule],
    body_state: BodyState,
    drive_module_states: List[DriveModuleMeasuredValues],
) -> List[Line2D]:  # pragma: no cover
    plots: List[Line2D] = []

    # Body x-velocity
    data = animated_plots.body_x_velocity.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    velocities: List[float] = list(data[1])
    velocities.append(body_state.motion_in_body_coordinates.linear_velocity.x)

    animated_plots.body_x_velocity.set_data(times, velocities)
    plots.append(animated_plots.body_x_velocity)

    # Body y-velocity
    data = animated_plots.body_y_velocity.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    velocities: List[float] = list(data[1])
    velocities.append(body_state.motion_in_body_coordinates.linear_velocity.y)

    animated_plots.body_y_velocity.set_data(times, velocities)
    plots.append(animated_plots.body_y_velocity)

    # Body x-acceleration
    data = animated_plots.body_x_acceleration.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    accelerations: List[float] = list(data[1])
    accelerations.append(body_state.motion_in_body_coordinates.linear_acceleration.x)

    animated_plots.body_x_acceleration.set_data(times, accelerations)
    plots.append(animated_plots.body_x_acceleration)

    # Body y-acceleration
    data = animated_plots.body_y_acceleration.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    accelerations: List[float] = list(data[1])
    accelerations.append(body_state.motion_in_body_coordinates.linear_acceleration.y)

    animated_plots.body_y_acceleration.set_data(times, accelerations)
    plots.append(animated_plots.body_y_acceleration)

    # Body x-jerk
    data = animated_plots.body_x_jerk.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    jerk: List[float] = list(data[1])
    jerk.append(body_state.motion_in_body_coordinates.linear_jerk.x)

    animated_plots.body_x_jerk.set_data(times, jerk)
    plots.append(animated_plots.body_x_jerk)

    # Body y-jerk
    data = animated_plots.body_y_jerk.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    jerk: List[float] = list(data[1])
    jerk.append(body_state.motion_in_body_coordinates.linear_jerk.y)

    animated_plots.body_y_jerk.set_data(times, jerk)
    plots.append(animated_plots.body_y_jerk)

    # body angular velocity
    data = animated_plots.body_angular_velocity.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    velocities: List[float] = list(data[1])
    velocities.append(body_state.motion_in_body_coordinates.angular_velocity.z)

    animated_plots.body_angular_velocity.set_data(times, velocities)
    plots.append(animated_plots.body_angular_velocity)

    # body angular acceleration
    data = animated_plots.body_angular_acceleration.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    accelerations: List[float] = list(data[1])
    accelerations.append(body_state.motion_in_body_coordinates.angular_acceleration.z)

    animated_plots.body_angular_acceleration.set_data(times, accelerations)
    plots.append(animated_plots.body_angular_acceleration)

    # body angular jerk
    data = animated_plots.body_angular_jerk.get_data()
    times: List[float] = list(data[0])
    times.append(current_time)

    jerk: List[float] = list(data[1])
    jerk.append(body_state.motion_in_body_coordinates.angular_jerk.z)

    animated_plots.body_angular_jerk.set_data(times, jerk)
    plots.append(animated_plots.body_angular_jerk)

    for i in range(len(drive_modules)):
        state = drive_module_states[i]

        # module orientation
        data = animated_plots.module_orientation[i].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        orientations: List[float] = list(data[1])

        orientations.append(state.orientation_in_body_coordinates.z)

        animated_plots.module_orientation[i].set_data(times, orientations)

        # module orientation velocity
        data = animated_plots.module_orientation_velocity[i].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        orientation_velocities: List[float] = list(data[1])

        orientation_velocities.append(state.orientation_velocity_in_body_coordinates.z)

        animated_plots.module_orientation_velocity[i].set_data(
            times, orientation_velocities
        )

        # module orientation acceleration
        data = animated_plots.module_orientation_acceleration[i].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        orientation_accelerations: List[float] = list(data[1])

        orientation_accelerations.append(
            state.orientation_acceleration_in_body_coordinates.z
        )

        animated_plots.module_orientation_acceleration[i].set_data(
            times, orientation_accelerations
        )

        # module orientation jerk
        data = animated_plots.module_orientation_jerk[i].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        orientation_jerk: List[float] = list(data[1])

        orientation_jerk.append(state.orientation_jerk_in_body_coordinates.z)

        animated_plots.module_orientation_jerk[i].set_data(times, orientation_jerk)

        # module velocity
        data = animated_plots.module_velocity[i].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        velocities: List[float] = list(data[1])

        velocities.append(state.drive_velocity_in_module_coordinates.x)

        animated_plots.module_velocity[i].set_data(times, velocities)

        # module acceleration
        data = animated_plots.module_acceleration[i].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        accelerations: List[float] = list(data[1])

        accelerations.append(state.drive_acceleration_in_module_coordinates.x)

        animated_plots.module_acceleration[i].set_data(times, accelerations)

        # module acceleration
        data = animated_plots.module_jerk[i].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        jerk: List[float] = list(data[1])

        jerk.append(state.drive_jerk_in_module_coordinates.x)

        animated_plots.module_jerk[i].set_data(times, jerk)

    plots.extend(animated_plots.module_orientation)
    plots.extend(animated_plots.module_orientation_velocity)
    plots.extend(animated_plots.module_orientation_acceleration)
    plots.extend(animated_plots.module_orientation_jerk)
    plots.extend(animated_plots.module_velocity)
    plots.extend(animated_plots.module_acceleration)
    plots.extend(animated_plots.module_jerk)

    animated_plots.legend_refresh()

    return plots


def create_module_acceleration_plot(
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    fig: Figure,
    grid: GridSpec,
    time_max: float,
) -> Axes:
    ax = fig.add_subplot(grid[2, 12:16])

    y_max: float = -100
    y_min: float = 100
    for states in drive_module_states:
        for state in states:
            if state.drive_acceleration_in_module_coordinates.x < y_min:
                y_min = state.drive_acceleration_in_module_coordinates.x

            if state.drive_acceleration_in_module_coordinates.x > y_max:
                y_max = state.drive_acceleration_in_module_coordinates.x

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Wheel acceleration", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Acceleration (m/s^2)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_module_jerk_plot(
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    fig: Figure,
    grid: GridSpec,
    time_max: float,
) -> Axes:
    ax = fig.add_subplot(grid[2, 16:20])

    y_max: float = -100
    y_min: float = 100
    for states in drive_module_states:
        for state in states:
            if state.drive_jerk_in_module_coordinates.x < y_min:
                y_min = state.drive_jerk_in_module_coordinates.x

            if state.drive_jerk_in_module_coordinates.x > y_max:
                y_max = state.drive_jerk_in_module_coordinates.x

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Wheel jerk", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Jerk (m/s^3)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_module_orientation_acceleration_plot(
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    fig: Figure,
    grid: GridSpec,
    time_max: float,
) -> Axes:
    ax = fig.add_subplot(grid[0, 2])

    y_max: float = -100
    y_min: float = 100
    for states in drive_module_states:
        for state in states:
            if state.orientation_acceleration_in_body_coordinates.z < y_min:
                y_min = state.orientation_acceleration_in_body_coordinates.z

            if state.orientation_acceleration_in_body_coordinates.z > y_max:
                y_max = state.orientation_acceleration_in_body_coordinates.z

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Steering angle acceleration", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Acceleration (rad/s^2)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_module_orientation_jerk_plot(
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    fig: Figure,
    grid: GridSpec,
    time_max: float,
) -> Axes:
    ax = fig.add_subplot(grid[0, 3])

    y_max: float = -100
    y_min: float = 100
    for states in drive_module_states:
        for state in states:
            if state.orientation_jerk_in_body_coordinates.z < y_min:
                y_min = state.orientation_jerk_in_body_coordinates.z

            if state.orientation_jerk_in_body_coordinates.z > y_max:
                y_max = state.orientation_jerk_in_body_coordinates.z

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Steering angle jerk", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Jerk (rad/s^3)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_module_orientation_plot(
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    fig: Figure,
    grid: GridSpec,
    time_max: float,
) -> Axes:
    ax = fig.add_subplot(grid[0, 0])

    y_max: float = -100
    y_min: float = 100
    for states in drive_module_states:
        for state in states:
            if state.orientation_in_body_coordinates.z < y_min:
                y_min = state.orientation_in_body_coordinates.z

            if state.orientation_in_body_coordinates.z > y_max:
                y_max = state.orientation_in_body_coordinates.z

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Steering angle", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Orientation (rad)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_module_orientation_velocity_plot(
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    fig: Figure,
    grid: GridSpec,
    time_max: float,
) -> Axes:
    ax = fig.add_subplot(grid[0, 1])

    y_max: float = -100
    y_min: float = 100
    for states in drive_module_states:
        for state in states:
            if state.orientation_velocity_in_body_coordinates.z < y_min:
                y_min = state.orientation_velocity_in_body_coordinates.z

            if state.orientation_velocity_in_body_coordinates.z > y_max:
                y_max = state.orientation_velocity_in_body_coordinates.z

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Steering angle velocity", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Velocity (rad/s)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_module_velocity_plot(
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    fig: Figure,
    grid: GridSpec,
    time_max: float,
) -> Axes:
    ax = fig.add_subplot(grid[2, 8:12])

    y_max: float = -100
    y_min: float = 100
    for states in drive_module_states:
        for state in states:
            if state.drive_velocity_in_module_coordinates.x < y_min:
                y_min = state.drive_velocity_in_module_coordinates.x

            if state.drive_velocity_in_module_coordinates.x > y_max:
                y_max = state.drive_velocity_in_module_coordinates.x

    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.set_xlim(0.0, time_max)

    ax.set_title("Wheel velocity", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Velocity (m/s)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def create_robot_movement_frame(
    drive_modules: List[DriveModule],
    body_state: BodyState,
    drive_module_states: List[DriveModuleMeasuredValues],
    icr_coordinate_map: Tuple[
        float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]
    ],
    wheel_color="-r",
    truck_color="-k",
) -> List[Line2D]:  # pragma: no cover
    # Compute the rotation matrix for the body orientation
    body_rotation_matrix = np.array(
        [
            [
                math.cos(body_state.orientation_in_world_coordinates.z),
                math.sin(body_state.orientation_in_world_coordinates.z),
            ],
            [
                -math.sin(body_state.orientation_in_world_coordinates.z),
                math.cos(body_state.orientation_in_world_coordinates.z),
            ],
        ]
    )

    # There is no layout for the robot body (because the calculations don't need it at the moment) so
    # the size of the body is based on the distances between the drive modules.
    # We assume that each drive module is on one of the corners of the robot body, e.g. like this
    #
    #
    #       ___  _________  ___
    #       | |  |       |  | |
    #       ---  |       |  ---
    #            |       |
    #            |       |
    #            |       |
    #       ___  |       |  ___
    #       | |  |       |  | |
    #       ---  ---------  ---
    #
    left_front_x: float = max(
        (drive_module.steering_axis_xy_position.x + drive_module.wheel_radius)
        for drive_module in drive_modules
    )
    left_front_y: float = max(
        (
            drive_module.steering_axis_xy_position.y
            - (0.5 * drive_module.wheel_width + drive_module.wheel_radius)
        )
        for drive_module in drive_modules
    )
    right_rear_x: float = min(
        (drive_module.steering_axis_xy_position.x - drive_module.wheel_radius)
        for drive_module in drive_modules
    )
    right_rear_y: float = min(
        (
            drive_module.steering_axis_xy_position.x
            + (0.5 * drive_module.wheel_width + drive_module.wheel_radius)
        )
        for drive_module in drive_modules
    )

    # The outline is one array of x-coordinates starting at the left-front, going counter clock-wise, and ending at the left-front, and
    # one array of y-coordinates
    body_outline = np.array(
        [
            [left_front_x, right_rear_x, right_rear_x, left_front_x, left_front_x],
            [left_front_y, left_front_y, right_rear_y, right_rear_y, left_front_y],
        ]
    )

    # Rotate the body to the correct orientation
    body_outline = (body_outline.T.dot(body_rotation_matrix)).T

    # Translate the body to the position
    body_outline[0, :] += body_state.position_in_world_coordinates.x
    body_outline[1, :] += body_state.position_in_world_coordinates.y

    #
    # DRIVE MODULES
    #

    wheels: List[np.array] = []
    icrs: List[np.array] = []
    for i in range(len(drive_modules)):
        drive_module = drive_modules[i]
        drive_module_state = drive_module_states[i]

        drive_module_rotation_matrix = np.array(
            [
                [
                    math.cos(drive_module_state.orientation_in_body_coordinates.z),
                    math.sin(drive_module_state.orientation_in_body_coordinates.z),
                ],
                [
                    -math.sin(drive_module_state.orientation_in_body_coordinates.z),
                    math.cos(drive_module_state.orientation_in_body_coordinates.z),
                ],
            ]
        )

        wheel = np.array(
            [
                # x-coordinates of the corners of the shape, starting on the top left, moving counter-clockwise
                [
                    drive_module.wheel_radius,
                    -drive_module.wheel_radius,
                    -drive_module.wheel_radius,
                    drive_module.wheel_radius,
                    drive_module.wheel_radius,
                ],
                # y-coordinates of the corners of the shape
                [
                    0.5 * drive_module.wheel_width,
                    0.5 * drive_module.wheel_width,
                    -0.5 * drive_module.wheel_width,
                    -0.5 * drive_module.wheel_width,
                    0.5 * drive_module.wheel_width,
                ],
            ]
        )

        icr_line_1 = np.array([[0.0, 0.0], [0.5 * drive_module.wheel_width, 25.0]])

        icr_line_2 = np.array([[0.0, 0.0], [0.5 * drive_module.wheel_width, -25.0]])

        # Rotate the wheel to the drive module orientation
        wheel = (wheel.T.dot(drive_module_rotation_matrix)).T
        icr_line_1 = (icr_line_1.T.dot(drive_module_rotation_matrix)).T
        icr_line_2 = (icr_line_2.T.dot(drive_module_rotation_matrix)).T

        # Translate the wheel to the body, with the body at (0, 0)
        wheel[0, :] += drive_module.steering_axis_xy_position.x
        wheel[1, :] += drive_module.steering_axis_xy_position.y

        icr_line_1[0, :] += drive_module.steering_axis_xy_position.x
        icr_line_1[1, :] += drive_module.steering_axis_xy_position.y

        icr_line_2[0, :] += drive_module.steering_axis_xy_position.x
        icr_line_2[1, :] += drive_module.steering_axis_xy_position.y

        # Rotate the wheel to match the body orientation
        wheel = (wheel.T.dot(body_rotation_matrix)).T
        icr_line_1 = (icr_line_1.T.dot(body_rotation_matrix)).T
        icr_line_2 = (icr_line_2.T.dot(body_rotation_matrix)).T

        # Translate the wheel to the actual body coordinates
        wheel[0, :] += body_state.position_in_world_coordinates.x
        wheel[1, :] += body_state.position_in_world_coordinates.y

        icr_line_1[0, :] += body_state.position_in_world_coordinates.x
        icr_line_1[1, :] += body_state.position_in_world_coordinates.y

        icr_line_2[0, :] += body_state.position_in_world_coordinates.x
        icr_line_2[1, :] += body_state.position_in_world_coordinates.y

        # Store the wheel outline information
        wheels.append(wheel)
        icrs.append(icr_line_1)
        icrs.append(icr_line_2)

    plots: List[Line2D] = []
    animated_robot.robot_body.set_data(
        np.array(body_outline[0, :]).flatten(), np.array(body_outline[1, :]).flatten()
    )
    plots.append(animated_robot.robot_body)

    for wheel_index in range(len(wheels)):
        wheel = wheels[wheel_index]
        animated_robot.wheels[wheel_index].set_data(
            np.array(wheel[0, :]).flatten(), np.array(wheel[1, :]).flatten()
        )
        plots.append(animated_robot.wheels[wheel_index])

    for icr_index in range(len(icrs)):
        icr_line = icrs[icr_index]
        animated_robot.icr_lines[icr_index].set_data(
            np.array(icr_line[0, :]).flatten(), np.array(icr_line[1, :]).flatten()
        )
        plots.append(animated_robot.icr_lines[icr_index])

    for icr_index in range(len(icr_coordinate_map[1])):
        _, _, icr_coordinate = icr_coordinate_map[1][icr_index]

        icr_point = np.array([[icr_coordinate.x], [icr_coordinate.y]])
        icr_point = (icr_point.T.dot(body_rotation_matrix)).T

        # Translate the wheel to the actual body coordinates
        icr_point[0, :] += body_state.position_in_world_coordinates.x
        icr_point[1, :] += body_state.position_in_world_coordinates.y

        animated_robot.icr_points[icr_index].set_data(
            np.array(icr_point[0, :]).flatten(), np.array(icr_point[1, :]).flatten()
        )
        plots.append(animated_robot.icr_points[icr_index])

    data = animated_robot.position.get_data()
    x_coordinates: List[float] = list(data[0])
    x_coordinates.append(body_state.position_in_world_coordinates.x)

    y_coordinates: List[float] = list(data[1])
    y_coordinates.append(body_state.position_in_world_coordinates.y)

    animated_robot.position.set_data(x_coordinates, y_coordinates)
    plots.append(animated_robot.position)

    return plots


def create_robot_plot(
    body_states: List[BodyState], fig: Figure, grid: GridSpec
) -> Axes:
    ax = fig.add_subplot(grid[0:3, 0:8])
    x_max: float = -100
    x_min: float = 100

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.position_in_world_coordinates.x < x_min:
            x_min = state.position_in_world_coordinates.x

        if state.position_in_world_coordinates.x > x_max:
            x_max = state.position_in_world_coordinates.x

        if state.position_in_world_coordinates.y < y_min:
            y_min = state.position_in_world_coordinates.y

        if state.position_in_world_coordinates.y > y_max:
            y_max = state.position_in_world_coordinates.y

    ax.set_ylim(y_min - 5, y_max + 5)
    ax.set_xlim(x_min - 5, x_max + 5)

    ax.set_xlabel("x-position (m)", fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_ylabel("y-position (m)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_title("Robot motion", fontsize=PLOT_TITLE_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color=".25", zorder=-10)

    return ax


def plot_movement_through_space(
    points_in_time: List[float],
    drive_modules: List[DriveModule],
    body_states: List[BodyState],
    drive_module_states: List[List[DriveModuleMeasuredValues]],
    icr_coordinate_map: List[
        Tuple[
            float,
            List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]],
        ]
    ],
    output_file_name_without_extension,
):
    fig = plt.figure(figsize=[25.0, 12.0], constrained_layout=True)
    # main_grid = fig.add_gridspec(4, 20)
    main_grid = fig.add_gridspec(2, 1, height_ratios=[3, 1])

    gs1 = main_grid[0].subgridspec(3, 20)
    gs2 = main_grid[1].subgridspec(1, 4)

    # Image of moving robot
    ax_robot = create_robot_plot(body_states, fig, gs1)

    # Robot body velocity and acceleration
    time_max: float = points_in_time[-1]

    ax_body_velocity = create_body_velocity_plot(body_states, fig, gs1, time_max)
    ax_body_acceleration = create_body_acceleration_plot(
        body_states, fig, gs1, time_max
    )
    ax_body_jerk = create_body_jerk_plot(body_states, fig, gs1, time_max)

    ax_body_angular_velocity = create_body_angular_velocity_plot(
        body_states, fig, gs1, time_max
    )
    ax_body_angular_acceleration = create_body_angular_acceleration_plot(
        body_states, fig, gs1, time_max
    )
    ax_body_angular_jerk = create_body_angular_jerk_plot(
        body_states, fig, gs1, time_max
    )

    # Module orientation and orientation velocity
    ax_module_orientation = create_module_orientation_plot(
        drive_module_states, fig, gs2, time_max
    )
    ax_module_angular_velocity = create_module_orientation_velocity_plot(
        drive_module_states, fig, gs2, time_max
    )
    ax_module_angular_acceleration = create_module_orientation_acceleration_plot(
        drive_module_states, fig, gs2, time_max
    )
    ax_module_angular_jerk = create_module_orientation_jerk_plot(
        drive_module_states, fig, gs2, time_max
    )

    # Module velocity and acceleration
    ax_module_velocity = create_module_velocity_plot(
        drive_module_states, fig, gs1, time_max
    )
    ax_module_acceleration = create_module_acceleration_plot(
        drive_module_states, fig, gs1, time_max
    )
    ax_module_jerk = create_module_jerk_plot(drive_module_states, fig, gs1, time_max)

    global animation_data
    animation_data = AnimationData(
        ax_robot,
        ax_body_velocity,
        ax_body_acceleration,
        ax_body_jerk,
        ax_body_angular_velocity,
        ax_body_angular_acceleration,
        ax_body_angular_jerk,
        ax_module_orientation,
        ax_module_angular_velocity,
        ax_module_velocity,
        ax_module_acceleration,
        points_in_time,
        drive_modules,
        body_states,
        drive_module_states,
        icr_coordinate_map,
    )

    global animated_robot
    animated_robot = AnimatedRobot(ax_robot)

    global animated_plots
    animated_plots = AnimatedPlots(
        ax_body_velocity,
        ax_body_acceleration,
        ax_body_jerk,
        ax_body_angular_velocity,
        ax_body_angular_acceleration,
        ax_body_angular_jerk,
        ax_module_orientation,
        ax_module_angular_velocity,
        ax_module_angular_acceleration,
        ax_module_angular_jerk,
        ax_module_velocity,
        ax_module_acceleration,
        ax_module_jerk,
        drive_modules,
    )

    # fig.tight_layout(pad=1.0)
    # main_grid.tight_layout(fig)
    animation = FuncAnimation(
        fig,
        animate,
        frames=range(len(points_in_time) // ANIMATION_FRAME_DIVIDER),
        interval=100,
        blit=True,
        repeat=True,
        repeat_delay=10,
    )

    # writer = FFMpegWriter()
    # output_file_name = output_file_name_without_extension + ".mp4"

    # writer = PillowWriter(fps=25)

    writer = HTMLWriter(fps=10)
    output_file_name = output_file_name_without_extension + ".html"

    animation.save(output_file_name, writer=writer)
    animation.save(output_file_name, writer=writer)
