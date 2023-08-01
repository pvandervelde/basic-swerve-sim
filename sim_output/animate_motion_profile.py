
import math
from matplotlib import pyplot as plt
from matplotlib.animation import ArtistAnimation, FFMpegWriter, FuncAnimation, HTMLWriter, PillowWriter
from matplotlib.axes import Axes
from matplotlib.collections import PathCollection
import matplotlib.colors as mcolors
from matplotlib.figure import Figure, SubFigure
from matplotlib.gridspec import GridSpec, GridSpecBase
from matplotlib.lines import Line2D
import numpy as np
from typing import List, Mapping, Tuple

# local
from swerve_controller.control_model import DriveModuleMeasuredValues
from swerve_controller.drive_module import DriveModule
from swerve_controller.geometry import Point
from swerve_controller.states import BodyState

plt.rcParams['animation.ffmpeg_path'] = 'ffmpeg'

class AnimationData(object):
    def __init__(
            self,
            points_in_time: List[float],
            positions: List[List[float]],
            velocities: List[List[float]],
            accelerations: List[List[float]],
            jerks: List[List[float]],
        ):
        self.points_in_time = points_in_time
        self.positions = positions
        self.velocities = velocities
        self.accelerations = accelerations
        self.jerks = jerks

class AnimatedPlots(object):
    def __init__(
            self,
            ax_combined_positions: Axes,
            ax_combined_velocities: Axes,
            ax_combined_accelerations: Axes,
            ax_combined_jerks: Axes,
            ax_individual_positions: List[Tuple[str, Axes]],
            ax_individual_velocities: List[Tuple[str, Axes]],
            ax_individual_accelerations: List[Tuple[str, Axes]],
            ax_individual_jerks: List[Tuple[str, Axes]],
            ):

        self.ax_combined_positions = ax_combined_positions
        self.ax_combined_velocities = ax_combined_velocities
        self.ax_combined_accelerations = ax_combined_accelerations
        self.ax_combined_jerks = ax_combined_jerks

        self.ax_individual_positions = ax_individual_positions
        self.ax_individual_velocities = ax_individual_velocities
        self.ax_individual_accelerations = ax_individual_accelerations
        self.ax_individual_jerks = ax_individual_jerks

        self.combined_positions: List[Line2D] = []
        self.combined_velocities: List[Line2D] = []
        self.combined_accelerations: List[Line2D] = []
        self.combined_jerks: List[Line2D] = []

        self.individual_positions: List[Line2D] = []
        self.individual_velocities: List[Line2D] = []
        self.individual_accelerations: List[Line2D] = []
        self.individual_jerks: List[Line2D] = []

        for index, pair in enumerate(ax_individual_positions):
            name = pair[0]
            axes = pair[1]
            color_name = motion_profile_colors[index]
            self.combined_positions.append(ax_combined_positions.plot([], [], lw=2.5, color=color_name, label=name)[0])
            self.individual_positions.append(axes.plot([], [], lw=2.5, color=color_name, label=name)[0])

        for index, pair in enumerate(ax_individual_velocities):
            name = pair[0]
            axes = pair[1]
            color_name = motion_profile_colors[index]
            self.combined_velocities.append(ax_combined_velocities.plot([], [], lw=2.5, color=color_name, label=name)[0])
            self.individual_velocities.append(axes.plot([], [], lw=2.5, color=color_name, label=name)[0])

        for index, pair in enumerate(ax_individual_accelerations):
            name = pair[0]
            axes = pair[1]
            color_name = motion_profile_colors[index]
            self.combined_accelerations.append(ax_combined_accelerations.plot([], [], lw=2.5, color=color_name, label=name)[0])
            self.individual_accelerations.append(axes.plot([], [], lw=2.5, color=color_name, label=name)[0])

        for index, pair in enumerate(ax_individual_jerks):
            name = pair[0]
            axes = pair[1]
            color_name = motion_profile_colors[index]
            self.combined_jerks.append(ax_combined_jerks.plot([], [], lw=2.5, color=color_name, label=name)[0])
            self.individual_jerks.append(axes.plot([], [], lw=2.5, color=color_name, label=name)[0])

    def legend_refresh(self):
        self.ax_combined_positions.legend(loc="upper right")
        self.ax_combined_velocities.legend(loc="upper right")
        self.ax_combined_accelerations.legend(loc="upper right")
        self.ax_combined_jerks.legend(loc="upper right")

        for pair in self.ax_individual_positions:
            axes = pair[1]
            axes.legend(loc="upper right")

        for pair in self.ax_individual_velocities:
            axes = pair[1]
            axes.legend(loc="upper right")

        for pair in self.ax_individual_accelerations:
            axes = pair[1]
            axes.legend(loc="upper right")

        for pair in self.ax_individual_jerks:
            axes = pair[1]
            axes.legend(loc="upper right")

ANIMATION_FRAME_DIVIDER: int = 1
PLOT_AXIS_BUFFER_PERCENT: float = 0.1
PLOT_TITLE_FONT_SIZE: int = 10
PLOT_AXIS_FONT_SIZE: int = 8

animation_data: AnimationData = None
animated_plots: AnimatedPlots = None

motion_profile_colors: List[str] = [
    "darkorange",
    "green",
    "blue",
    "purple"
]

def animate(time_index: int):
    positions = [i[time_index * ANIMATION_FRAME_DIVIDER] for i in animation_data.positions]
    velocities = [i[time_index * ANIMATION_FRAME_DIVIDER] for i in animation_data.velocities]
    accelerations = [i[time_index * ANIMATION_FRAME_DIVIDER] for i in animation_data.accelerations]
    jerks = [i[time_index * ANIMATION_FRAME_DIVIDER] for i in animation_data.jerks]
    current_time = animation_data.points_in_time[time_index * ANIMATION_FRAME_DIVIDER]

    frames: List[Line2D] = []

    graph_frames = create_graph_frames(
        current_time,
        positions,
        velocities,
        accelerations,
        jerks)
    frames.extend(graph_frames)

    return frames

def create_acceleration_plot(
        profile_name: str,
        profile_index: int,
        min: float,
        max: float,
        fig: Figure,
        grid: GridSpecBase,
        time_max: float) -> Axes:
    ax = fig.add_subplot(grid[profile_index, 2])

    ax.set_ylim(min - PLOT_AXIS_BUFFER_PERCENT * abs(min), max + PLOT_AXIS_BUFFER_PERCENT * abs(max))
    ax.set_xlim(0.0, time_max)

    ax.set_title('Acceleration for {} profile'.format(profile_name), fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Acceleration (m/s^2)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color='.25', zorder=-10)

    return ax

def create_graph_frames(
        current_time: float,
        positions: List[float],
        velocities: List[float],
        accelerations: List[float],
        jerks: List[float],
    ) -> List[Line2D]:

    plots: List[Line2D] = []

    update_plots(current_time, positions, animated_plots.combined_positions, animated_plots.individual_positions)
    update_plots(current_time, velocities, animated_plots.combined_velocities, animated_plots.individual_velocities)
    update_plots(current_time, accelerations, animated_plots.combined_accelerations, animated_plots.individual_accelerations)
    update_plots(current_time, jerks, animated_plots.combined_jerks, animated_plots.individual_jerks)

    plots.extend(animated_plots.combined_positions)
    plots.extend(animated_plots.combined_velocities)
    plots.extend(animated_plots.combined_accelerations)
    plots.extend(animated_plots.combined_jerks)

    plots.extend(animated_plots.individual_positions)
    plots.extend(animated_plots.individual_velocities)
    plots.extend(animated_plots.individual_accelerations)
    plots.extend(animated_plots.individual_jerks)

    animated_plots.legend_refresh()

    return plots

def create_jerk_plot(
        profile_name: str,
        profile_index: int,
        min: float,
        max: float,
        fig: Figure,
        grid: GridSpecBase,
        time_max: float) -> Axes:
    ax = fig.add_subplot(grid[profile_index, 3])

    ax.set_ylim(min - PLOT_AXIS_BUFFER_PERCENT * abs(min), max + PLOT_AXIS_BUFFER_PERCENT * abs(max))
    ax.set_xlim(0.0, time_max)

    ax.set_title('Jerk for {} profile'.format(profile_name), fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Jerk (m/s^3)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color='.25', zorder=-10)

    return ax

def create_position_plot(
        profile_name: str,
        profile_index: int,
        min: float,
        max: float,
        fig: Figure,
        grid: GridSpecBase,
        time_max: float) -> Axes:
    ax = fig.add_subplot(grid[profile_index, 0])

    ax.set_ylim(min - PLOT_AXIS_BUFFER_PERCENT * abs(min), max + PLOT_AXIS_BUFFER_PERCENT * abs(max))
    ax.set_xlim(0.0, time_max)

    ax.set_title('Position for {} profile'.format(profile_name), fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Position (m)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color='.25', zorder=-10)

    return ax

def create_velocity_plot(
        profile_name: str,
        profile_index: int,
        min: float,
        max: float,
        fig: Figure,
        grid: GridSpecBase,
        time_max: float) -> Axes:
    ax = fig.add_subplot(grid[profile_index, 1])

    ax.set_ylim(min - PLOT_AXIS_BUFFER_PERCENT * abs(min), max + PLOT_AXIS_BUFFER_PERCENT * abs(max))
    ax.set_xlim(0.0, time_max)

    ax.set_title('Velocity for {} profile'.format(profile_name), fontsize=PLOT_TITLE_FONT_SIZE)
    ax.set_xlabel("Time (s)", fontsize=PLOT_AXIS_FONT_SIZE)
    ax.set_ylabel("Velocity (m/s)", fontsize=PLOT_AXIS_FONT_SIZE)

    ax.grid(linestyle="--", linewidth=0.5, color='.25', zorder=-10)

    return ax

def find_min_and_max_in_matrix(matrix: List[List[float]]) -> Tuple[float, float]:
    max: float = -100
    min: float = 100
    for vector in matrix:
        for number in vector:
            if number < min:
                min = number

            if number > max:
                max = number

    return (min, max)

def find_min_and_max_in_vector(vector: List[float]) -> Tuple[float, float]:
    max: float = -100
    min: float = 100
    for number in vector:
        if number < min:
            min = number

        if number > max:
            max = number

    return (min, max)

def plot_profile(
        profiles: List[str],
        points_in_time: List[float],
        positions: List[List[float]],
        velocities: List[List[float]],
        accelerations: List[List[float]],
        jerks: List[List[float]],
        output_file_name_without_extension):
    fig = plt.figure(figsize=[25.0, 12.0], constrained_layout=True)
    main_grid = fig.add_gridspec(len(profiles) + 1, 4)

    ax_individual_positions: List[Tuple[str, Axes]] = []
    ax_individual_velocities: List[Tuple[str, Axes]] = []
    ax_individual_accelerations: List[Tuple[str, Axes]] = []
    ax_individual_jerks: List[Tuple[str, Axes]] = []

    time_max = points_in_time[-1]

    # Create the shared plots
    pos_min, pos_max = find_min_and_max_in_matrix(positions)
    ax_combined_positions = create_position_plot('all', 0, pos_min, pos_max, fig, main_grid, time_max)

    vel_min, vel_max = find_min_and_max_in_matrix(velocities)
    ax_combined_velocities = create_velocity_plot('all', 0, vel_min, vel_max, fig, main_grid, time_max)

    acc_min, acc_max = find_min_and_max_in_matrix(accelerations)
    ax_combined_accelerations = create_acceleration_plot('all', 0, acc_min, acc_max, fig, main_grid, time_max)

    jerk_min, jerk_max = find_min_and_max_in_matrix(jerks)
    ax_combined_jerks = create_jerk_plot('all', 0, jerk_min, jerk_max, fig, main_grid, time_max)

    for index, profile in enumerate(profiles):
        pos_min, pos_max = find_min_and_max_in_vector(positions[index])
        ax_individual_positions.append((profile, create_position_plot(profile, index + 1, pos_min, pos_max, fig, main_grid, time_max)))

        vel_min, vel_max = find_min_and_max_in_vector(velocities[index])
        ax_individual_velocities.append((profile, create_velocity_plot(profile, index + 1, vel_min, vel_max, fig, main_grid, time_max)))

        acc_min, acc_max = find_min_and_max_in_vector(accelerations[index])
        ax_individual_accelerations.append((profile, create_acceleration_plot(profile, index + 1, acc_min, acc_max, fig, main_grid, time_max)))

        jerk_min, jerk_max = find_min_and_max_in_vector(jerks[index])
        ax_individual_jerks.append((profile, create_jerk_plot(profile, index + 1, jerk_min, jerk_max, fig, main_grid, time_max)))

    global animation_data
    animation_data = AnimationData(
        points_in_time,
        positions,
        velocities,
        accelerations,
        jerks,
        )

    global animated_plots
    animated_plots = AnimatedPlots(
        ax_combined_positions,
        ax_combined_velocities,
        ax_combined_accelerations,
        ax_combined_jerks,
        ax_individual_positions,
        ax_individual_velocities,
        ax_individual_accelerations,
        ax_individual_jerks)

    #fig.tight_layout(pad=1.0)
    #main_grid.tight_layout(fig)
    animation = FuncAnimation(fig, animate, frames=range(len(points_in_time)//ANIMATION_FRAME_DIVIDER), interval=100, blit=True, repeat=True, repeat_delay=10)

    #writer = FFMpegWriter()
    #output_file_name = output_file_name_without_extension + ".mp4"

    #writer = PillowWriter(fps=25)

    writer = HTMLWriter(fps=10)
    output_file_name = output_file_name_without_extension + ".html"

    animation.save(output_file_name, writer=writer)

def update_plots(current_time: float, values: List[float], combined_values: List[Line2D], individual_values: List[Line2D]):
    for index, value in enumerate(values):
        # Combined positions
        data = combined_values[index].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        combined_plot_data: List[float] = list(data[1])
        combined_plot_data.append(value)

        combined_values[index].set_data(times, combined_plot_data)

        # Individual position
        data = individual_values[index].get_data()
        times: List[float] = list(data[0])
        times.append(current_time)

        individual_plot_data: List[float] = list(data[1])
        individual_plot_data.append(value)

        individual_values[index].set_data(times, individual_plot_data)
