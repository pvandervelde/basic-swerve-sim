
import math
from matplotlib import pyplot as plt
from matplotlib.animation import ArtistAnimation, FFMpegWriter, FuncAnimation, HTMLWriter, PillowWriter
from matplotlib.axes import Axes
from matplotlib.collections import PathCollection
from matplotlib.figure import Figure, SubFigure
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import numpy as np
from typing import List, Tuple

# local
from swerve_controller.control_model import DriveModuleMeasuredValues
from swerve_controller.drive_module import DriveModule
from swerve_controller.geometry import Point
from swerve_controller.states import BodyState

plt.rcParams['animation.ffmpeg_path'] = 'ffmpeg'

class AnimationData(object):
    def __init__(
            self,
            ax_robot: Axes,
            ax_body_velocity: Axes,
            ax_body_acceleration: Axes,
            ax_module_orientation: Axes,
            ax_module_angular_velocity: Axes,
            ax_module_velocity: Axes,
            ax_module_acceleration: Axes,
            points_in_time: List[float],
            drive_modules: List[DriveModule],
            body_states: List[BodyState],
            drive_module_states: List[List[DriveModuleMeasuredValues]],
            icr_coordinate_map: List[Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]]]):
        self.ax_robot = ax_robot
        self.ax_body_velocity = ax_body_velocity
        self.ax_body_acceleration = ax_body_acceleration
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
        self.robot_body: Line2D = ax.plot([], [], "-k")[0]
        self.wheels: List[Line2D] = [
            ax.plot([], [], "-r")[0],
            ax.plot([], [], "-r")[0],
            ax.plot([], [], "-r")[0],
            ax.plot([], [], "-r")[0],
        ]
        self.icr_lines: List[Line2D] = [
            ax.plot([], [], dashes=[10, 5, 10, 5])[0],
            ax.plot([], [], dashes=[10, 5, 10, 5])[0],
            ax.plot([], [], dashes=[10, 5, 10, 5])[0],
            ax.plot([], [], dashes=[10, 5, 10, 5])[0],
        ]
        self.icr_points: List[Line2D] = [
            ax.plot([], [], '-ro')[0],
            ax.plot([], [], '-ro')[0],
            ax.plot([], [], '-ro')[0],
            ax.plot([], [], '-ro')[0],
            ax.plot([], [], '-ro')[0],
            ax.plot([], [], '-ro')[0],
        ]
        self.position: Line2D = ax.plot([], [], marker="*", markersize=2)[0]

class AnimatedPlots(object):
    def __init__(
            self,
            body_velocity: Axes,
            body_acceleration: Axes,
            module_orientation: Axes,
            module_orientation_velocity: Axes,
            module_velocity: Axes,
            module_acceleration: Axes):
        self.body_velocity, = body_velocity.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none
        self.body_x_velocity, = body_velocity.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none
        self.body_y_velocity, = body_velocity.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none

        self.body_acceleration, = body_acceleration.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none
        self.body_x_acceleration, = body_acceleration.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none
        self.body_y_acceleration, = body_acceleration.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none

        self.module_orientation, = module_orientation.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none
        self.module_orientation_velocity = module_orientation_velocity.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none

        self.module_velocity, = module_velocity.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none
        self.module_acceleration = module_acceleration.plot([], [], marker="o", ls="", markersize=2) # set linestyle to none

animation_data: AnimationData = None
animated_robot: AnimatedRobot = None
animated_plots: AnimatedPlots = None

def animate(time_index: int):
    ax_robot = animation_data.ax_robot
    drive_modules = animation_data.drive_modules
    body_states = animation_data.body_states
    drive_module_states = animation_data.drive_module_states
    icr_coordinate_map = animation_data.icr_coordinate_map
    current_time = animation_data.points_in_time[time_index]

    frames: List[Line2D] = []

    robot_frames = create_robot_movement_frame(drive_modules, body_states[time_index * 2], drive_module_states[time_index * 2], icr_coordinate_map[time_index * 2])
    frames.extend(robot_frames)

    graph_frames = create_graph_frames(current_time, drive_modules, body_states[time_index * 2], drive_module_states[time_index * 2], icr_coordinate_map[time_index * 2])
    frames.extend(graph_frames)

    return robot_frames

def create_body_acceleration_plot(
        body_states: List[BodyState],
        fig: Figure,
        grid: GridSpec,
        time_max: float):
    ax_body_acceleration = fig.add_subplot(grid[0, 3], title='Body acceleration')

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.linear_velocity.x < y_min:
            y_min = state.motion_in_body_coordinates.linear_velocity.x

        if state.motion_in_body_coordinates.linear_velocity.x > y_max:
            y_max = state.motion_in_body_coordinates.linear_velocity.x

    ax_body_acceleration.set_ylim(y_min - 0.5, y_max + 0.5)
    ax_body_acceleration.set_xlim(0.0, time_max)
    return ax_body_acceleration

def create_body_velocity_plot(
        body_states: List[BodyState],
        fig: Figure,
        grid: GridSpec,
        time_max: float):
    ax_body_velocity = fig.add_subplot(grid[0, 2], title='Body velocity')

    y_max: float = -100
    y_min: float = 100
    for state in body_states:
        if state.motion_in_body_coordinates.linear_velocity.x < y_min:
            y_min = state.motion_in_body_coordinates.linear_velocity.x

        if state.motion_in_body_coordinates.linear_velocity.x > y_max:
            y_max = state.motion_in_body_coordinates.linear_velocity.x

    ax_body_velocity.set_ylim(y_min - 0.5, y_max + 0.5)
    ax_body_velocity.set_xlim(0.0, time_max)
    return ax_body_velocity

def create_graph_frames(
        current_time: float,
        drive_modules: List[DriveModule],
        body_state: BodyState,
        drive_module_states: List[DriveModuleMeasuredValues],
        icr_coordinate_map: Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]]) -> List[Line2D]:  # pragma: no cover

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

    # module orientation
    #data = animated_plots.module_orientation.get_data()
    #times: List[float] = list(data[0])
    #times.append(current_time)

    #velocities: List[float] = list(data[1])
    #velocities.append(body_state.motion_in_body_coordinates.linear_velocity.y)

    #animated_plots.module_orientation.set_data(times, velocities)
    #plots.append(animated_plots.module_orientation)


    # module orientation velocity

    # module velocity

    # module acceleration

    return plots

def create_robot_movement_frame(
        drive_modules: List[DriveModule],
        body_state: BodyState,
        drive_module_states: List[DriveModuleMeasuredValues],
        icr_coordinate_map: Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]],
        wheel_color="-r",
        truck_color="-k") -> List[Line2D]:  # pragma: no cover

    # Compute the rotation matrix for the body orientation
    body_rotation_matrix = np.array(
            [
                [math.cos(body_state.orientation_in_world_coordinates.z), math.sin(body_state.orientation_in_world_coordinates.z)],
                [-math.sin(body_state.orientation_in_world_coordinates.z), math.cos(body_state.orientation_in_world_coordinates.z)]
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
    left_front_x: float = max((drive_module.steering_axis_xy_position.x + drive_module.wheel_radius) for drive_module in drive_modules)
    left_front_y: float = max((drive_module.steering_axis_xy_position.y - (0.5 * drive_module.wheel_width + drive_module.wheel_radius)) for drive_module in drive_modules)
    right_rear_x: float = min((drive_module.steering_axis_xy_position.x - drive_module.wheel_radius) for drive_module in drive_modules)
    right_rear_y: float = min((drive_module.steering_axis_xy_position.x + (0.5 * drive_module.wheel_width + drive_module.wheel_radius)) for drive_module in drive_modules)

    # The outline is one array of x-coordinates starting at the left-front, going counter clock-wise, and ending at the left-front, and
    # one array of y-coordinates
    body_outline = np.array(
        [
            [left_front_x, right_rear_x, right_rear_x, left_front_x, left_front_x],
            [left_front_y, left_front_y, right_rear_y, right_rear_y, left_front_y]
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
                [math.cos(drive_module_state.orientation_in_body_coordinates.z), math.sin(drive_module_state.orientation_in_body_coordinates.z)],
                [-math.sin(drive_module_state.orientation_in_body_coordinates.z), math.cos(drive_module_state.orientation_in_body_coordinates.z)]
            ]
        )

        wheel = np.array(
            [
                # x-coordinates of the corners of the shape, starting on the top left, moving counter-clockwise
                [drive_module.wheel_radius, -drive_module.wheel_radius, -drive_module.wheel_radius, drive_module.wheel_radius, drive_module.wheel_radius],
                # y-coordinates of the corners of the shape
                [0.5 * drive_module.wheel_width, 0.5 * drive_module.wheel_width, -0.5 * drive_module.wheel_width, -0.5 * drive_module.wheel_width, 0.5 * drive_module.wheel_width]
            ]
        )

        icr_line = np.array(
            [
                [0.0, 0.0],
                [0.5 * drive_module.wheel_width, 25.0]
            ]
        )

        # Rotate the wheel to the drive module orientation
        wheel = (wheel.T.dot(drive_module_rotation_matrix)).T
        icr_line = (icr_line.T.dot(drive_module_rotation_matrix)).T

        # Translate the wheel to the body, with the body at (0, 0)
        wheel[0, :] += drive_module.steering_axis_xy_position.x
        wheel[1, :] += drive_module.steering_axis_xy_position.y

        icr_line[0, :] += drive_module.steering_axis_xy_position.x
        icr_line[1, :] += drive_module.steering_axis_xy_position.y

        # Rotate the wheel to match the body orientation
        wheel = (wheel.T.dot(body_rotation_matrix)).T
        icr_line = (icr_line.T.dot(body_rotation_matrix)).T

        # Translate the wheel to the actual body coordinates
        wheel[0, :] += body_state.position_in_world_coordinates.x
        wheel[1, :] += body_state.position_in_world_coordinates.y

        icr_line[0, :] += body_state.position_in_world_coordinates.x
        icr_line[1, :] += body_state.position_in_world_coordinates.y

        # Store the wheel outline information
        wheels.append(wheel)
        icrs.append(icr_line)

    plots: List[Line2D] = []
    animated_robot.robot_body.set_data(np.array(body_outline[0, :]).flatten(), np.array(body_outline[1, :]).flatten())
    plots.append(animated_robot.robot_body)

    for wheel_index in range(len(wheels)):
        wheel = wheels[wheel_index]
        animated_robot.wheels[wheel_index].set_data(np.array(wheel[0, :]).flatten(), np.array(wheel[1, :]).flatten())
        plots.append(animated_robot.wheels[wheel_index])

    for icr_index in range(len(icrs)):
        icr_line = icrs[icr_index]
        animated_robot.icr_lines[icr_index].set_data(np.array(icr_line[0, :]).flatten(), np.array(icr_line[1, :]).flatten())
        plots.append(animated_robot.icr_lines[icr_index])

    for icr_index in range(len(icr_coordinate_map[1])):
        _, _, icr_coordinate = icr_coordinate_map[1][icr_index]

        icr_point = np.array(
            [
                [icr_coordinate.x],
                [icr_coordinate.y]
            ]
        )
        icr_point = (icr_point.T.dot(body_rotation_matrix)).T

        # Translate the wheel to the actual body coordinates
        icr_point[0, :] += body_state.position_in_world_coordinates.x
        icr_point[1, :] += body_state.position_in_world_coordinates.y

        animated_robot.icr_points[icr_index].set_data(np.array(icr_point[0, :]).flatten(), np.array(icr_point[1, :]).flatten())
        plots.append(animated_robot.icr_points[icr_index])

    data = animated_robot.position.get_data()
    x_coordinates: List[float] = list(data[0])
    x_coordinates.append(body_state.position_in_world_coordinates.x)

    y_coordinates: List[float] = list(data[1])
    y_coordinates.append(body_state.position_in_world_coordinates.y)

    animated_robot.position.set_data(x_coordinates, y_coordinates)
    plots.append(animated_robot.position)

    return plots

def create_robot_plot(body_states: List[BodyState], fig: Figure, grid: GridSpec):
    ax_robot = fig.add_subplot(grid[:, :-2], title='Robot motion')
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

    ax_robot.set_ylim(y_min - 10, y_max + 10)
    ax_robot.set_xlim(x_min - 10, x_max + 10)

    return ax_robot

def plot_movement_through_space(
        points_in_time: List[float],
        drive_modules: List[DriveModule],
        body_states: List[BodyState],
        drive_module_states: List[List[DriveModuleMeasuredValues]],
        icr_coordinate_map: List[Tuple[float, List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]]],
        output_file_name):
    fig = plt.figure(figsize=[20.0, 10.0])
    grid = GridSpec(3, 4, figure=fig)

    # Image of moving robot
    ax_robot = create_robot_plot(body_states, fig, grid)

    # Robot body velocity and acceleration
    time_max: float = points_in_time[-1]

    ax_body_velocity = create_body_velocity_plot(body_states, fig, grid, time_max)

    ax_body_acceleration = create_body_acceleration_plot(body_states, fig, grid, time_max)

    # Module orientation and orientation velocity
    ax_module_orientation = fig.add_subplot(grid[1, 2], title='Module orientation')
    ax_module_angular_velocity = fig.add_subplot(grid[1, 3], title='Module orientation velocity')

    # Module velocity and acceleration
    ax_module_velocity = fig.add_subplot(grid[2, 2], title='Module velocity')
    ax_module_acceleration = fig.add_subplot(grid[2, 3], title='Module acceleration')

    global animation_data
    animation_data = AnimationData(
        ax_robot,
        ax_body_velocity,
        ax_body_acceleration,
        ax_module_orientation,
        ax_module_angular_velocity,
        ax_module_velocity,
        ax_module_acceleration,
        points_in_time,
        drive_modules,
        body_states,
        drive_module_states,
        icr_coordinate_map)

    global animated_robot
    animated_robot = AnimatedRobot(ax_robot)

    global animated_plots
    animated_plots = AnimatedPlots(
        ax_body_velocity,
        ax_body_acceleration,
        ax_module_orientation,
        ax_module_angular_velocity,
        ax_module_velocity,
        ax_module_acceleration)

    animation = FuncAnimation(fig, animate, frames=range(len(points_in_time)//2), interval=100, blit=True, repeat=True, repeat_delay=10)

    #writer = FFMpegWriter()
    #writer = PillowWriter(fps=25)
    writer = HTMLWriter(fps=10)
    animation.save(output_file_name, writer=writer)
