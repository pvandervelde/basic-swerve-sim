
import math
from matplotlib import pyplot as plt
from matplotlib.animation import ArtistAnimation, PillowWriter
from matplotlib.axes import Axes
from matplotlib.lines import Line2D
import numpy as np
from typing import List

# local
from swerve_controller.control_model import DriveModuleMeasuredValues
from swerve_controller.drive_module import DriveModule
from swerve_controller.states import BodyState


def plot_movement_through_space(points_in_time: List[float], drive_modules: List[DriveModule], body_states: List[BodyState], drive_module_states: List[List[DriveModuleMeasuredValues]], output_file_name):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # for each time step
    frames: List[object] = []
    for i in range(len(points_in_time)):
        frame = create_robot_movement_frame(ax, drive_modules, body_states[i], drive_module_states[i])
        for j in range(i):
            position_plot = ax.plot(body_states[j].position_in_world_coordinates.x, body_states[j].position_in_world_coordinates.y, "*")
            frame.extend(position_plot)

        frames.append(frame)

    animation = ArtistAnimation(fig, frames, interval=20, blit=True, repeat=True, repeat_delay=10)

    writer = PillowWriter(fps=25)
    animation.save(output_file_name, writer=writer)


#def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

def create_robot_movement_frame(ax: Axes, drive_modules: List[DriveModule], body_state: BodyState, drive_module_states: List[DriveModuleMeasuredValues], wheel_color="-r", truck_color="-k") -> List[any]:  # pragma: no cover

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

        # Rotate the wheel to the drive module orientation
        wheel = (wheel.T.dot(drive_module_rotation_matrix)).T

        # Translate the wheel to the body, with the body at (0, 0)
        wheel[0, :] += drive_module.steering_axis_xy_position.x
        wheel[1, :] += drive_module.steering_axis_xy_position.y

        # Rotate the wheel to match the body orientation
        wheel = (wheel.T.dot(body_rotation_matrix)).T

        # Translate the wheel to the actual body coordinates
        wheel[0, :] += body_state.position_in_world_coordinates.x
        wheel[1, :] += body_state.position_in_world_coordinates.y

        # Store the wheel outline information
        wheels.append(wheel)

    plots: List[Line2D] = []
    body_plot = ax.plot(np.array(body_outline[0, :]).flatten(),
        np.array(body_outline[1, :]).flatten(),
        truck_color)
    plots.extend(body_plot)

    for wheel in wheels:
        wheel_plot = ax.plot(np.array(wheel[0, :]).flatten(),
            np.array(wheel[1, :]).flatten(),
            wheel_color)
        plots.extend(wheel_plot)

    return plots
