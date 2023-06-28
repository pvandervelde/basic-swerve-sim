
from math import isclose, isinf
from os import makedirs, path
import plotly.graph_objects as go
import plotly.offline as py
from plotly.subplots import make_subplots
from typing import List, Mapping, NamedTuple, Tuple
from sim_output.animate import plot_movement_through_space

# local
from swerve_controller.control_model import DriveModuleMeasuredValues, Point
from swerve_controller.drive_module import DriveModule
from swerve_controller.sim_utils import instantaneous_center_of_rotation_at_current_time
from swerve_controller.states import BodyState

class ProfilePlotValues(NamedTuple):
    name: str
    markers: Mapping[str, int]
    x_values: List[float]
    y_values: List[float]
    annotations: List[str] = []

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

def get_plot(rows: int, cols: int) -> go.Figure:
    fig = make_subplots(
        rows=rows,
        cols=cols,
        shared_xaxes=False,
        shared_yaxes=True,
        )

    return fig

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

    plot_file_path = path.join(output_directory, "{}-trajectory.html".format(short_name))
    plot_movement_through_space(points_in_time, drive_modules, body_states, drive_states, icr_coordinate_map, plot_file_path)

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
