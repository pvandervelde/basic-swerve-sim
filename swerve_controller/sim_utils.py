
#!/usr/bin/python3

from math import cos, isclose, sin

import numpy as np
from typing import List, Tuple

# local
from .drive_module import DriveModule
from .geometry import Point
from .states import DriveModuleMeasuredValues

# From here: https://stackoverflow.com/a/42727584
def get_intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> Point:
    """
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """

    # s for stacked
    s = np.vstack(
            [
                [a1.x, a1.y],
                [a2.x, a2.y],
                [b1.x, b1.y],
                [b2.x, b2.y],
            ]
        )

    # h for homogeneous
    h = np.hstack((s, np.ones((4, 1))))

    # get first line
    l1 = np.cross(h[0], h[1])

    # get second line
    l2 = np.cross(h[2], h[3])

    # point of intersection
    x, y, z = np.cross(l1, l2)

    # lines are parallel
    if isclose(z, 0.0, abs_tol=1e-5, rel_tol=1e-5):
        return Point(float('inf'), float('inf'), 0.0)

    return Point(x/z, y/z, 0.0)

def instantaneous_center_of_rotation_at_current_time(drive_module_states: List[DriveModuleMeasuredValues]) -> List[Tuple[DriveModuleMeasuredValues, DriveModuleMeasuredValues, Point]]:
    # Get axle lines for each drive module
    #
    # First point is the location of the drive module
    # Second point is a point on the axle line.
    #
    # The axle line is computed by
    # Getting the line perpendicular to the orientation vector.
    # The orientation vector is [cos(orientation.z), sin(orientation.z) ]^T
    # The perpendicular vector of vector [x, y]^T in 2d is [-y, x]^T
    # So the orientation perpendicular vector is [-sin(orientation.z), cos(orientation.z)]^T
    #
    # Thus a point on that line is [-sin(orientation.z) + position.x, cos(orientation.z) + position.y]^T
    axle_lines: List[Tuple[Point, Point]] = []
    for state in drive_module_states:
        line = (
            state.position_in_body_coordinates,
            Point(
                -sin(state.orientation_in_body_coordinates.z) + state.position_in_body_coordinates.x,
                cos(state.orientation_in_body_coordinates.z) + state.position_in_body_coordinates.y,
                0.0,
            )
        )
        axle_lines.append(line)

    # Compute intersections
    result: List[Tuple[DriveModule, DriveModule, Point]] = []

    col: int = 0
    for col in range(len(drive_module_states)):
        set1 = axle_lines[col]

        row: int = 0
        for row in range(col + 1, len(drive_module_states)):
            set2 = axle_lines[row]

            intersect_point = get_intersect(set1[0], set1[1], set2[0], set2[1])
            result.append(
                (
                    drive_module_states[col],
                    drive_module_states[row],
                    intersect_point
                )
            )

    return result