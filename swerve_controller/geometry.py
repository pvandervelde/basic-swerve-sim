from abc import ABC, abstractmethod
import math
from typing import List

class Point(object):

    def __init__(self, x_in_meters: float, y_in_meters: float, z_in_meters: float):
        self.x = x_in_meters
        self.y = y_in_meters
        self.z = z_in_meters

class Orientation(object):

    def __init__(self, x_orientation_in_radians: float, y_orientation_in_radians: float, z_orienation_in_radians: float):
        self.x = x_orientation_in_radians
        self.y = y_orientation_in_radians
        self.z = z_orienation_in_radians

class Vector3(object):

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

# Attempt to describe the rational value spaces for the different control variables, e.g.
# the value space for the drive module forward velocity is a 1D linear space running from
# -infinity to +infinity, while the value space for the drive module angle is a 1D circular
# space running from -pi to +pi.
class RealNumberValueSpace(ABC):
    # Returns the distance between two values in the space
    #
    # @param value1 The first value in the space
    # @param value2 The second value in the space
    # @return The distance between the two values
    @abstractmethod
    def smallest_distance_between_values(self, value1: float, value2: float) -> float:
        pass

    # Returns all possible distances between two values in the space. For unbounded value spaces
    # there will only be one distance, but for bounded value spaces there may be multiple distances depending
    # on if the boundaries are periodic or not.
    # For example, the distance between 0 and 2pi in a circular space running from -pi to +pi is 2pi, but the
    # distance between 0 and 4pi is 2pi as well. Similarly the distance between 1/4 pi and -1/4 pi is 1/2 pi,
    # and 3/2 pi depending on the direction of travel.
    #
    # @param value1 The first value in the space
    # @param value2 The second value in the space
    # @return All possible distances between the two values
    @abstractmethod
    def distances_between_values(self, value1: float, value2: float) -> List[float]:
        pass

    # Returns the value in the space that is closest to the target value
    #
    # @param value The value in the space
    # @return The value in the space that is closest to the target value
    @abstractmethod
    def normalize_value(self, value: float) -> float:
        pass


# Defines a 1D linear space running from -infinity to +infinity
class LinearSpace(RealNumberValueSpace):
    # Returns the distance between two values in the space
    #
    # @param value1 The first value in the space
    # @param value2 The second value in the space
    # @return The distance between the two values
    def smallest_distance_between_values(self, value1: float, value2: float) -> float:
        return value2 - value1

    # Returns all possible distances between two values in the space. For unbounded value spaces
    # there will only be one distance, but for bounded value spaces there may be multiple distances depending
    # on if the boundaries are periodic or not.
    # For example, the distance between 0 and 2pi in a circular space running from -pi to +pi is 2pi, but the
    # distance between 0 and 4pi is 2pi as well. Similarly the distance between 1/4 pi and -1/4 pi is 1/2 pi,
    # and 3/2 pi depending on the direction of travel.
    #
    # @param value1 The first value in the space
    # @param value2 The second value in the space
    # @return All possible distances between the two values
    def distances_between_values(self, value1: float, value2: float) -> List[float]:
        return [ value2 - value1 ]

    # Returns the value in the space that is closest to the target value
    #
    # @param value The value in the space
    # @return The value in the space that is closest to the target value
    def normalize_value(self, value: float) -> float:
        return value

# Defines a 1D circular space running from -pi to +pi
class CircularSpace(RealNumberValueSpace):

    # Returns the distance between two values in the space
    #
    # @param value1 The first value in the space
    # @param value2 The second value in the space
    # @return The distance between the two values
    def smallest_distance_between_values(self, value1: float, value2: float) -> float:
        normalized_start = self.normalize_value(value1)
        normalized_end = self.normalize_value(value2)

        diff_angle = normalized_end - normalized_start

        # Bring the angle back to the range [0, 2pi]
        if diff_angle >= 2 * math.pi:
            diff_angle -= 2 * math.pi
        else:
            if diff_angle < 0.0:
                diff_angle += 2 * math.pi

        # make sure we get the smallest angle
        if diff_angle > math.pi:
            diff_angle -= 2 * math.pi

        return diff_angle

    # Returns all possible distances between two values in the space. For unbounded value spaces
    # there will only be one distance, but for bounded value spaces there may be multiple distances depending
    # on if the boundaries are periodic or not.
    # For example, the distance between 0 and 2pi in a circular space running from -pi to +pi is 2pi, but the
    # distance between 0 and 4pi is 2pi as well. Similarly the distance between 1/4 pi and -1/4 pi is 1/2 pi,
    # and 3/2 pi depending on the direction of travel.
    #
    # @param value1 The first value in the space
    # @param value2 The second value in the space
    # @return All possible distances between the two values
    def distances_between_values(self, value1: float, value2: float) -> List[float]:
        normalized_start = self.normalize_value(value1)
        normalized_end = self.normalize_value(value2)

        diff_angle = normalized_end - normalized_start

        # Bring the angle back to the range [0, 2pi]
        if diff_angle >= 2 * math.pi:
            diff_angle -= 2 * math.pi
        else:
            if diff_angle < 0.0:
                diff_angle += 2 * math.pi

        # make sure we get the smallest angle
        if diff_angle > math.pi:
            return [ diff_angle - 2 * math.pi, diff_angle ]
        else:
            return [ diff_angle, diff_angle - 2 * math.pi ]

    # Returns the value in the space that is closest to the target value
    #
    # @param value The value in the space
    # @return The value in the space that is closest to the target value
    def normalize_value(self, value: float) -> float:
        # reduce the angle to [-2pi, 2pi]
        angle = value % (2 * math.pi)

        # Force the angle to the between 0 and 2pi
        angle = (angle + 2 * math.pi) % (2 * math.pi)

        # Force the angle to the between -pi and pi
        if angle > math.pi:
            angle -= 2 * math.pi

        return angle
