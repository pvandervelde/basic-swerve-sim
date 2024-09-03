import math
from abc import ABC, abstractmethod
from typing import List, Tuple

from scipy.interpolate import BSpline, make_interp_spline

from swerve_controller.geometry import LinearUnboundedSpace, RealNumberValueSpace

# local
from .errors import InvalidTimeFractionException


class ProfilePoint(object):
    def __init__(
        self,
        time_fraction: float,
        value: float,
        first_derivative: float,
        second_derivative: float,
        third_derivative: float,
    ):
        self.time_fraction = time_fraction
        self.value = value
        self.first_derivative = first_derivative
        self.second_derivative = second_derivative
        self.third_derivative = third_derivative


class TransientVariableProfile(ABC):
    @abstractmethod
    def first_derivative_at(self, time_since_start_of_profile: float) -> float:
        pass

    @abstractmethod
    def second_derivative_at(self, time_since_start_of_profile: float) -> float:
        pass

    @abstractmethod
    def third_derivative_at(self, time_since_start_of_profile: float) -> float:
        pass

    @abstractmethod
    def value_at(self, time_since_start_of_profile: float) -> float:
        pass


class SingleVariableLinearProfile(TransientVariableProfile):
    def __init__(
        self,
        start: float,
        end: float,
        end_time: float = 1.0,
        coordinate_space: RealNumberValueSpace = LinearUnboundedSpace(),
    ):
        self.coordinate_space = coordinate_space
        self.start = coordinate_space.normalize_value(start)
        self.end = coordinate_space.normalize_value(end)

        self.end_time = end_time

    def first_derivative_at(self, time_since_start_of_profile: float) -> float:
        return (
            self.coordinate_space.smallest_distance_between_values(self.start, self.end)
            / self.end_time
        )

    def second_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if math.isclose(0.0, time_since_start_of_profile, rel_tol=1e-2, abs_tol=1e-2):
            return (
                self.coordinate_space.smallest_distance_between_values(
                    self.start, self.end
                )
                / 0.01
            )

        if math.isclose(
            self.end_time, time_since_start_of_profile, rel_tol=1e-2, abs_tol=1e-2
        ):
            return (
                -self.coordinate_space.smallest_distance_between_values(
                    self.start, self.end
                )
                / 0.01
            )

        return 0.0

    def third_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if math.isclose(0.0, time_since_start_of_profile, rel_tol=1e-2, abs_tol=1e-2):
            return (
                self.coordinate_space.smallest_distance_between_values(
                    self.start, self.end
                )
                / 0.01
                / 0.01
            )

        if math.isclose(
            self.end_time, time_since_start_of_profile, rel_tol=1e-2, abs_tol=1e-2
        ):
            return (
                -self.coordinate_space.smallest_distance_between_values(
                    self.start, self.end
                )
                / 0.01
                / 0.01
            )

        return 0.0

    def value_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return self.start

        if time_since_start_of_profile > self.end_time:
            return self.end

        distance = self.coordinate_space.smallest_distance_between_values(
            self.start, self.end
        )
        return self.coordinate_space.normalize_value(
            distance * time_since_start_of_profile / self.end_time + self.start
        )


class SingleVariableCompoundProfileValue(object):
    def __init__(
        self,
        location_fraction: float,
        value: float,
        first_derivative=0.0,
        second_derivative=0.0,
    ):
        self.location = location_fraction
        self.value = value
        self.first_derivative = first_derivative
        self.second_derivative = second_derivative


class SingleVariableMultiPointLinearProfile(TransientVariableProfile):
    """
    A class representing a single-variable multi-point linear profile.

    This class is used to create a profile with multiple points. Between each point a linear profile is assumed.
    The profile is assumed to start at time = 0.0 and ends at the given end_time. In order to retrieve a value
    or derivative either a specific time_fraction is specified (between 0.0 and 1.0) which internally is translated to
    a given point in time, or a specific point in time is specified.

    Attributes:
    - coordinate_space: The coordinate space for the profile values.
    - profiles: A list of SingleVariableCompoundProfileValue objects representing the points in the profile.
    - end_time: The end time of the profile.
    """

    def __init__(
        self,
        start: float,
        end: float,
        end_time: float = 1.0,
        start_velocity: float = 0.0,
        end_velocity: float = 0.0,
        start_acceleration: float = 0.0,
        end_acceleration: float = 0.0,
        coordinate_space: RealNumberValueSpace = LinearUnboundedSpace(),
    ):
        """
        Initializes a SingleVariableMultiPointLinearProfile object.

        Args:
        - start: The starting value of the profile.
        - end: The ending value of the profile.
        - end_time: The end time of the profile. Default is 1.0.
        - coordinate_space: The coordinate space for the profile values. Default is LinearUnboundedSpace().
        """

        self.coordinate_space = coordinate_space
        self.profiles: List[SingleVariableCompoundProfileValue] = [
            SingleVariableCompoundProfileValue(
                0.0, start, start_velocity, start_acceleration
            ),
            SingleVariableCompoundProfileValue(
                end_time, end, end_velocity, end_acceleration
            ),
        ]

        self.end_time = end_time

        self.spline: BSpline = None

    def add_value(
        self,
        time_since_start_of_profile: float,
        value: float,
        first_derivative: float = 0.0,
        second_derivative: float = 0.0,
    ):
        """
        Adds a value to the profile at the specified time fraction.

        Args:
        - time_since_start_of_profile: The time since the profile started.
        - value: The value to add to the profile.
        """

        if time_since_start_of_profile < 0.0:
            time_since_start_of_profile = 0.0

        if time_since_start_of_profile > self.end_time:
            time_since_start_of_profile = self.end_time

        section = SingleVariableCompoundProfileValue(
            time_since_start_of_profile,
            self.coordinate_space.normalize_value(value),
            first_derivative,
            second_derivative,
        )

        for i in range(len(self.profiles)):
            if math.isclose(
                time_since_start_of_profile,
                self.profiles[i].location,
                rel_tol=1e-7,
                abs_tol=1e-7,
            ):
                # Matching location. Replace it
                self.profiles[i] = section
                # we're replacing an existing point so the minimum polynomial order doesn't change
                break

            if self.profiles[i].location < time_since_start_of_profile:
                if i + 1 >= len(self.profiles):
                    # last profile
                    self.profiles.append(section)
                    break
                else:
                    # not the last profile. Go around the loop and we'll get it then
                    continue

            if self.profiles[i].location > time_since_start_of_profile:
                if i - 1 >= 0:
                    if self.profiles[i - 1].location < time_since_start_of_profile:
                        self.profiles.insert(i, section)
                        break

    def find_time_indices_for_time_fraction(
        self, time_since_profile_start: float
    ) -> Tuple[int, int]:
        # Assumption:
        #  0.0 <= time_since_profile_start <= end_time

        # Find the two time points that encompasses the given time_since_profile_start. One value will be the closest
        # smaller value and one will be the closest larger value
        index = -1
        for i in range(len(self.profiles)):
            # If the i-th value is smaller and the (i+1)-th is bigger then we have found the correct location
            if self.profiles[i].location >= time_since_profile_start:
                # Found the first location that is bigger than time_fraction
                index = i
                break

        if index == -1:
            # we didn't find anything. that's weird because there's a guaranteed beginning and ending
            # throw a hissy
            raise InvalidTimeFractionException(
                f"Could not find any known time locations smaller and larger than { time_since_profile_start }"
            )

        if i == 0:
            return (i, i + 1)
        else:
            return (i - 1, i)

    def first_derivative_at(self, time_since_start_of_profile: float) -> float:
        """
        Calculates the first derivative of the profile at the specified time.

        Args:
        - time_fraction: The time at which to calculate the first derivative.

        Returns:
        The value of the first derivative at the specified time.
        """

        if time_since_start_of_profile < 0.0:
            time_since_start_of_profile = 0.0

        if time_since_start_of_profile > self.end_time:
            time_since_start_of_profile = self.end_time

        poly = self.get_defining_spline()
        return float(
            poly.__call__(time_since_start_of_profile, nu=1, extrapolate=False)
        )

    def get_defining_spline(self) -> BSpline:
        if self.spline is None:
            k = 3 if len(self.profiles) >= 4 else len(self.profiles) - 1

            ts: List[float] = [x.location for x in self.profiles]
            ys: List[float] = [x.value for x in self.profiles]

            starting_first_derivative = self.profiles[0].first_derivative
            ending_first_derivative = self.profiles[-1].first_derivative

            self.spline = make_interp_spline(
                ts,
                ys,
                k=k,
                bc_type=(
                    [(1, starting_first_derivative)],
                    [(1, ending_first_derivative)],
                ),
            )

        return self.spline

    def polynomial_order(self) -> int:
        # For now we don't go beyond a 3rd order polynomial. A 3rd order polynomial should give us
        # - 3rd order position
        # - 2nd order velocity
        # - 1st order acceration
        # - 0th order jerk
        if self.maximum_polynomial_order <= 3:
            return self.maximum_polynomial_order
        else:
            return 3

    def second_derivative_at(self, time_since_start_of_profile: float) -> float:
        """
        Calculates the second derivative of the profile at the specified time.

        Args:
        - time_since_start_of_profile: The time at which to calculate the second derivative.

        Returns:
        The value of the second derivative at the specified time.
        """

        if time_since_start_of_profile < 0.0:
            time_since_start_of_profile = 0.0

        if time_since_start_of_profile > self.end_time:
            time_since_start_of_profile = self.end_time

        poly = self.get_defining_spline()
        if poly.k < 2:
            return 0.0

        return float(
            poly.__call__(time_since_start_of_profile, nu=2, extrapolate=False)
        )

    def third_derivative_at(self, time_since_start_of_profile: float) -> float:
        """
        Calculate the third derivative of the profile at a given time.

        Args:
            time_since_start_of_profile (float): The time since the start of the profile.

        Returns:
            float: The value of the third derivative at the given time.
        """

        if time_since_start_of_profile < 0.0:
            time_since_start_of_profile = 0.0

        if time_since_start_of_profile > self.end_time:
            time_since_start_of_profile = self.end_time

        poly = self.get_defining_spline()
        if poly.k < 3:
            return 0.0

        return float(
            poly.__call__(time_since_start_of_profile, nu=3, extrapolate=False)
        )

    def value_at(self, time_since_start_of_profile: float) -> float:
        """
        Returns the value of the profile at a given time.

        Args:
            time_since_start_of_profile (float): The time since the start of the profile.

        Returns:
            float: The value of the profile at the given time.
        """

        if time_since_start_of_profile < 0.0:
            time_since_start_of_profile = 0.0

        if time_since_start_of_profile > self.end_time:
            time_since_start_of_profile = self.end_time

        poly = self.get_defining_spline()
        return self.coordinate_space.normalize_value(
            float(poly.__call__(time_since_start_of_profile, nu=0, extrapolate=False))
        )


# see: https://www.mathworks.com/help/robotics/ug/design-a-trajectory-with-velocity-limits-using-a-trapezoidal-velocity-profile.html
class SingleVariableTrapezoidalProfile(TransientVariableProfile):
    def __init__(
        self,
        start: float,
        end: float,
        end_time: float = 1.0,
        value_space: RealNumberValueSpace = LinearUnboundedSpace(),
    ):
        self.value_space = value_space
        self.start = value_space.normalize_value(start)
        self.end = value_space.normalize_value(end)
        self.end_time = end_time

        # For a trapezoidal motion profile the progress in the profile
        # is based on the first derrivative, e.g. if the profile is
        # for position then the progress from one position to another
        # is based on the velocity profile
        #
        # The two extremes are:
        # - Constant velocity over the entire time span
        # - Constant acceleration over half the timespan and constant decleration over
        #   the other half of the timespan
        #
        # In the first case the velocity is (endValue - startValue) / timeSpan
        # In the second case the velocity_max is 2 * ((endValue - startValue) / timeSpan)
        # The actual velocity should be in between these values
        #
        # Initially assume that all phases take 1/3 of the total time
        #
        # Profiles are always defined on a relative time span of 1.0, which makes
        # it easy to alter the timespan.
        #
        # v_min = (end - start) / 1.0
        # v_max = 2 * v_min
        #
        # Assume the profile is 1/3rd acceleration, 1/3 constant velocity and
        # 1/3rd deceleration
        #
        # The total distance is equal to the integral of velocity over time. For
        # a trapezoidal profile this means
        #
        # s = 0.5 * v * t_acc + v * t_const + 0.5 * v * t_dec
        #
        # where:
        # - s = distance
        # - v = maximum velocity in the profile
        # - t_acc = time taken to accelerate
        # - t_const = time taken at constant velocity
        # - t_dec = time taken to decelerate
        #
        # s = v * (0.5 * t_acc + t_const + 0.5 * t_dec)
        #
        # Each segment is 1/3 of the total time
        #
        # s = v * 2/3 * t
        #
        # v = 1.5 * s / t
        self.velocity = 1.5 * (self.end - self.start) / self.end_time

        self.acceleration_phase_ratio = 1 / 3 * self.end_time
        self.constant_phase_ratio = 1 / 3 * self.end_time
        self.deceleration_phase_ratio = 1 / 3 * self.end_time

    def first_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if time_since_start_of_profile < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            velocity_due_to_acceleration = (
                (self.velocity - starting_velocity) / (self.acceleration_phase_ratio)
            ) * time_since_start_of_profile
            return starting_velocity + velocity_due_to_acceleration

        if time_since_start_of_profile > (
            self.acceleration_phase_ratio + self.constant_phase_ratio
        ):
            # deccelerating
            starting_velocity = self.velocity
            ending_velocity = 0.0
            velocity_due_to_acceleration = (
                (ending_velocity - self.velocity) / (self.deceleration_phase_ratio)
            ) * (
                time_since_start_of_profile
                - (self.acceleration_phase_ratio + self.constant_phase_ratio)
            )
            return starting_velocity + velocity_due_to_acceleration

        return self.velocity

    def second_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if time_since_start_of_profile < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            return (self.velocity - starting_velocity) / (self.acceleration_phase_ratio)

        if time_since_start_of_profile > (
            self.acceleration_phase_ratio + self.constant_phase_ratio
        ):
            # deccelerating
            ending_velocity = 0.0
            return (ending_velocity - self.velocity) / (self.deceleration_phase_ratio)

        return 0.0

    def third_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if math.isclose(0.0, time_since_start_of_profile, rel_tol=1e-2, abs_tol=1e-2):
            starting_velocity = 0.0
            return (
                ((self.velocity - starting_velocity) / (self.acceleration_phase_ratio))
                - 0.0
            ) / 0.01

        if math.isclose(
            time_since_start_of_profile,
            self.acceleration_phase_ratio,
            rel_tol=1e-2,
            abs_tol=1e-2,
        ):
            starting_velocity = 0.0
            return (
                0.0
                - (
                    (self.velocity - starting_velocity)
                    / (self.acceleration_phase_ratio)
                )
            ) / 0.01

        if math.isclose(
            time_since_start_of_profile,
            (self.acceleration_phase_ratio + self.constant_phase_ratio),
            rel_tol=1e-2,
            abs_tol=1e-2,
        ):
            ending_velocity = 0.0
            return (
                ((ending_velocity - self.velocity) / (self.deceleration_phase_ratio))
                - 0.0
            ) / 0.01

        if math.isclose(
            self.end_time, time_since_start_of_profile, rel_tol=1e-2, abs_tol=1e-2
        ):
            ending_velocity = 0.0
            return (
                0.0
                - ((ending_velocity - self.velocity) / (self.acceleration_phase_ratio))
            ) / 0.01

        return 0.0

    def value_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return self.start

        if time_since_start_of_profile > self.end_time:
            return self.end

        if time_since_start_of_profile < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            distance_change_from_velocity = (
                starting_velocity * time_since_start_of_profile
            )
            distance_change_from_acceleration = (
                0.5
                * (
                    (self.velocity - starting_velocity)
                    / (self.acceleration_phase_ratio)
                )
                * time_since_start_of_profile
                * time_since_start_of_profile
            )
            result = (
                self.start
                + distance_change_from_velocity
                + distance_change_from_acceleration
            )
            return self.value_space.normalize_value(result)

        distance_due_to_inital_acceleration = (
            0.5 * self.velocity * self.acceleration_phase_ratio
        )
        if time_since_start_of_profile > (
            self.acceleration_phase_ratio + self.constant_phase_ratio
        ):
            # deccelerating
            distance_due_to_constant_velocity = (
                self.velocity * self.constant_phase_ratio
            )

            deceleration_time = time_since_start_of_profile - (
                self.acceleration_phase_ratio + self.constant_phase_ratio
            )
            ending_velocity = 0.0
            distance_due_to_deceleration = (
                self.velocity * deceleration_time
                + 0.5
                * ((ending_velocity - self.velocity) / (self.deceleration_phase_ratio))
                * deceleration_time
                * deceleration_time
            )
            result = (
                self.start
                + distance_due_to_inital_acceleration
                + distance_due_to_constant_velocity
                + distance_due_to_deceleration
            )
            return self.value_space.normalize_value(result)

        result = (
            self.start
            + distance_due_to_inital_acceleration
            + (time_since_start_of_profile - self.acceleration_phase_ratio)
            * self.velocity
        )
        return self.value_space.normalize_value(result)


# S-Curve profile
# --> controlled by the second derivative being linear
class SingleVariableSCurveProfile(TransientVariableProfile):
    def __init__(
        self,
        start: float,
        end: float,
        end_time: float = 1.0,
        value_space: RealNumberValueSpace = LinearUnboundedSpace(),
    ):
        self.value_space = value_space
        self.start = value_space.normalize_value(start)
        self.end = value_space.normalize_value(end)
        self.end_time = end_time

        #      t_1     t_2  t_3     t_4  t_5       t_6  t_7
        #  |    *______*
        #  |   /        \
        #  |  /          \
        #  | /            \
        #  |/______________\*_______*____________________________
        #  |                         \                /
        #  |                          \              /
        #  |                           \            /
        #  |                            \*________*/
        #  |
        #
        #
        #
        #
        #
        #
        # For s-curve motion profile the progress in the profile
        # is based on the second and third derrivatives, e.g. if the profile is
        # for position then the progress from one position to another
        # is based on the acceleration and jerk profiles
        #
        # It is assumed that the profile has 7 different sections:
        #
        # 1) Positive jerk, increasing acceleration, increasing velocity
        # 2) zero jerk, constant acceleration, increasing velocity
        # 3) negative jerk, decreasing acceleration, increasing velocity
        # 4) zero jerk, zero acceleration, constant velocity
        # 5) negative jerk, increasingly negative acceleration, reducing velocity
        # 6) zero jerk, constant negative acceleration, reducing velocity
        # 7) positive jerk, decreasing negative acceleration, reducing velocity
        #
        # At the start of state 1) and at the end of state 7) the jerk,
        # acceleration and velocity are zero.
        #
        # For now assume that the profile time sections are:
        #
        # 1) 1/8 of the total time
        # 2) 1/8 of the total time
        # 3) 1/8 of the total time
        # 4) 2/8 of the total time
        # 5) 1/8 of the total time
        # 6) 1/8 of the total time
        # 7) 1/8 of the total time
        #
        # Solving the linear equations for distance based on jerk for each section
        # gives the
        #
        # s = j * 10 / 512 * t^3
        #
        # j =  (s * 512) / (10 * t^3)
        self.jerk = 512 / 10 * (self.end - self.start) / math.pow(self.end_time, 3.0)

        self.positive_acceleration_phase_ratio = 1 / 8 * self.end_time
        self.constant_acceleration_phase_ratio = 1 / 8 * self.end_time
        self.negative_acceleration_phase_ratio = 1 / 8 * self.end_time
        self.constant_phase_ratio = 1 / 4 * self.end_time

        self.t1 = self.positive_acceleration_phase_ratio
        self.t2 = self.t1 + self.constant_acceleration_phase_ratio
        self.t3 = self.t2 + self.negative_acceleration_phase_ratio
        self.t4 = self.t3 + self.constant_phase_ratio
        self.t5 = self.t4 + self.positive_acceleration_phase_ratio
        self.t6 = self.t5 + self.constant_acceleration_phase_ratio
        self.t7 = self.end_time

    def first_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if time_since_start_of_profile < self.t1:
            return (
                0.5
                * self.jerk
                * time_since_start_of_profile
                * time_since_start_of_profile
            )

        a1 = self.jerk * self.t1
        v1 = 0.5 * a1 * self.t1
        if time_since_start_of_profile < self.t2:
            return v1 + a1 * (time_since_start_of_profile - self.t1)

        a2 = a1
        v2 = v1 + a1 * (self.t2 - self.t1)
        if time_since_start_of_profile < self.t3:
            return (
                -0.5
                * self.jerk
                * (time_since_start_of_profile - self.t2)
                * (time_since_start_of_profile - self.t2)
                + a2 * (time_since_start_of_profile - self.t2)
                + v2
            )

        v3 = (
            -0.5 * self.jerk * (self.t3 - self.t2) * (self.t3 - self.t2)
            + a2 * (self.t3 - self.t2)
            + v2
        )
        if time_since_start_of_profile < self.t4:
            return v3

        if time_since_start_of_profile < self.t5:
            return (
                -0.5
                * self.jerk
                * (time_since_start_of_profile - self.t4)
                * (time_since_start_of_profile - self.t4)
                + v3
            )

        a5 = -self.jerk * (self.t5 - self.t4)
        v5 = -0.5 * self.jerk * (self.t5 - self.t4) * (self.t5 - self.t4) + v3
        if time_since_start_of_profile < self.t6:
            return a5 * (time_since_start_of_profile - self.t5) + v5

        a6 = a5
        v6 = a5 * (self.t6 - self.t5) + v5
        return (
            0.5
            * self.jerk
            * (time_since_start_of_profile - self.t6)
            * (time_since_start_of_profile - self.t6)
            + a6 * (time_since_start_of_profile - self.t6)
            + v6
        )

    def second_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if time_since_start_of_profile < self.t1:
            return self.jerk * time_since_start_of_profile

        if time_since_start_of_profile < self.t2:
            return self.jerk * self.t1

        if time_since_start_of_profile < self.t3:
            return (
                -self.jerk * (time_since_start_of_profile - self.t2)
                + self.jerk * self.t1
            )

        if time_since_start_of_profile < self.t4:
            return 0.0

        if time_since_start_of_profile < self.t5:
            return -self.jerk * (time_since_start_of_profile - self.t4)

        if time_since_start_of_profile < self.t6:
            return -self.jerk * (self.t5 - self.t4)

        return -self.jerk * (self.t5 - self.t4) + self.jerk * (
            time_since_start_of_profile - self.t6
        )

    def third_derivative_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return 0.0

        if time_since_start_of_profile > self.end_time:
            return 0.0

        if time_since_start_of_profile < self.t1:
            return self.jerk

        if time_since_start_of_profile < self.t2:
            return 0.0

        if time_since_start_of_profile < self.t3:
            return -self.jerk

        if time_since_start_of_profile < self.t4:
            return 0.0

        if time_since_start_of_profile < self.t5:
            return -self.jerk

        if time_since_start_of_profile < self.t6:
            return 0.0

        return self.jerk

    def value_at(self, time_since_start_of_profile: float) -> float:
        if time_since_start_of_profile < 0.0:
            return self.start

        if time_since_start_of_profile > self.end_time:
            return self.end

        if time_since_start_of_profile < self.t1:
            result = (
                1 / 6 * self.jerk * math.pow(time_since_start_of_profile, 3.0)
                + self.start
            )
            return self.value_space.normalize_value(result)

        a1 = self.jerk * self.t1
        v1 = 0.5 * a1 * self.t1
        s1 = 1 / 6 * self.jerk * math.pow(self.t1, 3.0) + self.start
        if time_since_start_of_profile < self.t2:
            result = (
                v1 * (time_since_start_of_profile - self.t1)
                + 0.5
                * a1
                * (time_since_start_of_profile - self.t1)
                * (time_since_start_of_profile - self.t1)
                + s1
            )
            return self.value_space.normalize_value(result)

        a2 = a1
        v2 = v1 + a1 * (self.t2 - self.t1)
        s2 = (
            v1 * (self.t2 - self.t1)
            + 0.5 * a1 * (self.t2 - self.t1) * (self.t2 - self.t1)
            + s1
        )
        if time_since_start_of_profile < self.t3:
            result = (
                -1
                / 6
                * self.jerk
                * math.pow(time_since_start_of_profile - self.t2, 3.0)
                + 0.5 * a2 * math.pow(time_since_start_of_profile - self.t2, 2.0)
                + v2 * (time_since_start_of_profile - self.t2)
                + s2
            )
            return self.value_space.normalize_value(result)

        v3 = (
            -0.5 * self.jerk * (self.t3 - self.t2) * (self.t3 - self.t2)
            + a1 * (self.t3 - self.t2)
            + v2
        )
        s3 = (
            -1 / 6 * self.jerk * math.pow(self.t3 - self.t2, 3.0)
            + 0.5 * a2 * math.pow(self.t3 - self.t2, 2.0)
            + v2 * (self.t3 - self.t2)
            + s2
        )
        if time_since_start_of_profile < self.t4:
            result = v3 * (time_since_start_of_profile - self.t3) + s3
            return self.value_space.normalize_value(result)

        s4 = v3 * (self.t4 - self.t3) + s3
        if time_since_start_of_profile < self.t5:
            result = (
                -1
                / 6
                * self.jerk
                * math.pow(time_since_start_of_profile - self.t4, 3.0)
                + v3 * (time_since_start_of_profile - self.t4)
                + s4
            )
            return self.value_space.normalize_value(result)

        a5 = -self.jerk * (self.t5 - self.t4)
        v5 = -0.5 * self.jerk * (self.t5 - self.t4) * (self.t5 - self.t4) + v3
        s5 = (
            -1 / 6 * self.jerk * math.pow(self.t5 - self.t4, 3.0)
            + v3 * (self.t5 - self.t4)
            + s4
        )
        if time_since_start_of_profile < self.t6:
            result = (
                0.5 * a5 * math.pow(time_since_start_of_profile - self.t5, 2.0)
                + v5 * (time_since_start_of_profile - self.t5)
                + s5
            )
            return self.value_space.normalize_value(result)

        a6 = a5
        v6 = a5 * (self.t6 - self.t5) + v5
        s6 = 0.5 * a5 * math.pow(self.t6 - self.t5, 2.0) + v5 * (self.t6 - self.t5) + s5
        result = (
            1 / 6 * self.jerk * math.pow(time_since_start_of_profile - self.t6, 3.0)
            + 0.5 * a6 * math.pow(time_since_start_of_profile - self.t6, 2.0)
            + v6 * (time_since_start_of_profile - self.t6)
            + s6
        )
        return self.value_space.normalize_value(result)


# 4th and 5th order s-curve
