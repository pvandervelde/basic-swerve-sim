from abc import ABC, abstractmethod
import math
from numpy.polynomial.polynomial import Polynomial
from typing import List, Tuple

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
        third_derivative: float
    ):
        self.time_fraction = time_fraction
        self.value = value
        self.first_derivative = first_derivative
        self.second_derivative = second_derivative
        self.third_derivative = third_derivative

class TransientVariableProfile(ABC):

    @abstractmethod
    def first_derivative_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def second_derivative_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def third_derivative_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def value_at(self, time_fraction: float) -> float:
        pass

class SingleVariableLinearProfile(TransientVariableProfile):

    def __init__(self, start: float, end: float, coordinate_space: RealNumberValueSpace = LinearUnboundedSpace()):
        self.coordinate_space = coordinate_space
        self.start = coordinate_space.normalize_value(start)
        self.end = coordinate_space.normalize_value(end)

    def first_derivative_at(self, time_fraction: float) -> float:
        return self.coordinate_space.smallest_distance_between_values(self.start, self.end)

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if math.isclose(0.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return self.coordinate_space.smallest_distance_between_values(self.start, self.end) / 0.01

        if math.isclose(1.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return -self.coordinate_space.smallest_distance_between_values(self.start, self.end) / 0.01

        return 0.0

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if math.isclose(0.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return self.coordinate_space.smallest_distance_between_values(self.start, self.end) / 0.01 / 0.01

        if math.isclose(1.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return -self.coordinate_space.smallest_distance_between_values(self.start, self.end) / 0.01 / 0.01

        return 0.0

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start

        if time_fraction > 1.0:
            return self.end

        distance = self.coordinate_space.smallest_distance_between_values(self.start, self.end)
        return self.coordinate_space.normalize_value(distance * time_fraction + self.start)

class SingleVariableCompoundProfileValue(object):

    def __init__(
        self,
        location_fraction: float,
        value: float
    ):
        self.location = location_fraction
        self.value = value

class SingleVariableMultiPointLinearProfile(TransientVariableProfile):

    def __init__(self, start: float, end: float, end_time: float = 1.0, coordinate_space: RealNumberValueSpace = LinearUnboundedSpace()):
        self.coordinate_space = coordinate_space
        self.profiles: List[SingleVariableCompoundProfileValue] = [
            SingleVariableCompoundProfileValue(0.0, start),
            SingleVariableCompoundProfileValue(end_time, end)
        ]

        self.end_time = end_time

        # We have two points (begin and end) so at best we can do a linear approach
        self.maximum_polynomial_order = 1

    def add_value(self, time_fraction: float, value: float):
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > self.end_time:
            time_fraction = self.end_time

        section = SingleVariableCompoundProfileValue(
            time_fraction,
            self.coordinate_space.normalize_value(value))

        for i in range(len(self.profiles)):
            if math.isclose(time_fraction, self.profiles[i].location, rel_tol=1e-7, abs_tol=1e-7):
                # Matching location. Replace it
                self.profiles[i] = section
                # we're replacing an existing point so the minimum polynomial order doesn't change
                break

            if self.profiles[i].location < time_fraction:
                if i + 1 >= len(self.profiles):
                    # last profile
                    self.profiles.append(section)
                    self.maximum_polynomial_order += 1
                    break
                else:
                    # not the last profile. Go around the loop and we'll get it then
                    continue

            if self.profiles[i].location > time_fraction:
                if i - 1 >= 0:
                    if self.profiles[i - 1].location < time_fraction:
                        self.profiles.insert(i, section)
                        self.maximum_polynomial_order += 1
                        break

    def find_time_indices_for_time_fraction(self, time_fraction: float) -> Tuple[int, int]:
        # Assumption:
        #  0.0 <= time_fraction <= 1.0

        # Find the two time fractions that encompasses the given time_fraction. One value will be the closest
        # smaller value and one will be the closest larger value
        index = -1
        for i in range(len(self.profiles)):
            # If the i-th value is smaller and the (i+1)-th is bigger then we have found the correct location
            if self.profiles[i].location >= time_fraction:
                # Found the first location that is bigger than time_fraction
                index = i
                break

        if index == -1:
            # we didn't find anything. that's weird because there's a guaranteed beginning and ending
            # throw a hissy
            raise InvalidTimeFractionException(f'Could not find any known time locations smaller and larger than { time_fraction }')

        if i == 0:
            return (i, i + 1)
        else:
            return ( i - 1, i)

    def first_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > self.end_time:
            time_fraction = self.end_time

        poly = self.polynomial_at_time(time_fraction)
        first_deriv = poly.deriv(1)
        return first_deriv(time_fraction)

    def polynomial_at_time(self, time_fraction: float) -> Polynomial:
        # find the index for the points
        number_of_points = self.polynomial_order() + 1
        smaller_index, larger_index = self.find_time_indices_for_time_fraction(time_fraction)

        # bias towards the 'future' because that's what we want to achieve
        past_points = number_of_points // 2
        future_points = number_of_points - past_points

        smallest_index = smaller_index - past_points + 1
        if smallest_index < 0:
            future_points += int(abs(smallest_index))
            smallest_index = 0

        largest_index = larger_index + future_points - 1
        if largest_index > len(self.profiles) - 1:
            smallest_index -= int(abs(len(self.profiles) - 1 - largest_index))
            largest_index = len(self.profiles) - 1

        time_fractions = []
        values = []
        for i in range(smallest_index, largest_index + 1):
            time_fractions.append(self.profiles[i].location)
            values.append(self.profiles[i].value)

        return Polynomial.fit(time_fractions, values, number_of_points - 1, domain=[time_fractions[0], time_fractions[len(time_fractions) - 1]])

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

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > self.end_time:
            time_fraction = self.end_time

        poly = self.polynomial_at_time(time_fraction)
        first_deriv = poly.deriv(2)
        return first_deriv(time_fraction)

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > self.end_time:
            time_fraction = self.end_time

        poly = self.polynomial_at_time(time_fraction)
        first_deriv = poly.deriv(3)
        return first_deriv(time_fraction)

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > self.end_time:
            time_fraction = self.end_time

        poly = self.polynomial_at_time(time_fraction)
        return self.coordinate_space.normalize_value(poly(time_fraction))

class CompoundProfileSection(object):

    def __init__(
        self,
        starting_location_fraction: float,
        ending_location_fraction: float,
        profile: TransientVariableProfile
    ):
        self.starting_location = starting_location_fraction
        self.ending_location = ending_location_fraction
        self.profile = profile

# see: https://www.mathworks.com/help/robotics/ug/design-a-trajectory-with-velocity-limits-using-a-trapezoidal-velocity-profile.html
class SingleVariableTrapezoidalProfile(TransientVariableProfile):

    def __init__(self, start: float, end: float, value_space: RealNumberValueSpace = LinearUnboundedSpace()):
        self.value_space = value_space
        self.start = value_space.normalize_value(start)
        self.end = value_space.normalize_value(end)

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
        self.velocity = 1.5 * (self.end - self.start) / 1.0

        self.acceleration_phase_ratio = 1/3
        self.constant_phase_ratio = 1/3
        self.deceleration_phase_ratio = 1/3

    def first_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            velocity_due_to_acceleration = ((self.velocity - starting_velocity) / self.acceleration_phase_ratio) * time_fraction
            return starting_velocity + velocity_due_to_acceleration

        if time_fraction > (self.acceleration_phase_ratio + self.constant_phase_ratio):
            # deccelerating
            starting_velocity = self.velocity
            ending_velocity = 0.0
            velocity_due_to_acceleration = ((ending_velocity - self.velocity) / self.deceleration_phase_ratio) * (time_fraction - (self.acceleration_phase_ratio + self.constant_phase_ratio))
            return starting_velocity + velocity_due_to_acceleration

        return self.velocity

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            return (self.velocity - starting_velocity) / self.acceleration_phase_ratio

        if time_fraction > (self.acceleration_phase_ratio + self.constant_phase_ratio):
            # deccelerating
            ending_velocity = 0.0
            return (ending_velocity - self.velocity) / self.deceleration_phase_ratio

        return 0.0

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if math.isclose(0.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            starting_velocity = 0.0
            return (((self.velocity - starting_velocity) / self.acceleration_phase_ratio) - 0.0) / 0.01

        if math.isclose(time_fraction, self.acceleration_phase_ratio, rel_tol=1e-2, abs_tol=1e-2):
            starting_velocity = 0.0
            return (0.0 - ((self.velocity - starting_velocity) / self.acceleration_phase_ratio)) / 0.01

        if math.isclose(time_fraction, self.acceleration_phase_ratio + self.constant_phase_ratio, rel_tol=1e-2, abs_tol=1e-2):
            ending_velocity = 0.0
            return (((ending_velocity - self.velocity) / self.deceleration_phase_ratio) - 0.0) / 0.01

        if math.isclose(1.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            ending_velocity = 0.0
            return (0.0 - ((ending_velocity - self.velocity) / self.acceleration_phase_ratio)) / 0.01

        return 0.0

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start

        if time_fraction > 1.0:
            return self.end

        if time_fraction < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            distance_change_from_velocity = starting_velocity * time_fraction
            distance_change_from_acceleration = 0.5 * ((self.velocity - starting_velocity) / self.acceleration_phase_ratio) * time_fraction * time_fraction
            result = self.start + distance_change_from_velocity + distance_change_from_acceleration
            return self.value_space.normalize_value(result)

        distance_due_to_inital_acceleration = 0.5 * self.velocity * self.acceleration_phase_ratio
        if time_fraction > (self.acceleration_phase_ratio + self.constant_phase_ratio):
            # deccelerating
            distance_due_to_constant_velocity = self.velocity * self.constant_phase_ratio

            deceleration_time = time_fraction - (self.acceleration_phase_ratio + self.constant_phase_ratio)
            ending_velocity = 0.0
            distance_due_to_deceleration = self.velocity * deceleration_time + 0.5 * ((ending_velocity - self.velocity) / self.deceleration_phase_ratio) * deceleration_time * deceleration_time
            result = self.start + distance_due_to_inital_acceleration + distance_due_to_constant_velocity + distance_due_to_deceleration
            return self.value_space.normalize_value(result)

        result = self.start + distance_due_to_inital_acceleration + (time_fraction - self.acceleration_phase_ratio) * self.velocity
        return self.value_space.normalize_value(result)

# S-Curve profile
# --> controlled by the second derivative being linear
class SingleVariableSCurveProfile(TransientVariableProfile):


    def __init__(self, start: float, end: float, value_space: RealNumberValueSpace = LinearUnboundedSpace()):
        self.value_space = value_space
        self.start = value_space.normalize_value(start)
        self.end = value_space.normalize_value(end)

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
        # s = j * 10 / 512 * t
        #
        # j =  (s * 512) / (10 * t)
        self.jerk = 512 / 10 * (self.end - self.start) / 1.0

        self.positive_acceleration_phase_ratio = 1/8
        self.constant_acceleration_phase_ratio = 1/8
        self.negative_acceleration_phase_ratio = 1/8
        self.constant_phase_ratio = 1/4

        self.t1 = self.positive_acceleration_phase_ratio
        self.t2 = self.t1 + self.constant_acceleration_phase_ratio
        self.t3 = self.t2 + self.negative_acceleration_phase_ratio
        self.t4 = self.t3 + self.constant_phase_ratio
        self.t5 = self.t4 + self.positive_acceleration_phase_ratio
        self.t6 = self.t5  + self.constant_acceleration_phase_ratio
        self.t7 = 1.0

    def first_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.t1:
            return 0.5 * self.jerk * time_fraction * time_fraction

        a1 = self.jerk * self.t1
        v1 = 0.5 * a1 * self.t1
        if time_fraction < self.t2:
            return  v1 + a1 * (time_fraction - self.t1)

        a2 = a1
        v2 = v1 + a1 * (self.t2 - self.t1)
        if time_fraction < self.t3:
            return -0.5 * self.jerk * (time_fraction - self.t2) * (time_fraction - self.t2) + a2 * (time_fraction - self.t2) + v2

        v3 = -0.5 * self.jerk * (self.t3 - self.t2) * (self.t3 - self.t2) + a2 * (self.t3 - self.t2) + v2
        if time_fraction < self.t4:
            return v3

        if time_fraction < self.t5:
            return -0.5 * self.jerk * (time_fraction - self.t4) * (time_fraction - self.t4) + v3

        a5 = -self.jerk * (self.t5 - self.t4)
        v5 = -0.5 * self.jerk * (self.t5 - self.t4) * (self.t5 - self.t4) + v3
        if time_fraction < self.t6:
            return a5 * (time_fraction - self.t5) + v5

        a6 = a5
        v6 = a5 * (self.t6 - self.t5) + v5
        return 0.5 * self.jerk * (time_fraction - self.t6) * (time_fraction - self.t6) + a6 * (time_fraction - self.t6) + v6

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.t1:
            return self.jerk * time_fraction

        if time_fraction < self.t2:
            return self.jerk * self.t1

        if time_fraction < self.t3:
            return -self.jerk * (time_fraction - self.t2) + self.jerk * self.t1

        if time_fraction < self.t4:
            return 0.0

        if time_fraction < self.t5:
            return -self.jerk * (time_fraction - self.t4)

        if time_fraction < self.t6:
            return -self.jerk * (self.t5 - self.t4)

        return -self.jerk * (self.t5 - self.t4) + self.jerk * (time_fraction - self.t6)

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.t1:
            return self.jerk

        if time_fraction < self.t2:
            return 0.0

        if time_fraction < self.t3:
            return -self.jerk

        if time_fraction < self.t4:
            return 0.0

        if time_fraction < self.t5:
            return -self.jerk

        if time_fraction < self.t6:
            return 0.0

        return self.jerk

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start

        if time_fraction > 1.0:
            return self.end

        if time_fraction < self.t1:
            result = 1/6 * self.jerk * math.pow(time_fraction, 3.0) + self.start
            return self.value_space.normalize_value(result)

        a1 = self.jerk * self.t1
        v1 = 0.5 * a1 * self.t1
        s1 = 1/6 * self.jerk * math.pow(self.t1, 3.0) + self.start
        if time_fraction < self.t2:
            result =  v1 * (time_fraction - self.t1) + 0.5 * a1 * (time_fraction - self.t1) * (time_fraction - self.t1) + s1
            return self.value_space.normalize_value(result)

        a2 = a1
        v2 = v1 + a1 * (self.t2 - self.t1)
        s2 = v1 * (self.t2 - self.t1) + 0.5 * a1 * (self.t2 - self.t1) * (self.t2 - self.t1) + s1
        if time_fraction < self.t3:
            result = -1/6 * self.jerk * math.pow(time_fraction - self.t2, 3.0) + 0.5 * a2 * math.pow(time_fraction - self.t2, 2.0) + v2 * (time_fraction - self.t2) + s2
            return self.value_space.normalize_value(result)

        v3 = -0.5 * self.jerk * (self.t3 - self.t2) * (self.t3 - self.t2) + a1 * (self.t3 - self.t2) + v2
        s3 = -1/6 * self.jerk * math.pow(self.t3 - self.t2, 3.0) + 0.5 * a2 * math.pow(self.t3 - self.t2, 2.0) + v2 * (self.t3 - self.t2) + s2
        if time_fraction < self.t4:
            result = v3 * (time_fraction - self.t3) + s3
            return self.value_space.normalize_value(result)

        s4 = v3 * (self.t4 - self.t3) + s3
        if time_fraction < self.t5:
            result = -1/6 * self.jerk * math.pow(time_fraction - self.t4, 3.0) + v3 * (time_fraction - self.t4) + s4
            return self.value_space.normalize_value(result)

        a5 = -self.jerk * (self.t5 - self.t4)
        v5 = -0.5 * self.jerk * (self.t5 - self.t4) * (self.t5 - self.t4) + v3
        s5 = -1/6 * self.jerk * math.pow(self.t5 - self.t4, 3.0) + v3 * (self.t5 - self.t4) + s4
        if time_fraction < self.t6:
            result = 0.5 * a5 * math.pow(time_fraction - self.t5, 2.0) + v5 * (time_fraction - self.t5) + s5
            return self.value_space.normalize_value(result)

        a6 = a5
        v6 = a5 * (self.t6 - self.t5) + v5
        s6 = 0.5 * a5 * math.pow(self.t6 - self.t5, 2.0) + v5 * (self.t6 - self.t5) + s5
        result = 1/6 * self.jerk * math.pow(time_fraction - self.t6, 3.0) + 0.5 * a6 * math.pow(time_fraction - self.t6, 2.0) + v6 * (time_fraction - self.t6) + s6
        return self.value_space.normalize_value(result)

# 4th and 5th order s-curve
