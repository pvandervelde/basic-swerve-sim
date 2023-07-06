from abc import ABC, abstractmethod
import math
from numpy.polynomial.polynomial import Polynomial
from typing import List, Tuple

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
    def inflection_points(self) -> List[ProfilePoint]:
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

    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end

    def first_derivative_at(self, time_fraction: float) -> float:
        return self.end - self.start

    def inflection_points(self) -> List[ProfilePoint]:
        return [
            ProfilePoint(
                0.0,
                self.start,
                self.first_derivative_at(0.0),
                self.second_derivative_at(0.0),
                self.third_derivative_at(0.0)
            ),
            ProfilePoint(
                1.0,
                self.end,
                self.first_derivative_at(1.0),
                self.second_derivative_at(1.0),
                self.third_derivative_at(1.0)
            )
        ]

    def second_derivative_at(self, time_fraction: float) -> float:
        # Linear profile. There should never be an acceleration
        return 0.0

    def third_derivative_at(self, time_fraction: float) -> float:
        return 0.0

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start

        if time_fraction > 1.0:
            return self.end

        return (self.end - self.start) * time_fraction + self.start

class SingleVariableCompoundProfileValue(object):

    def __init__(
        self,
        location_fraction: float,
        value: float
    ):
        self.location = location_fraction
        self.value = value

class SingleVariableMultiPointLinearProfile(TransientVariableProfile):

    def __init__(self, start: float, end: float):
        self.profiles: List[SingleVariableCompoundProfileValue] = [
            SingleVariableCompoundProfileValue(0.0, start),
            SingleVariableCompoundProfileValue(1.0, end)
        ]

        # We have two points (begin and end) so at best we can do a linear approach
        self.maximum_polynomial_order = 1

    def add_value(self, time_fraction: float, value: float):
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > 1.0:
            time_fraction = 1.0

        section = SingleVariableCompoundProfileValue(time_fraction,value)

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

        if time_fraction > 1.0:
            time_fraction = 1.0

        poly = self.polynomial_at_time(time_fraction)
        first_deriv = poly.deriv(1)
        return first_deriv(time_fraction)

    def inflection_points(self) -> List[ProfilePoint]:
        pass

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

        if time_fraction > 1.0:
            time_fraction = 1.0

        poly = self.polynomial_at_time(time_fraction)
        first_deriv = poly.deriv(2)
        return first_deriv(time_fraction)

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > 1.0:
            time_fraction = 1.0

        poly = self.polynomial_at_time(time_fraction)
        first_deriv = poly.deriv(3)
        return first_deriv(time_fraction)

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > 1.0:
            time_fraction = 1.0

        poly = self.polynomial_at_time(time_fraction)
        return poly(time_fraction)

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

class SingleVariableCompoundProfile(TransientVariableProfile):
    def __init__(self):
        self.profiles: List[CompoundProfileSection] = []

    def add_profile(self, starting_time_fraction: float, ending_time_fraction: float, profile: TransientVariableProfile):
        if starting_time_fraction > ending_time_fraction:
            raise InvalidTimeFractionException()

        if starting_time_fraction < 0.0:
            starting_time_fraction = 0.0

        if ending_time_fraction > 1.0:
            ending_time_fraction = 1.0

        section = CompoundProfileSection(starting_time_fraction, ending_time_fraction, profile)

        if len(self.profiles) == 0:
            self.profiles.append(section)
            return

        index = 0
        for i in range(len(self.profiles)):
            if self.profiles[i].ending_location <= starting_time_fraction:
                if i + 1 >= len(self.profiles):
                    # last profile
                    index = i + 1
                    break
                else:
                    # not the last profile. Go around the loop and we'll get it then
                    continue

            if self.profiles[i].starting_location >= ending_time_fraction:
                if i - 1 >= 0:
                    if self.profiles[i - 1].ending_location <= starting_time_fraction:
                        index = i
                        break
                    else:
                        raise InvalidTimeFractionException(f'The profile overlaps with existing profile at { starting_time_fraction }')
                else:
                    index = 0
                    break
            else:
                if self.profiles[i].ending_location <= ending_time_fraction:
                    continue
                else:
                    raise InvalidTimeFractionException(f'The profile overlaps with existing profile at { ending_time_fraction }')

        self.profiles.insert(index, section)

    def find_profile_for_time_fraction(self, time_fraction: float) -> CompoundProfileSection:
        # Find the profile that encompasses the given time_fraction. This profile will have a smaller start time
        # and a larger end time.
        #
        # It is possible that the time fraction is on the connection of two profiles. In that case we take the
        # profile which has a matching start.
        #
        # It is also possible that there is no profile for the given time_fraction. In that case we assume a linear
        # profile between existing profiles, or between the first profile and the start of time, or between the last
        # profile and the end of time.
        for i in range(len(self.profiles)):
            # If the start of the profile is smaller and the end is bigger then we have found the correct profile
            # Profiles are assumed to be inclusive of the start and exclusive of the end, except for the last profile
            if self.profiles[i].starting_location <= time_fraction:
                if self.profiles[i].ending_location > time_fraction:
                    return self.profiles[i]
                else:
                    # The end is either smaller or equal
                    if self.profiles[i].ending_location == time_fraction:
                        if i + 1 < len(self.profiles):
                            # not the last profile so the next profile should get it
                            continue
                        else:
                            # last profile, if this profile ends at the 1.0 fraction then
                            # return it. Otherwise assume a constant profile for the end
                            if self.profiles[i].ending_location == 1.0:
                                return self.profiles[i]
                            else:
                                value = self.profiles[i].profile.value_at(1.0)
                                return CompoundProfileSection(
                                    self.profiles[i].ending_location,
                                    1.0,
                                    SingleVariableLinearProfile(
                                        value,
                                        value
                                    ))
                    else:
                        # The end is smaller
                        if i < len(self.profiles) - 1:
                            # There are more profiles. The next one should get it
                            continue
                        else:
                            value = self.profiles[i].profile.value_at(1.0)
                            return CompoundProfileSection(
                                self.profiles[i].ending_location,
                                1.0,
                                SingleVariableLinearProfile(
                                    value,
                                    value
                                ))
            else:
                # Start is bigger and we haven't found a profile. Check if the end on the previous
                # profile is smaller. If so then we found a gap in our profiles
                if i - 1 >= 0:
                    return CompoundProfileSection(
                        self.profiles[i - 1].ending_location,
                        self.profiles[i].starting_location,
                        SingleVariableLinearProfile(
                            self.profiles[i - 1].profile.value_at(1.0),
                            self.profiles[i].profile.value_at(0.0)
                        ))
                else:
                    # The current profile is the first one. So make a profile, assuming linear
                    value = self.profiles[i].profile.value_at(0.0)
                    return CompoundProfileSection(
                        0.0,
                        self.profiles[i].starting_location,
                        SingleVariableLinearProfile(
                            value,
                            value
                        ))

        # We should never end here. But if we do, we're hosed
        return CompoundProfileSection(
            0.0,
            1.0,
            SingleVariableLinearProfile(
                0.0,
                0.0
            ))

    def first_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > 1.0:
            time_fraction = 1.0

        profile = self.find_profile_for_time_fraction(time_fraction)
        from_start = time_fraction - profile.starting_location
        total = profile.ending_location - profile.starting_location

        local_fraction = from_start / total
        return profile.profile.first_derivative_at(local_fraction)

    def inflection_points(self) -> List[ProfilePoint]:
        result: List[ProfilePoint] = []
        for profile in self.profiles:
            profile_inflection_points = profile.profile.inflection_points()
            for point in profile_inflection_points:
                time = profile.starting_location + point.time_fraction * (profile.ending_location - profile.starting_location)

                inflection_point = ProfilePoint(
                    time,
                    point.value,
                    point.first_derivative,
                    point.second_derivative,
                    point.third_derivative,
                )

                if result[-1].time_fraction == time:
                    result[-1] = inflection_point
                else:
                    result.append(inflection_point)

        return result

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > 1.0:
            time_fraction = 1.0

        profile = self.find_profile_for_time_fraction(time_fraction)
        from_start = time_fraction - profile.starting_location
        total = profile.ending_location - profile.starting_location

        local_fraction = from_start / total
        return profile.profile.second_derivative_at(local_fraction)

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > 1.0:
            time_fraction = 1.0

        profile = self.find_profile_for_time_fraction(time_fraction)
        from_start = time_fraction - profile.starting_location
        total = profile.ending_location - profile.starting_location

        local_fraction = from_start / total
        return profile.profile.third_derivative_at(local_fraction)

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            time_fraction = 0.0

        if time_fraction > 1.0:
            time_fraction = 1.0

        profile = self.find_profile_for_time_fraction(time_fraction)
        from_start = time_fraction - profile.starting_location
        total = profile.ending_location - profile.starting_location

        local_fraction = from_start / total
        return profile.profile.value_at(local_fraction)

# see: https://www.mathworks.com/help/robotics/ug/design-a-trajectory-with-velocity-limits-using-a-trapezoidal-velocity-profile.html
class SingleVariableTrapezoidalProfile(TransientVariableProfile):


    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end

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
        self.velocity = 1.5 * (end - start) / 1.0

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

    def inflection_points(self) -> List[ProfilePoint]:
        return [
            ProfilePoint(
                0.0,
                self.start,
                self.first_derivative_at(0.0),
                self.second_derivative_at(0.0),
                self.third_derivative_at(0.0)
            ),
            ProfilePoint(
                self.acceleration_phase_ratio,
                self.value_at(self.acceleration_phase_ratio),
                self.first_derivative_at(self.acceleration_phase_ratio),
                self.second_derivative_at(self.acceleration_phase_ratio),
                self.third_derivative_at(self.acceleration_phase_ratio)
            ),
            ProfilePoint(
                self.acceleration_phase_ratio + self.constant_phase_ratio,
                self.value_at(self.acceleration_phase_ratio + self.constant_phase_ratio),
                self.first_derivative_at(self.acceleration_phase_ratio + self.constant_phase_ratio),
                self.second_derivative_at(self.acceleration_phase_ratio + self.constant_phase_ratio),
                self.third_derivative_at(self.acceleration_phase_ratio + self.constant_phase_ratio)
            ),
            ProfilePoint(
                1.0,
                self.end,
                self.first_derivative_at(1.0),
                self.second_derivative_at(1.0),
                self.third_derivative_at(1.0)
            )
        ]

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
            return self.start + distance_change_from_velocity + distance_change_from_acceleration

        distance_due_to_inital_acceleration = 0.5 * self.velocity * self.acceleration_phase_ratio
        if time_fraction > (self.acceleration_phase_ratio + self.constant_phase_ratio):
            # deccelerating
            distance_due_to_constant_velocity = self.velocity * self.constant_phase_ratio

            deceleration_time = time_fraction - (self.acceleration_phase_ratio + self.constant_phase_ratio)
            ending_velocity = 0.0
            distance_due_to_deceleration = self.velocity * deceleration_time + 0.5 * ((ending_velocity - self.velocity) / self.deceleration_phase_ratio) * deceleration_time * deceleration_time
            return self.start + distance_due_to_inital_acceleration + distance_due_to_constant_velocity + distance_due_to_deceleration

        return self.start + distance_due_to_inital_acceleration + (time_fraction - self.acceleration_phase_ratio) * self.velocity

# S-Curve profile
# --> controlled by the second derivative being linear

# other jerk limited profile
