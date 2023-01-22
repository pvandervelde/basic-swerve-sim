from abc import ABC, abstractmethod
from typing import List

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

class TransientValueProfile(ABC):

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

class LinearProfile(TransientValueProfile):

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

class CompoundProfileSection(object):

    def __init__(
        self,
        starting_location_fraction: float,
        ending_location_fraction: float,
        profile: TransientValueProfile
    ):
        self.starting_location = starting_location_fraction
        self.ending_location = ending_location_fraction
        self.profile = profile


class CompoundProfile(TransientValueProfile):
    def __init__(self):
        self.profiles: List[CompoundProfileSection] = []

    def add_profile(self, starting_time_fraction: float, ending_time_fraction: float, profile: TransientValueProfile):
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
                                    LinearProfile(
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
                                LinearProfile(
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
                        LinearProfile(
                            self.profiles[i - 1].profile.value_at(1.0),
                            self.profiles[i].profile.value_at(0.0)
                        ))
                else:
                    # The current profile is the first one. So make a profile, assuming linear
                    value = self.profiles[i].profile.value_at(0.0)
                    return CompoundProfileSection(
                        0.0,
                        self.profiles[i].starting_location,
                        LinearProfile(
                            value,
                            value
                        ))

        # We should never end here. But if we do, we're hosed
        return CompoundProfileSection(
            0.0,
            1.0,
            LinearProfile(
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
