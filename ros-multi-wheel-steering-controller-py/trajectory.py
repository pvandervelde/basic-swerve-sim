from typing import Mapping, List, Tuple

from control_model import BodyState, DriveModuleState


class LinearProfile(object):

    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end

    # Get Intermediate points

# A collection of position / velocity / acceleration profiles
class BodyStateTrajectory(object):

    def __init__(self, current: BodyState, desired: BodyState):
        self.profiles = []

    def waypoints_for_rate(self, rate: float) -> List[Tuple[float, BodyState]]:
        pass

class DriveModuleStateTrajectory(object):

    def __init__(self):
        self.profiles = []
