from abc import ABC, abstractmethod
from typing import Mapping, List, Tuple

# local
from .control_model import ControlModelBase
from .geometry import Vector3
from .states import BodyState, DriveModuleDesiredValues, BodyMotion

class MotionCommand(ABC):

    # Determine what the body state would be if the robot would execute the current
    # motion command.
    @abstractmethod
    def to_body_state(self, model: ControlModelBase) -> BodyMotion:
        pass

    # Determine what the state of the drive modules would be if the robot would execute
    # the current motion command.
    @abstractmethod
    def to_drive_module_state(self, model: ControlModelBase) -> List[DriveModuleDesiredValues]:
        pass

# Defines a motion command that specifies a motion from the robot body perspective
class BodyMotionCommand(MotionCommand):

    def __init__(
        self,
        linear_x_velocity_in_meters_per_second: float,
        linear_y_velocity_in_meters_per_second: float,
        angular_z_velocity_in_radians_per_second: float
        ):
        self.linear_velocity = Vector3(linear_x_velocity_in_meters_per_second, linear_y_velocity_in_meters_per_second, 0.0)
        self.angular_velocity = Vector3(0.0, 0.0, angular_z_velocity_in_radians_per_second)

    # Determine what the body state would be if the robot would execute the current
    # motion command.
    def to_body_state(self, model: ControlModelBase) -> BodyMotion:
        return BodyMotion(self.linear_velocity.x, self.linear_velocity.y, self.angular_velocity.z)

    # Determine what the state of the drive modules would be if the robot would execute
    # the current motion command.
    def to_drive_module_state(self, model: ControlModelBase) -> List[DriveModuleDesiredValues]:

        # We get both the forward and reverse options here. We should see which is the better one
        # For now just use the forward one
        drive_module_potential_states = model.state_of_wheel_modules_from_body_motion(
            BodyMotion(
                self.linear_velocity.x,
                self.linear_velocity.y,
                self.angular_velocity.z,
            ))
        return [x[0] for x in drive_module_potential_states]

# Defines a motion command from the robot drive module perspective
class DriveModuleMotionCommand(MotionCommand):

    def __init__(
        self,
        desired_states: List[DriveModuleDesiredValues]):
        self.desired_states = desired_states

    # Determine what the body state would be if the robot would execute the current
    # motion command.
    def to_body_state(self, model: ControlModelBase) -> BodyMotion:
        return model.body_motion_from_wheel_module_states(self.desired_states)

    # Determine what the state of the drive modules would be if the robot would execute
    # the current motion command.
    def to_drive_module_state(self, model: ControlModelBase) -> List[DriveModuleDesiredValues]:
        return self.desired_states
