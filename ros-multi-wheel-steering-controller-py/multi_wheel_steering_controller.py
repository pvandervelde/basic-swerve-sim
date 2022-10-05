# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import Mapping

class ControllerParameter(object):

    def __init__(self, name: str, param_type: Parameter.Type, value):
        self.name = name
        self.param_type = param_type
        self.value = value

class WheelModule(object):

    def __init__(self):
        self.steering_link_name = ''
        self.driving_link_name = ''

        # Assume a vertical steering axis that goes through the center of the wheel (i.e. no steering offset)
        self.steering_axis_xy_position = []
        self.wheel_radius = 0.0

        self.steering_motor_maximum_velocity = 0.0

        self.steering_motor_minimum_acceleration = 0.0
        self.steering_motor_maximum_acceleration = 0.0

        self.drive_motor_maximum_velocity = 0.0

        self.drive_motor_minimum_acceleration = 0.0
        self.drive_motor_maximum_acceleration = 0.0

class MultiWheelSteeringController(Node):

    def __init__(self):
        super().__init__('multi_wheel_steering_controller')

        # Get all the configuration settings
        self.parameters = self.get_configuration()

        # Subscriptions
        # Subscribe to 'cmd_vel' to receive velocity / direction commands
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publishers
        wheel_steering_publish_topic = "/" + self.wheel_steering_controller_name + "/" + "joint_trajectory"
        self.pub_wheel_steering = self.create_publisher(JointTrajectory, wheel_steering_publish_topic, 1)

        wheel_velocity_publish_topic = "/" + self.wheel_velocity_controller_name + "/" + "commands"
        self.pub_wheel_velocity = self.create_publisher(Float64MultiArray, wheel_velocity_publish_topic, 1)

        # Get the geometry for the robot
        self.modules = []

        # Get the current steering angles
        self.steering_angles = []

        # Create the timer that we use to send controller messages
        self.timer = self.create_timer(1.0/self.publish_rate_in_hz, self.publish_controller_commands)

    def cmd_vel_callback(self):
        # Subscribe to cmd_vel --> Twist message
        # Store last message
        # Timer to decide if we're going to stop. Stopping x seconds after receiving last message with no new one
        pass

    def get_configuration(self) -> Mapping[str, ControllerParameter]:
        result = {}

        param_name = "wheel_steering_controller_name"
        result[param_name] = self.get_parameter_controller_name(
            controller_name = param_name,
            description = "The name of the controller that handles the steering position commands.",
            default_value = "position_trajectory_controller",
        )

        param_name = "wheel_velocity_controller_name"
        result[param_name] = self.get_parameter_controller_name(
            controller_name = param_name,
            description = "The name of the controller that handles the wheel velocity commands.",
            default_value = "velocity_controller",
        )

        publish_rate_parameter = self.get_parameter_publish_rate()
        result[publish_rate_parameter.name] = publish_rate_parameter

        # module information
        module_names_parameter = self.get_parameter_module_names()
        result[module_names_parameter.name] = module_names_parameter

        for name in module_names_parameter.value:
            # modules:
            #   - right_front:
            #       - steering_link: ""
            #         drive_link: ""
            #         xy_position: []
            #         wheel_radius: 0.04
            #         steering_motor_velocity_limits: [0.05, 0.5]
            #         steering_motor_acceleration_limits: [0.05, 0.5]
            #         drive_motor_velocity_limits: [0.05, 0.5]
            #         drive_motor_acceleration_limits: [0.05, 0.5]
            steering_link_parameter = self.get_parameter_module_parameter(
                module = name,
                name = "steering_link",
                description = "",
                type = Parameter.Type.STRING)
            result[steering_link_parameter.name] = steering_link_parameter

            self.declare_parameter("steering_motors_minimum_accelerations", [])
            self.steering_motors_minimum_acceleration = self.get_parameter("steering_motors_minimum_accelerations").value

            self.declare_parameter("steering_motors_maximum_accelerations", [])
            self.steering_motors_maximum_acceleration = self.get_parameter("steering_motors_maximum_accelerations").value

            self.declare_parameter("drive_motors_minimum_accelerations", [])
            self.drive_motors_minimum_acceleration = self.get_parameter("drive_motors_minimum_accelerations").value

            self.declare_parameter("drive_motors_maximum_accelerations", [])
            self.drive_motors_maximum_acceleration = self.get_parameter("drive_motors_maximum_accelerations").value

        # coordinates for each steering axis. Assume that the axis is vertical and runs through the center of the wheel
        # so that there's no offset

        # Make sure we're notified when one of our parameters changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        return result

    def get_parameter_controller_name(self, controller_name: str, description: str, default_value: str) -> ControllerParameter:
        descriptor = ParameterDescriptor(
            name = controller_name,
            description = description,
            type = Parameter.Type.STRING,
            read_only = True,
        )
        self.declare_parameter(
            controller_name,
            default_value,
            descriptor)
        value = self.get_parameter(controller_name).value
        if value is None or "".__eq__(value.strip()):
            pass

        return ControllerParameter(
            name = controller_name,
            param_type = Parameter.Type.STRING,
            value = value,
        )

    def get_parameter_module_names(self) -> ControllerParameter:
        param_name = "module_names"
        descriptor = ParameterDescriptor(
            name = param_name,
            description = "The names of the different drive modules.",
            type = Parameter.Type.STRING_ARRAY,
            read_only = True,
        )

        self.declare_parameter(param_name, [], descriptor)
        value = self.get_parameter(param_name).value

        return ControllerParameter(
            name = param_name,
            param_type = Parameter.Type.STRING_ARRAY,
            value = value
        )

    def get_parameter_module_parameter(self, module: str, name: str, description: str, type) -> ControllerParameter:
        param_name = f"modules.{module}.{name}"
        descriptor = ParameterDescriptor(
            name = param_name,
            description = description,
            type = Parameter.Type.STRING_ARRAY,
            read_only = True,
        )

        self.declare_parameter(param_name, [], descriptor)
        value = self.get_parameter(param_name).value

        return ControllerParameter(
            name = param_name,
            param_type = Parameter.Type.STRING_ARRAY,
            value = value
        )

    def get_parameter_publish_rate(self) -> ControllerParameter:
        param_name = "publish_rate_in_hz"
        descriptor = ParameterDescriptor(
            name = param_name,
            description = "The rate (in Hz) at which new updates are published to the controllers.",
            type = Parameter.Type.INTEGER,
            read_only = True,
        )

        # For now assume 10Hz, probably need at least 25 or 50
        self.declare_parameter(param_name, 10.0, descriptor)
        value = self.get_parameter(param_name).value

        return ControllerParameter(
            name = param_name,
            param_type = Parameter.Type.INTEGER,
            value = value
        )

    def on_parameters_change(self, params):
        for param in params:
            if not self.parameters.has_key(param.name):
                return SetParametersResult(
                    successful = False,
                    reason = f"No parameter with name { param.name } known. Cannot set parameter.")

            stored_param = self.parameters[param.name]
            if param.type_ not in [stored_param.param_type]:
                return SetParametersResult(
                    successful = False,
                    reason = f"Parameter with name { param.name } is of type { stored_param.param_type }. Instead got a parameter with type { param.type_ }.")

            stored_param.value = param.value

            # If it's part of a module we also need to update the module information

        return SetParametersResult(successful=True)

    def publish_controller_commands(self):

        # compute the wheel velocity and steering angle targets
        for module in self.modules:
            #    * compute wheel velocity and steering angle
            #    * based on the current steering angle (and steering velocity / acceleration) determine the trajectory for the steering goal
            #    * based on the current wheel velocity determine the trajectory for the wheel velocity goal
            #      --> Keep in mind velocity flip, and keep velocity, acceleration and jerk under control
            pass

        #    * Match trajectories between modules so that they all span the same time
        #    * Apply limits and scale / recalculate trajectories --> goal is to keep the center of rotation the same for all modules
        #    * Split trajectory in pieces and linearize
        #    * Send to hardware

        steering_trajectories = []
        velocity_trajectories = []

        # Kinematics
        # Literature:
        # - https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383/5
        # -
        # For wheel i
        #  - velocity = sqrt( (v_x - omega * y_i)^2 + (v_y + omega * x_i)^2 )
        #  - angle = acos( (v_x - omega * y_i) / (velocity) ) = asin( (v_y + omega * x_i) / (velocity) )
        #
        # Angle: 0 < alpha < Pi
        #  The angle also needs a differentiation if it should go between Pi and 2Pi
        #
        # This assumes that (x_i, y_i) is the coordinate for the steering axis. And that the steering axis is in z-direction.
        # And that the wheel contact point is on that steering axis

        # Deal with singularities, e.g. rotation around a wheel
        pass

    def steering_state_callback(self):
        # Store the current steering angle of the steering module
        # Store the current steering velocity of the steering module
        pass

    def velocity_state_callback(self):
        # store the current wheel position for the wheel module
        # store the current velocity of the wheel module
        pass


def main(args=None):
    rclpy.init(args=args)

    controller = MultiWheelSteeringController()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()