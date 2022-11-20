#!/usr/bin/python3

from math import acos, cos, degrees, isclose, pow, radians, sin, sqrt, tan
from turtle import forward

import numpy as np
from typing import Mapping, List, Tuple

# ROS types
import rclpy
from rclpy.node import Node

# ROS message types
from builtin_interfaces.msg import Duration
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import Point, PoseWithCovariance, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# local
from control_model import BodyState, DriveModuleState, body_state_from_twist, ControlModelBase, SimpleFourWheelSteeringControlModel
from drive_module import DriveModule
from trajectory import BodyStateTrajectory, DriveModuleStateTrajectory

class ControllerParameter(object):

    def __init__(
        self,
        name: str,
        param_type: Parameter.Type,
        value):
        self.name = name
        self.param_type = param_type
        self.value = value

class MultiWheelSteeringControllerROS(Node):

    # CLASS CONSTANTS
    PARAM_WHEEL_STEERING_CONTROLLER_NAME = "wheel_steering_controller_name"
    PARAM_WHEEL_VELOCITY_CONTROLLER_NAME = "wheel_velocity_controller_name"
    PARAM_PUBLISH_RATE_IN_HZ = "publish_rate_in_hz"

    PARAM_MODULE_STEERING_LINK = "steering_link"
    PARAM_MODULE_DRIVE_LINK = "drive_link"
    PARAM_MODULE_XY_POSITION = "xy_position"
    PARAM_MODULE_WHEEL_RADIUS = "wheel_radius"
    PARAM_MODULE_STEERING_MOTOR_VELOCITY_LIMITS = "steering_motor_velocity_limits"
    PARAM_MODULE_STEERING_MOTOR_ACCELERATION_LIMITS = "steering_motor_acceleration_limits"
    PARAM_MODULE_DRIVE_MOTOR_VELOCITY_LIMITS = "drive_motor_velocity_limits"
    PARAM_MODULE_DRIVE_MOTOR_ACCELERATION_LIMITS = "drive_motor_acceleration_limits"

    def __init__(self):
        super().__init__('multi_wheel_steering_controller')

        # Get all the configuration settings
        self.parameters = self.get_configuration()

        # Get the geometry for the robot
        self.modules = self.load_modules()

        # Use a simple control model for the time being. Just need something that roughly works
        self.control_model = SimpleFourWheelSteeringControlModel(self.modules)

        # Subscriptions
        # Subscribe to 'cmd_vel' to receive velocity / direction commands
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.sub_dynamic_joint_states = self.create_subscription(
            DynamicJointState,
            'dynamic_joint_states',
            self.dynamic_joint_states_callback,
            10)

        # Publishers
        wheel_steering_publish_topic = "/" + self.wheel_steering_controller_name + "/" + "joint_trajectory"
        self.pub_wheel_steering = self.create_publisher(JointTrajectory, wheel_steering_publish_topic, 1)

        wheel_velocity_publish_topic = "/" + self.wheel_velocity_controller_name + "/" + "commands"
        self.pub_wheel_velocity = self.create_publisher(Float64MultiArray, wheel_velocity_publish_topic, 1)

        odometry_publish_topic = "odom"
        self.pub_odometry = self.create_publisher(Odometry, odometry_publish_topic, 10)

        # Create the timer that we use to send controller messages
        self.timer = self.create_timer(1.0/self.parameters[self.PARAM_PUBLISH_RATE_IN_HZ].value, self.publish_controller_commands)

    def cmd_vel_callback(self, message: Twist):
        self.current_velocity_command = message

    def dynamic_joint_states_callback(self, message: DynamicJointState):
        data = {}
        joint_names = message.joint_names

        interface_values: InterfaceValue
        for i, interface_values in enumerate(message.interface_values):
            names = interface_values.interface_names
            values = interface_values.values

            value_map = {}
            for j, name in names:
                value_map[name] = values[j]

            data[joint_names[i]] = value_map

        module: DriveModule
        for module in self.modules:
            steering_link = module.steering_link_name
            if steering_link in data:
                value_map = data[steering_link]
                module.set_current_steering_direction_in_radians(value_map['position'])
                module.set_current_steering_velocity_in_radians_per_second(value_map['velocity'])

            drive_link = module.driving_link_name
            if drive_link in data:
                value_map = data[drive_link]
                module.set_current_drive_velocity_in_meters_per_second(value_map['velocity'])

    def get_configuration(self) -> Mapping[str, ControllerParameter]:
        result = {}

        param_name = self.PARAM_WHEEL_STEERING_CONTROLLER_NAME
        result[param_name] = self.get_parameter_controller_name(
            controller_name = param_name,
            description = "The name of the controller that handles the steering position commands.",
            default_value = "position_trajectory_controller",
        )

        param_name = self.PARAM_WHEEL_VELOCITY_CONTROLLER_NAME
        result[param_name] = self.get_parameter_controller_name(
            controller_name = param_name,
            description = "The name of the controller that handles the wheel velocity commands.",
            default_value = "velocity_controller",
        )

        publish_rate_parameter = self.get_parameter_publish_rate()
        result[publish_rate_parameter.name] = publish_rate_parameter

        module_names_parameter = self.get_parameter_module_names()
        result[module_names_parameter.name] = module_names_parameter

        for name in module_names_parameter.value:
            steering_link_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_STEERING_LINK,
                description = f"The name of the {name} steering link in the URDF file.",
                type = Parameter.Type.STRING,
                default_value = "")
            result[steering_link_parameter.name] = steering_link_parameter

            drive_link_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_DRIVE_LINK,
                description = f"The name of the {name} drive link in the URDF file.",
                type = Parameter.Type.STRING,
                default_value = "")
            result[drive_link_parameter.name] = drive_link_parameter

            xy_position_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_XY_POSITION,
                description = f"The (x, y, z) coordinates of the {name} module relative to the navigation coordinate system (generally base_link).",
                type = Parameter.Type.DOUBLE_ARRAY,
                default_value = [0.0, 0.0, 0.0])
            result[xy_position_parameter.name] = xy_position_parameter

            wheel_radius_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_WHEEL_RADIUS,
                description = f"The radius of the wheel attached to the {name} module.",
                type = Parameter.Type.DOUBLE,
                default_value = 0.1)
            result[wheel_radius_parameter.name] = wheel_radius_parameter

            steering_motor_velocity_limits_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_STEERING_MOTOR_VELOCITY_LIMITS,
                description = f"The minimum and maximum velocity in radians per second for the steering motor of the {name} module. It is assumed the motor will turn both forwards and backwards.",
                type = Parameter.Type.DOUBLE_ARRAY,
                default_value = [0.01, 1.0])
            result[steering_motor_velocity_limits_parameter.name] = steering_motor_velocity_limits_parameter

            steering_motor_acceleration_limits_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_STEERING_MOTOR_ACCELERATION_LIMITS,
                description = f"The minimum and maximum acceleration in radians per second squared for the steering motor of the {name} module. It is assumed the motor will turn both forwards and backwards.",
                type = Parameter.Type.DOUBLE_ARRAY,
                default_value = [0.01, 1.0])
            result[steering_motor_acceleration_limits_parameter.name] = steering_motor_acceleration_limits_parameter

            drive_motor_velocity_limits_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_DRIVE_MOTOR_VELOCITY_LIMITS,
                description = f"The minimum and maximum velocity in meters per second for the drive motor of the {name} module. It is assumed the motor will turn both forwards and backwards.",
                type = Parameter.Type.DOUBLE_ARRAY,
                default_value = [0.01, 1.0])
            result[drive_motor_velocity_limits_parameter.name] = drive_motor_velocity_limits_parameter

            drive_motor_acceleration_limits_parameter = self.get_parameter_module_parameter(
                module = name,
                name = self.PARAM_MODULE_DRIVE_MOTOR_ACCELERATION_LIMITS,
                description = f"The minimum and maximum acceleration in meters per second squared for the drive motor of the {name} module. It is assumed the motor will turn both forwards and backwards.",
                type = Parameter.Type.DOUBLE_ARRAY,
                default_value = [0.01, 1.0])
            result[drive_motor_acceleration_limits_parameter.name] = drive_motor_acceleration_limits_parameter

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

    def get_parameter_module_parameter(self, module: str, name: str, description: str, type, default_value) -> ControllerParameter:
        param_name = f"modules.{module}.{name}"
        descriptor = ParameterDescriptor(
            name = param_name,
            description = description,
            type = type,
            read_only = True,
        )

        self.declare_parameter(param_name, default_value, descriptor)
        value = self.get_parameter(param_name).value

        return ControllerParameter(
            name = param_name,
            param_type = type,
            value = value
        )

    def get_parameter_publish_rate(self) -> ControllerParameter:
        param_name = self.PARAM_PUBLISH_RATE_IN_HZ
        descriptor = ParameterDescriptor(
            name = param_name,
            description = "The rate (in Hz) at which new updates are published to the controllers.",
            type = Parameter.Type.INTEGER,
            read_only = True,
        )

        # For now assume 25Hz, might need 50Hz or maybe even 100Hz.
        # A good measure is to figure out how long it takes the motors to respond to a command
        # and what the PWM frequency is.
        self.declare_parameter(param_name, 25.0, descriptor)
        value = self.get_parameter(param_name).value

        return ControllerParameter(
            name = param_name,
            param_type = Parameter.Type.INTEGER,
            value = value
        )

    def on_parameters_change(self, params):
        has_module_changes: bool = False
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

            if param.name.startswith('modules.'):
                has_module_changes = True

            stored_param.value = param.value

        if has_module_changes:
            self.modules = self.load_modules()

        return SetParametersResult(successful=True)

    def load_modules(self) -> List[DriveModule]:
        module_elements = {}
        for key, value in self.parameters.items():
            if key.startswith('modules.'):
                if key.count('.') > 2:
                    # error here, there should only be three sections in a module parameter name
                    pass

                # Split the parameter into sections:
                # [0]: 'modules'
                # [1]: <module_name>
                # [2]: <module_parameter>
                _, module_name, module_parameter = key.split(".")

                if module_name not in module_elements:
                    module_elements[module_name] = {}

                module_elements[module_name][module_parameter] = value

        result = []
        for key, value in module_elements:
            steering_axis_coordinates = Point(value['steering_axis_xy_position'])
            point = Point(
                steering_axis_coordinates[0],
                steering_axis_coordinates[1],
                steering_axis_coordinates[2],
            )
            wheel_module = DriveModule(
                name = key,
                steering_link = value['steering_link'],
                drive_link = value['drive_link'],
                steering_axis_xy_position = point,
                wheel_radius = value['wheel_radius'],
                steering_motor_maximum_velocity = value['steering_motor_velocity_limits'][1],
                steering_motor_minimum_acceleration = value['steering_motor_acceleration_limits'][0],
                steering_motor_maximum_acceleration = value['steering_motor_acceleration_limits'][1],
                drive_motor_maximum_velocity = value['drive_motor_velocity_limits'][1],
                drive_motor_minimum_acceleration = value['drive_motor_acceleration_limits'][0],
                drive_motor_maximum_acceleration = value['drive_motor_acceleration_limits'][1]
            )
            result.append(wheel_module)
        return result

    # Do stuff
    def publish_controller_commands(self):

        # The following procedure assumes that all the wheels are pointing to the same ICR. Ideally
        # there would be a node that watches the current state and stops the robot if something goes
        # wrong.

        # Compute the twist trajectory, i.e. how do we get from our current (v, omega) to the
        # desired (v, omega)
        #   Ideally we want O(1) for jerk, O(2) for acceleration, O(3) for velocity
        #   Can use a spline or b-spline or other trajectory approach
        current_body_state = self.control_model.state_of_body_frame_from_wheel_module_states()
        desired_body_state = body_state_from_twist(self.current_velocity_command)

        # Note: We recalculate the trajectory at this stage so that we use the current snapshot of what is
        # going on. If we were to have a trajectory that is re-used we will have to safe guard the updates
        # because the updates would come in on different threads.
        body_trajectory = BodyStateTrajectory()
        body_trajectory.update_current_state(current_body_state)
        body_trajectory.update_desired_state(desired_body_state)

        # use the twist trajectory to compute the state for the steering modules for the end state
        # and several intermediate points, i.e. determine the vector [[v_i];[gamma_i]].
        #    Use Seegmiller and Kelly to compute the desired velocities and angles
        #
        #    Keep in mind that our update rate determines the points in time where we can do something
        #
        #    Also keep in mind that steering the wheel effectively changes the velocity of the wheel
        #    if we use a co-axial system
        drive_module_trajectory = DriveModuleStateTrajectory(self.modules)
        drive_module_trajectory.set_current_state()
        drive_module_trajectory.set_desired_end_state(
            self.control_model.state_of_wheel_modules_from_body_state(desired_body_state)
        )

        # Correct for motor capabilities (max velocity, max acceleration, deadband etc.)
        # and make sure that the trajectories of the different modules all span the same time frame
        drive_module_trajectory.align_module_profiles()

        # Send commands to the hardware
        steering_trajectories = []
        velocity_trajectories = []

        for drive_module in self.modules:
            module_id = drive_module.name
            profile = drive_module_trajectory.profiles_for_module(module_id)

            # We need the trajectory way point for the next point in time. Assuming that
            # the first point on the trajectory is the current velocity, then we want the second
            # point on the trajectory.
            steering_trajectories.append(profile.steering_angle_for_waypoint_at(1))
            velocity_trajectories.append(profile.velocity_for_waypoint_at(1))

        self.publish_module_steering_angles()
        self.publish_module_velocities(velocity_trajectories)
        self.publish_odometry(current_body_state)

    def publish_module_steering_angles(self, steering_angles: List[float]):
        traj = JointTrajectory()
        #traj.header
        traj.joint_names = [drive_module.steering_link for drive_module in self.modules]
        for angle in steering_angles:
            point = JointTrajectoryPoint()
            point.positions = angle
            point.time_from_start = Duration(sec = 0) # Immediate execute --> Technically we want the motor driver to spread the movement out over the current period of the update rate

            traj.points.append(point)
        self.pub_wheel_steering.publish(traj)

    def publish_module_velocities(self, wheel_velocities: List[float]):
        wheel_velocity_message = Float64MultiArray()
        wheel_velocity_message.data = wheel_velocities
        self.pub_wheel_velocity.publish(wheel_velocity_message)

    def publish_odometry(self, current_body_state: BodyState):

        current_time = rospy.Time.now()

        x, y = scuttle.get_global_position()
        velocity_x, velocity_theta = scuttle.get_motion()
        th = scuttle.get_heading()

        odometry_quaternion = tf.transformations.quaternion_from_euler(0, 0, th)

        odometry = Odometry()
        odometry.header.stamp = current_time
        odometry.header.frame_id = "odom"

        odometry.pose.pose = PoseWithCovariance(Point(x, y, 0.), Quaternion(*odometry_quaternion))
        # Since some ROS nodes expect non-zero pose covariance,
        # set non-zero default pose covariance matrix.
        odometry.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                0, 0.01, 0, 0, 0, 0,
                                0, 0, 0.01, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1]

        odometry.child_frame_id = "base_link"
        odometry.twist.twist = Twist(Vector3(0, velocity_x, 0), Vector3(0, 0, 0))

        odom_message = Odometry()
        odom_message.pose = None
        odom_message.twist = None

        self.pub_odometry.publish(odom_message)

def main(args=None):
    rclpy.init(args=args)

    controller = MultiWheelSteeringControllerROS()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()