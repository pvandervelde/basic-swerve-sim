# Single axis motion validation configurations

This folder contains configurations for simulations where the robot moves along a single axis,
e.g. linear moves along the x- or y-axis, or rotations around the z-axis. These simulations are
designed to validate that the simulation treats the positive and  negative directions of the
different axis.

The different simulation configurations are:

- `body_orientation_0_body_vel_angular_negative_dm_direction_rotation_inplace`
  - Robot is oriented in the x-direction
  - Robot is given a negative angular velocity
  - The drive modules are oriented such that the robot will turn around its own axis, i.e. the
    modules have their steering angles set to
    - front-left: 135 degrees
    - rear-left: 225 degrees
    - rear-right: 315 degrees
    - front-right: 45 degrees
- `body_orientation_0_body_vel_angular_positive_dm_direction_rotation_inplace`
  - Robot is oriented in the x-direction
  - Robot is given a positive angular velocity
  - The drive modules are oriented such that the robot will turn around its own axis, i.e. the
    modules have their steering angles set to
    - front-left: 135 degrees
    - rear-left: 225 degrees
    - rear-right: 315 degrees
    - front-right: 45 degrees
- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_0`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the negative x-direction
  - The drive modules are set to 0 degrees steering angle, i.e. facing in the robot positive
    x-axis direction
- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_90`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the negative y-direction
  - The drive modules are set to 90 degrees steering angle, i.e. facing in the robot positive
    y-axis direction
- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_180`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the negative x-direction
  - The drive modules are set to 180 degrees steering angle, i.e. facing in the robot negative
    x-axis direction
- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_270`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the negative y-direction
  - The drive modules are set to 270 degrees steering angle, i.e. facing in the robot negative
    y-axis direction
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_0`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the positive x-direction
  - The drive modules are set to 0 degrees steering angle, i.e. facing in the robot positive
    x-axis direction
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_90`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the positive y-direction
  - The drive modules are set to 90 degrees steering angle, i.e. facing in the robot positive
    y-axis direction
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_180`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the positive x-direction
  - The drive modules are set to 180 degrees steering angle, i.e. facing in the robot negative
    x-axis direction
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_270`
  - Robot is oriented in the x-direction
  - Robot is given a linear velocity in the positive y-direction
  - The drive modules are set to 270 degrees steering angle, i.e. facing in the robot negative
    y-axis direction
