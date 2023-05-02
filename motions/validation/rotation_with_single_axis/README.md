# Rotation with single axis motion validation configurations

This folder contains configurations for simulations where the robot rotates and then moves along
a single axis, e.g. a rotation to face in the world y-direction followed by a linear move. These
simulations are designed to validate that the simulation treats the coordinate rotations correctly.

The different simulation configurations are:

- `body_orientation_90_body_vel_linear_negative_dm_direction_linear_-90`
  - Robot is rotated so that the robot x-axis is facing in the world y-direction, i.e. a 90 degree turn
  - Robot is given a negative linear velocity in the robot y-direction
  - The drive modules are set to -90 degrees steering angle, i.e. facing in the robot negative
    y-axis direction
- `body_orientation_90_body_vel_linear_positive_dm_direction_linear_-90`
  - Robot is rotated so that the robot x-axis is facing in the world y-direction, i.e. a 90 degree turn
  - Robot is given a positive linear velocity in the robot y-direction
  - The drive modules are set to -90 degrees steering angle, i.e. facing in the robot negative
    y-axis direction

- `body_orientation_90_body_vel_linear_negative_dm_direction_linear_0`
  - Robot is rotated so that the robot x-axis is facing in the world y-direction, i.e. a 90 degree turn
  - Robot is given a negative linear velocity in the robot x-direction
  - The drive modules are set to 0 degrees steering angle, i.e. facing in the robot positive
    x-axis direction
- `body_orientation_90_body_vel_linear_positive_dm_direction_linear_0`
  - Robot is rotated so that the robot x-axis is facing in the world y-direction, i.e. a 90 degree turn
  - Robot is given a positive linear velocity in the robot x-direction
  - The drive modules are set to 0 degrees steering angle, i.e. facing in the robot positive
    x-axis direction
