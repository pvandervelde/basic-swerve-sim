# Combined axis motion verification configurations

This folder contains configurations for simulations where the robot moves along multiple axis
at the same time. These simulations are designed to validate that the simulation treats combinations
of the different axis correctly.

The different simulation configurations are:

- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_30`
  - Robot is oriented in the x-direction
  - Robot is given a negative linear velocity in a 30 degree direction
  - The drive modules are set to 30 degrees steering angle
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_30`
  - Robot is oriented in the x-direction
  - Robot is given a positive linear velocity in a 30 degree direction
  - The drive modules are set to 30 degrees steering angle

- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_60`
  - Robot is oriented in the x-direction
  - Robot is given a negative linear velocity in a 60 degree direction
  - The drive modules are set to 60 degrees steering angle
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_60`
  - Robot is oriented in the x-direction
  - Robot is given a positive linear velocity in a 60 degree direction
  - The drive modules are set to 60 degrees steering angle

- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_120`
  - Robot is oriented in the x-direction
  - Robot is given a negative linear velocity in a 120 degree direction
  - The drive modules are set to 120 degrees steering angle
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_120`
  - Robot is oriented in the x-direction
  - Robot is given a positive linear velocity in a 120 degree direction
  - The drive modules are set to 120 degrees steering angle

- `body_orientation_0_body_vel_linear_negative_dm_direction_linear_150`
  - Robot is oriented in the x-direction
  - Robot is given a negative linear velocity in a 150 degree direction
  - The drive modules are set to 150 degrees steering angle
- `body_orientation_0_body_vel_linear_positive_dm_direction_linear_150`
  - Robot is oriented in the x-direction
  - Robot is given a positive linear velocity in a 150 degree direction
  - The drive modules are set to 150 degrees steering angle

- `body_orientation_0_body_vel_linear_positive_rotation_positive`
  - Robot is oriented in the x-direction at [0, -1]
  - Robot is given both an angular velocity and a linear x-velocity such that the robot will
    drive in a circle with as center point [0, 0].
  - The drive modules are oriented such that the robot will turn in a circle, i.e. the
    modules have their steering angles set to
    - front-left: 45 degrees
    - rear-left: 315 degrees
    - rear-right: 342 degrees
    - front-right: 18 degrees
