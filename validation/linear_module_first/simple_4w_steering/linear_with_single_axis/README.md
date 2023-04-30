# Validation of the Simple 4 wheel steering model using single linear movements

The goal for this validation set are to validate that movements in the robot axis directions
are correct. The validations consist of movements in both the positive and negative direction
of the robot axis to ensure that there are differences between these directions.

## Linear movements

Eight linear movement simulations were done:

* A negative linear velocity in x-direction, ramping up from 0 m/s to -1 m/s with the robot
  facing in the x-direction.
* A positive linear velocity in x-direction, ramping up from 0 m/s to 1 m/s with the robot
  facing in the x-direction.
* A negative linear velocity in y-direction, ramping up from 0 m/s to -1 m/s with the robot
  facing in the x-direction.
* A positive linear velocity in y-direction, ramping up from 0 m/s to 1 m/s with the robot
  facing in the x-direction.
* A negative linear velocity in negative x-direction, ramping up from 0 m/s to -1 m/s with the robot
  facing in the x-direction.
* A positive linear velocity in negative x-direction, ramping up from 0 m/s to 1 m/s with the robot
  facing in the x-direction.
* A negative linear velocity in negative y-direction, ramping up from 0 m/s to -1 m/s with the robot
  facing in the x-direction.
* A positive linear velocity in negative y-direction, ramping up from 0 m/s to 1 m/s with the robot
  facing in the x-direction.

These simulations validate that the linear single axis movements have been correctly implemented.

### Setup

For all simulations not running in the positive x-direction in the first second of the simulation the
drive modules are turned to the appropriate angle for that simulation. Once the modules have reached
their final steering angle the robot is commanded to move in the desired direction at 1 m/s or -1 m/s.
The velocity is ramped up to the final velocity over a period of 1 second.

### Expectations and results

For the simulations where the drive modules are turned to the appropriate steering angle in the
first part of the simulation it is expected that there is no movement of the robot body.

During the part of the simulation where the wheel velocity ramps up from stand still to the final
velocity it is expected that the wheel velocity follows a linear path. At the same time it is
expected that the body location follows a quadratic curve.

The results of the simulations can be found in the following sub directories:

* [positive x-direction](body_orientation_0_body_vel_linear_positive_dm_direction_linear_0/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the positive x-direction, drive modules set
  to zero degree steering angle. The expectations are:
  * The robot will move 0.5 meters in the x-direction
  * There is no change in the angular direction
  * The robot x-velocity will linearly ramp up to 1 m/s
  * The robot y-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 0 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [negative x-direction](body_orientation_0_body_vel_linear_negative_dm_direction_linear_0/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the negative x-direction, drive modules set
  to zero degree steering angle. The expectations are:
  * The robot will move -0.5 meters in the x-direction
  * There is no change in the angular direction
  * The robot x-velocity will linearly ramp up to -1 m/s
  * The robot y-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 0 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

* [positive x-direction, reversed modules](body_orientation_0_body_vel_linear_positive_dm_direction_linear_180/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the positive x-direction, drive modules set
  to 180 degree steering angle. The expectations are:
  * The robot will move 0.5 meters in the x-direction
  * There is no change in the angular direction
  * The robot x-velocity will linearly ramp up to 1 m/s
  * The robot y-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 180 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s
* [negative x-direction, reversed modules](body_orientation_0_body_vel_linear_negative_dm_direction_linear_180/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the negative x-direction, drive modules set
  to 180 degree steering angle. The expectations are:
  * The robot will move -0.5 meters in the x-direction
  * There is no change in the angular direction
  * The robot x-velocity will linearly ramp up to -1 m/s
  * The robot y-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 180 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s

* [positive y-direction](body_orientation_0_body_vel_linear_positive_dm_direction_linear_90/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the positive y-direction, drive modules set
  to 90 degree steering angle. The expectations are:
  * The robot will move 0.5 meters in the y-direction
  * There is no change in the angular direction
  * The robot y-velocity will linearly ramp up to 1 m/s
  * The robot x-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 90 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [negative y-direction](body_orientation_0_body_vel_linear_negative_dm_direction_linear_90/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the negative y-direction, drive modules set
  to 90 degree steering angle. The expectations are:
  * The robot will move -0.5 meters in the y-direction
  * There is no change in the angular direction
  * The robot y-velocity will linearly ramp up to -1 m/s
  * The robot x-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 90 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

* [positive y-direction, reversed modules](body_orientation_0_body_vel_linear_positive_dm_direction_linear_270/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the positive y-direction, drive modules set
  to 270 degree steering angle. The expectations are:
  * The robot will move 0.5 meters in the y-direction
  * There is no change in the angular direction
  * The robot y-velocity will linearly ramp up to 1 m/s
  * The robot x-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 270 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s
* [negative y-direction, reversed modules](body_orientation_0_body_vel_linear_negative_dm_direction_linear_270/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the negative y-direction, drive modules set
  to 270 degree steering angle. The expectations are:
  * The robot will move 0.5 meters in the y-direction
  * There is no change in the angular direction
  * The robot y-velocity will linearly ramp up to -1 m/s
  * The robot x-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 270 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s

The simulation results align with the expectations.

## Rotational movements

Two rotational movement simulations were done:

* A negative angular velocity around the center of the robot, ramping up from 0 rad/s to -1 rad/s. The
  results for this simulation
* A positive angular velocity around the center of the robot, ramping up from 0 rad/s to 1 rad/s

These simulations are used to verify that the rotational parts of the equations have been
properly implemented.

### Setup

For both simulations in the first second of the simulation the drive modules are turned to the
following angles:

* left-front: 135 degrees
* left-rear: 225 degrees
* right-rear: 315 degrees
* right-front: 45 degrees

Once the modules have been turned to their final steering angle the robot is commanded to rotate
at 1 rad/s or -1 rad/s. The velocity is ramped to the final velocity over a period of 1 second.

### Expectations and results

For the simulations the drive modules are turned to the appropriate steering angle in the
first part of the simulation it is expected that there is no movement of the robot body.

During the part of the simulation where the wheel velocity ramps up from stand still to the final
velocity it is expected that the wheel velocity follows a linear path. At the same time it is
expected that the body orientation follows a quadratic curve.

The results of the simulations can be found in the following sub directories:

* [positive rotation](body_orientation_0_body_vel_angular_positive_dm_direction_rotation_inplace/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, a negative robot body rotation, drive modules set
  to the appropriate angles for a rotation around the robot body center. The expectations are:
  * The robot will rotate 0.5 radians around the center of the robot
  * There is no change in the x or y directions
  * The robot rotational velocity will linearly ramp up to 1 rad/s
  * The robot x-velocity and y-velocity will remain at zero
  * The drive module velocity will linearly ramp up to 0.707 m/s
* [negative rotation](body_orientation_0_body_vel_angular_positive_dm_direction_rotation_inplace/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, a positive robot body rotation, drive modules set
  to the appropriate angles for a rotation around the robot body center. The expectations are:
  * The robot will rotate -0.5 radians around the center of the robot
  * There is no change in the x or y directions
  * The robot rotational velocity will linearly ramp up to -1 rad/s
  * The robot x-velocity and y-velocity will remain at zero
  * The drive module velocity will linearly ramp up to -0.707 m/s

The simulation results align with the expectations.
