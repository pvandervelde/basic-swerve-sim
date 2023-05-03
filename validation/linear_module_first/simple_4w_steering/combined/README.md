# Validation of the Simple 4 wheel steering model using combined movements

The goal for this validation set are to validate that movements along multiple axis
are correct. The validations consist of movements in at least two axis directions at any
given time.

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

* [orientation 0, positive velocity, modules 30 degree](body_orientation_0_body_vel_linear_positive_dm_direction_linear_30/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 30 degrees to the positive x-direction, drive
  modules set to 30 degree steering angle and are given a positive wheel velocity. The expectations are:
  * The robot will move 0.5 meters at an angle of 30 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to 1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 30 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [orientation 0, negative velocity, modules 30 degree](body_orientation_0_body_vel_linear_negative_dm_direction_linear_30/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 30 degrees to the positive x-direction, drive
  modules set to 30 degree steering angle and are given a negative wheel velocity. The expectations are:
  * The robot will move -0.5 meters at an angle of 30 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to -1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 30 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

* [orientation 0, positive velocity, modules 60 degree](body_orientation_0_body_vel_linear_positive_dm_direction_linear_60/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 60 degrees to the positive x-direction, drive
  modules set to 60 degree steering angle and are given a positive wheel velocity. The expectations are:
  * The robot will move 0.5 meters at an angle of 60 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to 1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 60 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [orientation 0, negative velocity, modules 60 degree](body_orientation_0_body_vel_linear_negative_dm_direction_linear_60/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 60 degrees to the positive x-direction, drive
  modules set to 60 degree steering angle and are given a negative wheel velocity. The expectations are:
  * The robot will move -0.5 meters at an angle of 60 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to -1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 60 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

* [orientation 0, positive velocity, modules 120 degree](body_orientation_0_body_vel_linear_positive_dm_direction_linear_120/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 120 degrees to the positive x-direction, drive
  modules set to 120 degree steering angle and are given a positive wheel velocity. The expectations are:
  * The robot will move 0.5 meters at an angle of 120 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to 1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 120 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [orientation 0, negative velocity, modules 120 degree](body_orientation_0_body_vel_linear_negative_dm_direction_linear_120/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 120 degrees to the positive x-direction, drive
  modules set to 120 degree steering angle and are given a negative wheel velocity. The expectations are:
  * The robot will move -0.5 meters at an angle of 120 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to -1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 120 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

* [orientation 0, positive velocity, modules 150 degree](body_orientation_0_body_vel_linear_positive_dm_direction_linear_150/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 150 degrees to the positive x-direction, drive
  modules set to 150 degree steering angle and are given a positive wheel velocity. The expectations are:
  * The robot will move 0.5 meters at an angle of 150 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to 1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 150 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [orientation 0, negative velocity, modules 150 degree](body_orientation_0_body_vel_linear_negative_dm_direction_linear_150/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement at 150 degrees to the positive x-direction, drive
  modules set to 150 degree steering angle and are given a negative wheel velocity. The expectations are:
  * The robot will move -0.5 meters at an angle of 150 degrees to the x-direction
  * There is no change in the angular direction
  * The robot linear velocity will linearly ramp up to -1 m/s
  * The robot rotational velocity will remain at zero
  * The drive module steering angle will be 150 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

The simulation results align with the expectations.

## Rotational movements

One rotational movement simulation was done:

* A positive angular velocity around the center of the robot, combined with a positive velocity in
  the robot x-direction. The velocities set such that the robot turns in a circle.

These simulations are used to verify that the combination of linear and rotational motions have been
properly implemented.

### Setup

At the start of the simulation the robot is placed at the x, y coordinates of [0, -1] with the aim
of turning a circle which has [0, 0] as the center point.

For the simulation in the first second of the simulation the drive modules are turned to the
following angles and wheel velocities:

* left-front: 45 degrees steering angle and 0.889 m/s wheel velocity
* left-rear: 315 degrees steering angle and 0.889 m/s wheel velocity
* right-rear: 342 degrees steering angle and 1.99 m/s wheel velocity
* right-front: 18 degrees steering angle and 1.99 m/s wheel velocity

Once the modules have been turned to their final steering angle the robot is commanded to rotate
at 1.257 rad/s while at the same time given a linear x-velocity of 1.257 m/s. Both velocities is
ramped to the final velocity over a period of 5 seconds.

### Expectations and results

The results of the simulations can be found in the following sub directories:

* [circular motion](body_orientation_0_body_vel_linear_positive_rotation_positive/sim_results.csv) -
  Simulation with the robot driving in a circle around [0, 0]. The expectations are:
  * In 5 seconds the robot will complete 1 full circle around [0, 0]
  * During this period there is no change in drive module steering angle and wheel velocities
  * The robot rotational velocity and x-velocity will be a constant.
  * The robot y-velocity will remain at zero

The simulation results align with the expectations.
