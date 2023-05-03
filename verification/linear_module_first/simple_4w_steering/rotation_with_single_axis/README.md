# Verification of the Simple 4 wheel steering model using rotation followed by linear movements

The goal for this verification set are to validate that movements in the robot axis directions
are correct. The verifications consist of movements in both the positive and negative direction
of the robot axis to ensure that there are differences between these directions.

Four movement simulations were done:

* A negative linear velocity in robot x-direction, ramping up from 0 m/s to -1 m/s with the robot
  facing in the y-direction.
* A positive linear velocity in robot x-direction, ramping up from 0 m/s to 1 m/s with the robot
  facing in the y-direction.

* A negative linear velocity in robot y-direction, ramping up from 0 m/s to -1 m/s with the robot
  facing in the y-direction.
* A positive linear velocity in robot y-direction, ramping up from 0 m/s to 1 m/s with the robot
  facing in the y-direction.

These simulations validate that the coordinate system rotations have been correctly implemented.

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

* [orientation y-direction, positive x-direction movement](body_orientation_90_body_vel_linear_positive_dm_direction_linear_0/sim_results.csv) -
  Simulation with body orientation along the world y-axis, linear movement in the positive robot x-direction, drive modules set
  to zero degree steering angle. The expectations are:
  * The robot will move 0.5 meters in the world y-direction
  * There is no change in the angular direction
  * The robot x-velocity will linearly ramp up to 1 m/s
  * The robot y-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 0 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [orientation y-direction, negative x-direction movement](body_orientation_90_body_vel_linear_negative_dm_direction_linear_0/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the negative x-direction, drive modules set
  to zero degree steering angle. The expectations are:
  * The robot will move -0.5 meters in the world y-direction
  * There is no change in the angular direction
  * The robot x-velocity will linearly ramp up to -1 m/s
  * The robot y-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be 0 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

* [orientation y-direction, positive y-direction movement](body_orientation_90_body_vel_linear_positive_dm_direction_linear_-90/sim_results.csv) -
  Simulation with body orientation along the world y-axis, linear movement in the positive robot y-direction, drive modules set
  to zero degree steering angle. The expectations are:
  * The robot will move -0.5 meters in the world x-direction
  * There is no change in the angular direction
  * The robot y-velocity will linearly ramp up to 1 m/s
  * The robot x-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be -90 degrees before movement starts.
  * The drive module velocity will linearly ramp up to 1 m/s
* [orientation y-direction, negative y-direction movement](body_orientation_90_body_vel_linear_negative_dm_direction_linear_-90/sim_results.csv) -
  Simulation with body orientation along the robot x-axis, linear movement in the negative x-direction, drive modules set
  to zero degree steering angle. The expectations are:
  * The robot will move 0.5 meters in the world x-direction
  * There is no change in the angular direction
  * The robot y-velocity will linearly ramp up to -1 m/s
  * The robot x-velocity and rotational velocity will remain at zero
  * The drive module steering angle will be -90 degrees before movement starts.
  * The drive module velocity will linearly ramp up to -1 m/s

The simulation results align with the expectations.
