# Verification Results

This folder contains the verification results for the different combinations of [controller](../swerve_controller/multi_wheel_steering_controller.py)
and the [model](../swerve_controller/control_model.py).

- linear_body_first - Verifications for the different implementations of a controller that uses linear movement profiles
  for the robot body. Movement profiles for the drive modules are determined based on the motion profile of the
  robot body.
  - [Simple 4 wheel steering](linear_body_first/simple_4w_steering/README.md) - Verifications for the
    [simple 4 wheel steering model](https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383/5)
- linear_module_first - Verifications for the different implementations of a controller that uses linear movement profiles
  for the drive modules. Movement profiles for the robot body are determined based on the motion profile of the drive
  modules.
  - [Simple 4 wheel steering](linear_module_first/simple_4w_steering/README.md) - Verifications for the
    [simple 4 wheel steering model](https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383/5)
