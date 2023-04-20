# Motion configurations for Validation of the code

This folder contains the simulation configuration files for the validation calculations. The validation
calculations are divided into three groups:

- [Single axis movements](linear_with_single_axis/README.md) - Configurations for simulations where
  the robot moves along a single axis, e.g. linear moves along the x- or y-axis, or rotations around
  the z-axis. These simulations are designed to validate that the simulation treats the positive and
  negative directions of the different axis.
- [Rotation movements](rotation_with_single_axis/README.md) - Configurations for simulations where
  the robot moves along a single axis before or after a rotation. These simulations are designed to
  validate that the results of the rotation are correctly taken into account.
- [Combined movements](combined/README.md) - Configurations for simulations where the robot moves
  along multiple axis at the same time, e.g. combined motion in the x- and y-axis directions. These
  simulations are designed to validate that combination movements return the correct results.

