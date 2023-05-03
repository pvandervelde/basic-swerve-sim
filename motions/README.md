# Motion configurations

The `motions` folder contains the [sample](samples/readme.md) and [verification](verification/readme.md)
configuration files for the simulation code. The files in this folder are used to simulate a single
set of movements for a swerve drive robot.

Each file is a YAML file with the following structure

```
---
plan:
  description: "HUMAN_READABLE_DESCRIPTION_HERE"
  name: "CODE_NAME"
  start_state:
    body:
      position_in_meters_relative_to_world:
        x: 0.0
        y: 0.0
      orientation_in_radians_relative_to_world:
        z: 0.0
      angular_velocity_in_radians_per_second:
        z: 0.0
      linear_velocity_in_meters_per_second:
        x: 0.0
        y: 0.0
    modules:
      - name: "left-front"
        orientation_in_radians_relative_to_body: 0.0
        velocity_in_meters_per_second: 0.0
      - name: "left_rear"
        orientation_in_radians_relative_to_body: 0.0
        velocity_in_meters_per_second: 0.0
      - name: "right-rear"
        orientation_in_radians_relative_to_body: 0.0
        velocity_in_meters_per_second: 0.0
      - name: "right-front"
        orientation_in_radians_relative_to_body: 0.0
        velocity_in_meters_per_second: 0.0
  commands:
    - time_span: 1.0
      modules:
        - name: "left-front"
          orientation_in_radians_relative_to_body: 0.785398
          velocity_in_meters_per_second: 0.0
        - name: "left_rear"
          orientation_in_radians_relative_to_body: 0.785398
          velocity_in_meters_per_second: 0.0
        - name: "right-rear"
          orientation_in_radians_relative_to_body: 0.785398
          velocity_in_meters_per_second: 0.0
        - name: "right-front"
          orientation_in_radians_relative_to_body: 0.785398
          velocity_in_meters_per_second: 0.0
    - time_span: 1.0
      body:
        angular_velocity_in_radians_per_second:
          z: 0.0
        linear_velocity_in_meters_per_second:
          x: 0.70710678118
          y: 0.70710678118
    - time_span: 1.0
      body:
        angular_velocity_in_radians_per_second:
          z: 1.0
        linear_velocity_in_meters_per_second:
          x: 0.0
          y: 0.0

```

Where the sections are:

- `description` - The human readable description of the configuration file. Used for graph titles.
- `name` - The unique name for the configuration. Used for output directory and output file naming.
- `start_state` - The start state of the robot. Defined by specifying the state of the robot body
  and the state of the drive modules. Note that there is no detection of inconsistent body and
  drive module states, i.e. the code will not detect if the state of the body does not agree with
  the state of the drive modules.
  - `body` - Defines the state of the robot body. Defines the location, orientation and linear and
    angular velocities.
  - `modules` - Defines the state of the drive modules. At the moment the simulation code hard-codes
    four drive modules with the names `left-front`, `left-rear`, `right-rear` and `right-front`.
- `commands` - A list of movement commands. A movement command can be one for the body or for the
  drive modules. Each movement specifies a time span over which the movement takes place. It is
  assumed that this time span is in seconds.
  - `body` - Defines a movement command for the body of the robot. There may be zero or more of these
    in the commands list.
  - `modules` - Defines a movement command for all the drive modules. There may be zero or more of these
    in the commands list.
