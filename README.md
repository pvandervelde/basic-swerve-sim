# ros-multi-wheel-steering-controller-py

This package provides a controller for a wheeled mobile robot which has all wheel drive and all
wheel steering


## Kinematics

<figure
    style="float:left"
    width="560"
    height="315">
<img alt="Kinematics diagram for the multi-wheel steering controller" src="doc/kinematics.png" />
<figcaption>Kinematics diagram for the multi-wheel steering controller</figcaption>
</figure>

The two main ways of calculating the kinematics for a wheeled mobile robot with N wheels, all of
which both steer and provide drive capabilities are using 3D vectors or using the Center of Rotation (CoG)
approach.


Equations:

    v_i = v + W x r_i

    alpha = acos (v_i_x / |v_i|)
          = asin (v_i_y / |v_i|)

Which translates to

    v_i_x = v_x - W * r_i_y
    v_i_y = v_y + W * r_i_x


### Assumptions

- No wheel side slip
- The wheel steering axis goes through the center of the wheel in a vertical direction


## Implementation

For each time step
    Calculate the current velocity vector V and the rotation vector W using forward kinematics

    Determine the trajectory from the current [V, W] to the desired [V, W]

    Determine the desired [v_i, alpha_i] for each wheel module

    Determine the trajectory from the current [v_i, alpha_i] to the desired [v_i, alpha_i], such
    that it limits the acceleration / jerk of the vehicle and stays within module kinematic constraints


## Testing





## Special cases

- start: all modules aligned, moving forwards. end: all modules aligned moving backwards
- start: all modules aligned, moving forwards. end: all modules aligned moving sideways
- start: velocity = 0, rotation != 0. end: velocity = 0, rotation != 0. rotation_start != rotation_end
- start: velocity = 0, rotation != 0. end: velocity = 0, rotation != 0. rotation_start = -rotation_end
- start: velocity != 0, rotation != 0. end: all stop
- start: all stop. end: velocity != 0, rotation != 0
- start: velocity != 0, rotation = 0. end: velocity != 0, rotation = 0. velocity_start != velocity_end
- start: velocity = 0, rotation != 0. end: velocity != 0, rotation = 0
- start: velocity != 0, rotation = 0. end: velocity = 0, rotation != 0
- rotation around wheel


## Literature

- <https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383/5>
- A vector algebra formulation of kinematics of wheeled mobile robots - Alonzo Kelly - 2010
- A vector algebra formulation of mobile robot velocity kinematics - Alonzo Kelly and Neal Seegmiller
- Enhanced 3D kinematic modeling of wheeled mobile robots - Neal Seegmiller and Alonzo Kelly
-