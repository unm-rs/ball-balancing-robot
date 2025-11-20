# Ball-Balancing Robot

A Raspberry Pi 4-powered ball-balancing robot that combines inverse kinematics, computer vision, and PID control to keep a ball stable on a movable platform.

## Features

- **Raspberry Pi-based control** - the "brain" of the robot with control loops and vision
- **PID control** - To stabilize the ball on the platform in real time
- **Computer vision** - To track the coordinates of the ball for PID control
- **Inverse kinematics** - To calculate servo angles for precise control over the tilt of the platform


## System Overview

#### The Control Pipeline

1. **Camera** captures frames of the platform.
2. **Computer vision** detects the ball and converts its position into (x, y) coordinates on the platform.
3. **PID controllers** compute the required tilt angles to reduce the error between the current and target position.
4. **Inverse kinematics** calculates the angle of the servo motors to be converted to the desired tilt angles.
5. **Servos** move the joints and tilts the platform accordingly.

## Materials

| Number | Name | Quantity |
| --- | --- | --- |
| 1 | Raspberry Pi 4 4GB | 1 |
| 2 | Raspberry Pi Camera Module V2 | 1 |
| 3 | 30kgcm Torque Servo Motor | 3 |
| 4 | 6mm Male Rod End Bearing | 3 |
| 5 | M2 x 8 Cap Head Socket Screw | 6 |
| 6 | M2.5 x 10 Cap Head Socket Screw | 4 |
| 7 | M3 x 5 Socket Head Screw | 18 |
| 8 | M3 x 10 Socket Head Screw| 3 |
| 9 | M3 x 15 Socket Head Screw | 1 |
| 10 | M3 x 20 Socket Head Screw | 3 |
| 11 | M4 x 20 Socket Head Screw | 6 |
| 12 | M4 x 30 Socket Head Screw | 6 |
| 13 | M5 x 30 Socket Head Screw | 3 |
| 14 | M3 Hex Nut | 9 |
| 15 | M4 Hex Nut | 12 |
| 16 | M3 x 10 Standoff | 9 |
| 17 | M3 x 15 Standoff | 6 |
| 18 | M3 x 20 Standoff | 3 |
| 19 | 4-10-4 Bearing | 6 |
| 20 | Rubber Foot 12x9x9 | 3 |
| 21 | MM5 Washer | 6 |

## Algorithm Explanation

#### Inverse Kinematics
- the mathematical process of calculating the joint angles of a robot's limb to reach specific position and orientation in space.


To achieve a precise tilt of the platform on the ball-balancing robot, inverse kinematics is done to calculate the angles that each servo should be set to.
