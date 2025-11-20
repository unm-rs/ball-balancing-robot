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

### Inverse Kinematics
- The mathematical process of calculating the joint angles of a robot's limb to reach specific position and orientation in space.
- To achieve a precise tilt of the platform on the ball-balancing robot, inverse kinematics is done to calculate the angles that each servo should be set to.

#### Geometry of the Robot
The robot consists of **three** arms equally spaced at 120 degrees, which connects the top platform to the bottom base.

#### Inverse Kinematics Calculations
As the robot has three arms, the platform can move in the x-axis, y-axis, and z-axis directions. 

- The orientation Vector [a,b,c] represent the x,y, and z components.
- These three vectors form a unit length of 1: a^2 + b^2 + c^2 = 1.
- This vector ensures that the algorithm knows exactly the rotation of the platform in 3D space

##### General Calculation Flow
Input: orientation vector [a,b,c] + desired height [h] → Output: servo motor angles θ1​,θ2​,θ3​.
Examples: 
- [0, 0, 1] - to move the platform straight up
- a = sin(5)cos(0), b = sin(5)sin(0), c = cos(5): [0.087, 0, 0.996] - to tilt the platform forward by 5 degrees

When we tilt the platform, the upper attachment points move to new positions in 3D space. Since the lower attachment points are fixed to the base, the question is: 
- Where exactly should the middle joint C be located so that both links (l1 and l2) maintain their fixed lengths?  
This condition is essential because the robot’s arms are rigid mechanical links — they cannot stretch or shrink. By fixing their lengths, we ensure that the calculated joint positions are physically possible for the real robot, so the motion derived from the inverse kinematics corresponds exactly to what the hardware can achieve.

##### Servo Angle Calculation: 
After solving the geometry, the algorithm knows where the middle joint (C1) needs to be in 3D space. The base joint (B1) is fixed, and the servo is at B1. To reach C1 with the bottom link, the servo must rotate to a certain angle θ1.
Mathematically, the code does:
θ1​=π/2 ​− arctan​ ( (sqrt(C1x^2 + C1y^2) - lb)/C1z)
