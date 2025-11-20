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

| Number | Name |
| --- | --- |
| 1 | what the sigma |
