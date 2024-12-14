# ME405 Romi Line Follower Project
#### By Marcus Pietro and Justin Lesko
## Overview
This repository contains code and auxillary files relavent to run a line following robot. 
## Table of Contents
1. Electrical Design
2. Mechanical Design
3. Code Structure
4. Control Structures and Calculations
## Electrical Design
This section describes the electrical design of our line following robot.
### Components
Besides the power distribution board and gearmotor-encoder pairs that came in the ROMI kit, we also used three other devices, as noted in the table below.
| Sensor   | Part #  | Vendor  | 
| -------- | ------- | ------- | 
| IMU      | BNO055  | adafruit|
| Bump switch| 3678  | Polulu | 
| IR Line Sensor|3545| Polulu|

Each of the three sensors above was necessary to complete specific parts of the line-following track. 
- The IMU enables the robot to accurately turn around the obstacle and allows for the robot to return to the starting square using its initial heading. 
- The bump switches allow the robot to detect an obstacle in its path
- The IR Line Sensor allows the robot to follow the line.
### Wiring
Our electrical design is fairly standard for ROMI line following robots. All of our components are plugged into a nucleo development board, which receives power from a ROMI PDB. Shown below is our wiring diagram.

![Untitled Diagram drawio](https://github.com/user-attachments/assets/a3b0814f-1b20-4740-8eae-b674b0f8a109)
Of note, due to the number of devices used in this project, most 5V and GND connections were made on a section of perf board instead of plugging directly into the nucleo board.
## Mechanical Design
The mechanical design for our line follower is a modified version of the standard ROMI POLULU chassis kit. The main modifications we made were 3D printed mounts for the IMU, bump sensors, and IR line sensor. All necessary CAD files are in the CAD folder in the repository.
### IMU Mount
In order to have enough space on the front of the robot for the line and bump sensors, the IMU is mounted on the "back" of the ROMI. A mount was 3D printed to hold the IMU level above the back caster. A picture of the CAD for this mount can be seen below.
![image](https://github.com/user-attachments/assets/4bd9ce43-ba0c-4eb3-98f1-6e1b10570b08)
### Bump Sensor and IR line sensor mount
In order to secure all of the necessary sensors to the front, we 3D printed a mount. The bump sensors are secured using the holes closest to the edges on the side, while the IR light sensor is mounted using tape to the front rack of the mount. A picture of the CAD can be seen below. Initial designs for this mount did not include room for the IR light sensor due to pre-fabricated mounting holes in the ROMI itself, but we found that this caused the IR light sensor to be right between the wheels, causing the robot to have a much worse line following consistency. Therefore, mounting the IR light sensor to the front of this part is necessary for consistency and resolution in line reading. 
![image](https://github.com/user-attachments/assets/72b3a93d-ac85-48df-9d0d-ee1f16bd5c0a)
### Overall Assembly
Both of these mounts were screwed into the main chassis using M2.5 screws and bolts. Mutltiple pictures of our overall assembly in action can be seen below. 

<img width="300" src="https://github.com/user-attachments/assets/24a3d4a6-d5e1-4f57-bb6a-67a0ccb37b00" /><img width="275" src="https://github.com/user-attachments/assets/f7620406-b6aa-49b0-98e4-28ff58cb5f17" />


## Code Structure

## Control Structures
This section will describe the control structures that are used in the robot.
### Overall
Overall, the robot's operation is most closely linked to the chosen set point velocity and turning radius, defined at the top of **main.py**. Past implimentations of a line follower have used a velocity and yaw rate which is the manipulated by the rest of the program, but we believe that tuning a setpoint turning radius is much more intuitive than a yaw rate. Througout the code, the velocity of the robot and the turning radius is manipulated from these setpoints, so some tuning of these two values could be necessary for line following tracks or hardware different than ours.
### Motor Control
In order to control the speed of each motor, closed-loop proportional-integral speed control is used . 
<img width="749" alt="Screen Shot 2024-11-21 at 9 37 00 AM" src="https://github.com/user-attachments/assets/ef799270-c59c-4ba0-b219-5e157936887b" />

This control scheme can be seen in controller.py, under the ProportionalIntegralController class. This class takes uses motor and encoder objects as attributes, calculates proportional and integral errors, and returns a saturation-checked signal for the motor PWM.
Of note, all of the math that the class does is in encoder ticks, so target motor speeds are in units of encoder ticks per second. In addition, this causes ki and kp selected for the motors to be very small due to the high ratio of encoder ticks per revolution.
### Line Sensing Control
In order to track lines, semi-closed loop control is implimented using the line sensor.
If the robot is in its line-following mode, the centroid read by the IR light sensor is then used to manipulate the turning radius of the robot. For instance, if the line is in the middle of the sensor, then the centroid  will be 0, which then causes the turning radius to be 0. If the line is to the right of the sensor, then the centroid read will be positive, which will then cause the turning radius to be divided by the centroid value. Therefore, the further away from the center the line is, the smaller the turning radius is, causing the robot to correct harder if it is off course. 


### Heading control
At multiple points in the line following track, the robot must move to a set angle, so closed-loop proportional heading control was selected. This closed-loop heading signal is then fed into the motor controller. In order to ensure that the robot behaves consistently, the gains for these controlled spins are set very low so that the inertia does not disrupt the following movement. 
## Calculations

## References


