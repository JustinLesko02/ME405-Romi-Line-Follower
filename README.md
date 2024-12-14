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
- 
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
Both of these mounts were screwed into the main chassis using M2.5 screws
## Code Structure

## Control Structures
This section will describe the control structures that are used in the robot.
### Motor Control
Each of our motors uses closed-loop proportional-integral control. 
<img width="749" alt="Screen Shot 2024-11-21 at 9 37 00 AM" src="https://github.com/user-attachments/assets/ef799270-c59c-4ba0-b219-5e157936887b" />
### Line Sensing Control
### Heading control
At multiple points in the line following track, the robot must move to a set angle, so closed-loop heading control which fed directly into 
## Calculations
## References


