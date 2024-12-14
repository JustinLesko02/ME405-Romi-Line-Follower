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
### Overview

The system runs on a MicroPython-based Nucleo STM32 board and controls a Romi robot platform. Its functionality includes line following, obstacle avoidance, and returning to a home position. Each major subsystem (motors, IMU, line sensors) is encapsulated in dedicated modules and tasks, and all coordination is achieved through a cooperative scheduler and shared variables. The code translates high-level goals into specific motor commands and state transitions based on sensor inputs.

### Detailed File-by-File and Inter-Task Mechanics

#### main.py

**Setup and Configuration:**  
- Declares global constants for geometry and motion (e.g., `Turn_Radius`, `Velocity`, `Track_Width`, `Wheel_Radius`).
- These values are used to compute `Set_Speed` and `Set_Turn`. For instance:
  - `Set_Speed` converts a desired linear velocity (in inches/second) into encoder ticks/second, considering wheel radius and encoder resolution.
  - `Set_Turn` determines how to bias the inside vs. outside wheel speed for a given turn radius.
- Initializes multiple `Share` objects (e.g., `heading`, `motor_run_flag`, `turn_signal`, `black_line_flag`) to enable safe data exchange between tasks.
- Configures interrupt handlers for user input and bump sensors. These interrupts directly modify shares like `motor_run_flag` or `button_ok` to signal the tasks that a new event has occurred (e.g., user pressed the start button, or robot bumped into an obstacle).

**Task Creation and Scheduling:**  
- Defines tasks as generator functions: `motor_update_A`, `motor_update_B`, `IMU_update`, `line_sensor_update`, and `print_status`.
- Each task is appended to a global task list in `cotask.py`.
- After initialization, `main.py` enters a loop calling `cotask.task_list.pri_sched()` repeatedly. This scheduler checks which tasks are ready to run, runs them in priority order, and ensures periodic execution.

#### BNO055.py

**IMU Initialization and Calibration:**  
- Communicates with the IMU via I2C, setting the operation mode to NDOF (sensor fusion mode).
- Attempts to load previously saved calibration data from `calibration.txt`. If not found, the code instructs the user to move the robot to achieve full calibration (all sensor statuses must reach 3). Once calibrated, the data is saved for future runs.

**Data Reading:**  
- `read_euler_angles()` and `read_velocity()` read Euler angles and gyroscopic data from specific registers.
- Values are unpacked using `struct` and converted to meaningful units (degrees for heading, deg/s or related units for gyro).
- The IMU task in `main.py` uses these functions to update the `heading` and `gyroZ` shares, which other tasks read.

#### controller.py

**Proportional and PI Controllers:**  
- `ProportionalController` and `ProportionalIntegralController` classes encapsulate the feedback control logic.
- Each update:
  - Uses `ticks_us()` and `ticks_diff()` to measure elapsed time since the last update, enabling speed computation from encoder deltas.
  - Reads the encoder’s delta counts to compute actual speed.
  - Calculates error as `(target_speed - actual_speed)`.
  - Applies proportional and possibly integral terms to produce a control signal (duty cycle).
  - Clamps duty cycle to safe limits (−100 to +100) and sends it to the motor driver.

**Integration with Motor Tasks:**  
- The motor tasks create these controller objects once during initialization.
- On each run, they set the `target_speed` dynamically based on current mode: normal circle running, obstacle avoidance maneuvers, or return-to-home sequences.

#### DRV8838.py

**Motor Driving:**  
- For a given motor, sets direction pins high or low and updates PWM duty cycle via a Timer channel.
- `enable()` and `disable()` allow the system to start and stop motors cleanly.

**Motor Task Usage:**  
- `motor_update_A` and `motor_update_B` tasks call `set_duty()` indirectly through the controller’s update method, ensuring closed-loop control drives the wheels at the correct speed and direction.

#### Encoder.py

**Quadrature Decoding:**  
- Uses a hardware timer in `ENC_AB` mode to read two encoder signals.
- Each `update()` call reads the timer’s current count, compares it to previous counts, and calculates delta (position change).
- Overflows are handled by adjusting delta if the timer wraps around.

**Speed Feedback Loop:**  
- Motor tasks run their controller’s `update()` method, which calls `encoder.update()`.
- The resulting delta is converted to speed (counts/second), serving as real-time feedback for the PI loop.

#### linesensor.py

**Reading IR Sensors:**  
- Activates IR LEDs, then configures multiple pins as ADC inputs to read reflectance from the surface below.
- For each sensor, the code:
  - Drives the pin high as output, then switches to input mode.
  - Uses `ADC(pin)` to read reflectance value.
  - Compares the reading against a threshold.

**Centroid and Black Line Detection:**  
- After reading all sensors, the code computes a centroid value based on which sensors see dark vs. light surfaces.
- The `turn_signal` share is set to a function of the centroid, influencing how the motor tasks adjust their wheel speeds to correct deviations.
- If all or most sensors detect black, `black_line_flag` is set, signaling the motors to initiate return-to-home logic.

#### cotask.py and task_share.py

**Task Scheduling:**  
- Task objects store the generator function, priority, period, and profiling data.
- `pri_sched()` checks the current time and runs the highest priority task that’s ready (its period expired or a condition triggered).
- Tasks yield after a brief execution, allowing other tasks to run and preventing blocking.

**Data Sharing:**  
- `Share` objects store single values safely. Interrupts are disabled when reading/writing shares to prevent corruption.
- The code uses shares extensively to pass sensor data (e.g., `heading`, `gyroZ`) and state flags (`motor_run_flag`, `black_line_flag`, `button_ok`) between tasks.

### Task-Level State Machines and Interactions

**Motor Tasks (`motor_update_A` and `motor_update_B`):**  
- Start in `S0_INIT` setting up hardware and controllers.
- Move to `S1` (standby) waiting for:
  - `calibration_flag` from the IMU to confirm the IMU is calibrated.
  - `motor_run_flag` indicating the user started the robot.
- When ready, enable motors and enter `S2_RUN`, using line sensor `turn_signal` to maintain a curve and `heading_error` for fine adjustments.
- If `motor_run_flag` changes due to a bump, they transition through states like `S_BUMP_STOP`, `S_BUMP_TURN`, `S_BUMP_CIRCLE` to navigate around obstacles.
- If `black_line_flag` indicates a special line, they switch to states like `S_BLACK_LINE`, `S_TURN_180`, `S_GO_STRAIGHT` to return home.

**IMU Task (`IMU_update`):**  
- In `S0_INIT`, sets up I2C and IMU mode.
- `S1` calibrates or loads calibration data.
- `S2` waits for start signal.
- `S3_RUN` continuously reads heading, updates `heading_error`, and checks `motor_run_flag` states:
  - If in obstacle avoidance turn mode (`S_BUMP_TURN`), it checks heading against a target angle. Once heading is within a tolerance, sets `motor_run_flag` to the next stage.
  - If in return-to-home (`S_TURN_180`), monitors heading until aligned with the starting orientation before switching `motor_run_flag` to straight movement.

**Line Sensor Task (`line_sensor_update`):**  
- After initialization, once running, it periodically reads the line sensors.
- Computes centroid and sets `turn_signal`.
- If full black is detected, sets `black_line_flag`.

**Print Status Task (`print_status`):**  
- Periodically prints out diagnostic information such as current heading, gyro rates, encoder deltas, and `motor_run_flag`.
- Helps debug timing, confirm that tasks run as expected, and that control loops behave correctly.

### Additional Internal Details

**Timing and Measurement:**  
- `ticks_us()` and `ticks_diff()` from MicroPython’s timing functions measure intervals accurately.
- The controller classes rely on these to convert encoder deltas into speeds.

**Heading Wrapping:**  
- The IMU task uses offset logic when planning turns. If a desired turn angle might wrap heading from just below 360° to just above 0° (or vice versa), offsets ensure consistent error calculations.

**Shared Variable Access Patterns:**  
- Motor tasks primarily read `heading_error`, `turn_signal`, and `motor_run_flag`.
- IMU task writes to `heading_error` and reads `motor_run_flag`.
- Line sensor task writes to `turn_signal` and `black_line_flag`.
- Interrupt handlers write to `motor_run_flag` and `button_ok` when triggered.

By coordinating these reads and writes, the entire robot’s operation emerges from these modular updates. Each sensor and controller runs at its own pace (period) defined in the task constructors, ensuring stable, predictable performance.

### Summary of Internal Workings

Each file focuses on a clear responsibility:
- Sensor drivers read data.
- Controllers compute duty cycles.
- Tasks run state machines that interpret shared flags to decide what action to take next.

The cooperative scheduler ensures all tasks receive CPU time. The shared variables serve as the communication backbone, enabling events like bumps or black line detection to ripple through the state machines and alter the robot’s course.

In this way, the code continuously processes sensor input, updates its internal states, and directs the motors to achieve higher-level goals, all through a structured, modular, and well-documented approach.

### Information flow diagram

![image](https://github.com/user-attachments/assets/f6d21c75-c450-40c8-b682-94a9301b0f24)


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
If the robot is in its line-following mode, the centroid read by the IR light sensor is then used to manipulate the turning radius of the robot. For instance, if the line is in the middle of the sensor, then the centroid  will be 0, which then causes the turning radius to be 0. If the line is to the right of the sensor, then the centroid read will be positive, which will then cause the turning radius to be divided by the centroid value. Therefore, the further away from the center the line is, the smaller the turning radius is, causing the robot to correct harder if it is off course. The schematic below shows the control scheme flow. The feedback in this case is more subtle, operating between the centroid reading at the end and the turning radius.
![image](https://github.com/user-attachments/assets/ecb52f0e-803f-4b98-b2fb-cf633d7e9dcf)



### Heading control
At multiple points in the line following track, the robot must move to a set angle, so closed-loop proportional heading control was selected. This closed-loop heading signal is then fed into the motor controller. In order to ensure that the robot behaves consistently, the gains for these controlled spins are set very low so that the inertia does not disrupt the following movement. 
## Calculations

## References


