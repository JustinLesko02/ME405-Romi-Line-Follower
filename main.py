#-------------------------------------------------------------------------------------------------*
# MAIN MODULE (main.py)
#
# Description:
# This code runs line following and IMU guidance. Now updated to handle an obstacle:
#  - If a bump is detected, stop line following, back up, drive in a circle around the obstacle,
#    then re-enable line following after the line is reacquired.
# 
#-------------------------------------------------------------------------------------------------*

from pyb import Pin, Timer, ExtInt
from time import ticks_ms, ticks_diff, ticks_us
from DRV8838 import DRV8838
from Encoder import Encoder
from BNO055 import BNO055
from controller import  ProportionalIntegralController
from linesensor import linesensor
import gc
import cotask
import pyb
import task_share
import math

BNO055_ADDRESS = 0x28

# States
S0_INIT = 0
S1 = 1
S2 = 2
S3 = 3
S4 = 4

# Motor handling states
S_RUN = 2
S_BUMP_STOP = 10
S_BUMP_TURN = 11
S_BUMP_CIRCLE = 12
S_BUMP_WAIT_LINE = 13
S_BUMP_LINE_FOUND = 14
S_BLACK_LINE = 15
S_TURN_180 = 16
S_GO_STRAIGHT = 17
S_BUMP_WAIT = 20
S_RTH_NUDGE = 21

# Configurable dimensions
Turn_Radius = 6 # inches
Velocity = 20 # inches/second
Track_Width = 137.5 #mm
Wheel_Radius = 70 #mm
impact_radius = 11 # Inches
turn_angle = 90 # Degrees
reset_angle = 43 # Degrees

# Gains
motor_kp = 0.02
motor_ki = 0.05
Turn_180_heading_gain = 0.02
Turn_90_heading_gain = 0.0125
Return_to_home_heading_gain = 0.2
# Calculated values for movement
Set_Speed = Velocity*25.4/(Wheel_Radius*2*math.pi)*1440
Set_Turn = Track_Width/(2*Turn_Radius*25.4)
def motor_update_A(shares):
    t1state = 0
    heading_error, black_line_flag, button_ok, turn_signal, deltaA, calibration_flag, motor_run_flag= shares
    print("motor A initialize!")
    while True:
        # Implement FSM inside while loop
        try:
            #print("t1state:",t1state)
            if t1state == S0_INIT:
               #Setup Motor A on Timer 8 Channel 3, with the specified pins. 
               tim_A = Timer(8, freq=20_000)
               CH_A = 3
               motor_A = DRV8838(CH_A, tim_A, Pin.cpu.C5, Pin.cpu.C6, Pin.cpu.C8)
               
               #Disable the motor and set duty to 0 to initialize motors to off
               motor_A.set_duty(0)
               motor_A.disable()
               #Initialize encoder at timer 2 with specified pins
               encoder_A = Encoder(timer_num=2, pinA=Pin.cpu.A0, pinB=Pin.cpu.A1)
               #Initialize motor speed to move forward once the button is pressed
               target_speed_A = -(Set_Speed)
               #Setup PI-control object using motor and encoder objects - used for updating speed based off of desired speed, kp, and ki
               controller_A = ProportionalIntegralController(motor_A, encoder_A, target_speed_A, kp=motor_kp, ki = motor_ki)
               #Next loop goes to state 1, standby
               t1state = S1
               print("motorA init")
            elif t1state == S1: #Standby
                #Test if the IMU is calibrated and the motor run flag has been set to the run value
                if calibration_flag.get() == 1 and motor_run_flag.get() == S_RUN:
                    #Enable Motor
                    motor_A.enable()
                    
                    print('motor a on')
                    #Start timer for speed control purposes
                    controller_A.ticks = ticks_us()
                    #Set motor run flag to 3 for other motor to catch 
                    motor_run_flag.put(3)
                    # Enable bump detection
                    button_ok.put(1)
                    #Next loop goes to state 2, run
                    t1state=S2

            elif t1state == S2: #run
                #Update motor speed based off of errors
                controller_A.target_speed = -(Set_Speed)*(1-(turn_signal.get()*Set_Turn))
                #print("target speed a", target_speed_A)
                controller_A.update()
                deltaA.put(encoder_A.get_delta())
                #If motor_run_flag has been turned off by IMU, set motor to stop and next loop goes to standby
                if motor_run_flag.get() == 0:
                    t1state = S1
                    motor_A.set_duty(0)
                    print('motor a off')
                #If a bump has been sensed, next state goes to Object Avoidance state, motor is turned off.
                elif motor_run_flag.get() == S_BUMP_STOP:
                    t1state = S3
                    motor_A.set_duty(0)
                    print('motor a off')
                    motor_ticks = ticks_ms()
                    wait_time = 0
                #If a black line has been sensed, next state goes to Return To Home state, motor is turned off.     
                elif black_line_flag.get() == 2:
                    #Return to Home state
                    t1state = S4
                    #DISABLE MOTOR
                    motor_A.set_duty(0)
                    #RESET MOTOR TICKS FOR WAIT TIMER
                    motor_ticks = ticks_ms()
                    #COMMUNICATE TO OTHER TASKS THAT ROMI IS DOING THE BLACK LINE CASE NOW
                    motor_run_flag.put(S_BLACK_LINE)
                    print("motor flag changed:", motor_run_flag.get())
                    #RESET BUFFER FOR RETURNING TO HOME SO THE ROBOT DOES NOT ACCIDENTALLY RECORD A BLACK LINE AGAIN.
                    black_line_buffer = 50

                    
            elif t1state == S3: #OBJECT AVOIDANCE
            
                if motor_run_flag.get() == S_BUMP_STOP: ## GO BACKWARDS AFTER BUMPING INTO THE BOX
                    #GO BACKWARDS IF THE ROMI JUST BUMPED INTO THE BOX
                    controller_A.target_speed = (2*Set_Speed)
                    controller_A.update()
                    if abs(ticks_diff(motor_ticks, ticks_ms()))>(1000*7/Velocity):
                        #IF THE CORRECT AMOUNT OF TIME HAS PASSED, ROMI THEN TURNS TO THE RIGHT
                        motor_run_flag.put(S_BUMP_TURN)
                        
                elif motor_run_flag.get() == S_BUMP_TURN: ## TURN RIGHT
                    #TURN TO THE RIGHT WITH CLOSED LOOP HEADING CONTROL IF THE ROMI TURN HAS NOT YET REACH 90
                    controller_A.target_speed = -Turn_90_heading_gain*heading_error.get()*Set_Speed
                    controller_A.update()
                    
                elif motor_run_flag.get() == S_BUMP_WAIT: ## WAIT FOR A SMALL AMOUNT OF TIME SO THAT THE INERTIA OF TURNING THE ROMI DOES NOT DISRUPT THE OBJECT AVOIDANCE ARC
                    #TURN OFF MOTOR
                    motor_A.set_duty(0)
                    #TICK DOWN WAIT TIME UNTIL 0 
                    if wait_time == 0:
                        #IF WAIT TIME = 0, TELL THE ROMI TO RUN IN A CIRCLE AROUND OBJECT AND RESET WAIT TIME
                        wait_time= 10
                        motor_run_flag.put(S_BUMP_CIRCLE)
                    else:
                        wait_time-=1
                    
                elif motor_run_flag.get() == S_BUMP_CIRCLE: ## MOVE IN A CIRCLE AROUND THE OBJECT
                    #UPDATE SPEED WITH NECESSARY ARC
                    controller_A.target_speed = -2*(Set_Speed)*(1-(Track_Width/(2*impact_radius*25.4)))
                    controller_A.update()
                    
                else: #IF FINISHED, GO BACK TO NORMAL RUNNING/LINE DETECTION
                    #NEXT STATE GOES BACK TO RUN
                    t1state = S2
                    #INDICATE TO MOTOR B AND OTHER TASKS THAT THE ROMI IS LINE DETECT RUNNING NOW
                    motor_run_flag.put(3)
                    #BUTTONS CAN BE PRESSED AGAIN
                    button_ok.put(1)
                    #RESET CONTROLLER TICKS FOR CONTROL PURPOSES
                    controller_A.ticks = ticks_us()
                    
            elif t1state == S4: # Go back to home
            
                if motor_run_flag.get() == S_BLACK_LINE: # ROMI GOES FORWARD FOR A BIT AFTER DETECTING LINE
                    #GO FORWARD WITH THE SET SPEED
                    controller_A.target_speed = -(Set_Speed)
                    controller_A.update()
                    # TEST IF NECESSARY TIME HAS PASSED
                    if abs(ticks_diff(motor_ticks, ticks_ms()))>(1000*10/Velocity):
                        # IF SO, TELL ROMI TO TURN 180
                        motor_run_flag.put(S_TURN_180)
                        
                elif motor_run_flag.get() == S_TURN_180: # ROMI IS TURNING 180 BACK TO INITIAL HEADING
                    # TURN BACK TO INITIAL HEADING WITH CLOSED LOOP HEADING CONTROL UNTIL AT THE NECESSARY VALUE
                    controller_A.target_speed = -Turn_180_heading_gain*heading_error.get()*(Set_Speed)
                    controller_A.update()
                elif motor_run_flag.get() == S_GO_STRAIGHT: # ROMI IS NOW GOING STRAIGHT BACK TO START BOX
                    # MOVE IN A STRAIGHT LINE WITH CLOSED LOOP HEADING CONTROL TO ENSURE IT IS MOVING STRAIGHT
                    controller_A.target_speed = -(Set_Speed)*(1+(heading_error.get()*Return_to_home_heading_gain*Set_Turn))
                    controller_A.update()
                    # TICK DOWN THE BLACK LINE BUFFER UNTIL IT IS 0, AND THEN TEST IF THE LINE SENSOR DETECTS A FULL BLACK LINE. 
                    if black_line_buffer != 0:
                        black_line_buffer-=1
                    elif black_line_flag.get() == 2:
                        #IF A BLACK LINE IS DETECTED, 
                        motor_run_flag.put(S_RTH_NUDGE)
                        black_line_flag.put(0)
                        motor_ticks = ticks_ms()
                    else:
                        black_line_flag.put(1)
                elif motor_run_flag.get() == S_RTH_NUDGE:
                    # Move in a straight line
                    controller_A.target_speed = -(Set_Speed)
                    controller_A.update()
                    # If the necessary amount of time has passed, 
                    if abs(ticks_diff(motor_ticks, ticks_ms()))>(1000*15/Velocity):
                        motor_run_flag.put(0)
                        print("motor flag changed:", motor_run_flag.get())
                        t1state = S1
                        motor_A.set_duty(0)
                        button_ok.put(1)
                        controller_A.ticks = ticks_us()
            else:
            # If the state isnt 0, 1, or 2 we have an invalid state
                raise ValueError("Invalid state")
                
        except ValueError:
            motor_A.disable()
            break
        yield 0

def motor_update_B(shares):
    t2state = 0
    heading_error, button_ok, turn_signal, deltaB, calibration_flag, motor_run_flag = shares
    print("motor B initialize!")
    while True:
        try:
            #print("t2state:",t2state)
            # Implement FSM inside while loop
            if t2state == S0_INIT: #INIT
            
               #Setup Motor B on Timer 8 Channel 3, with the specified pins. 
               tim_B = Timer(1, freq=20_000)
               CH_B = 1
               motor_B = DRV8838(CH_B, tim_B, Pin.cpu.B13, Pin.cpu.B14, Pin.cpu.A8)
               
               #Disable the motor and set duty to 0 to initialize motors to off
               motor_B.set_duty(0)
               motor_B.disable()
               
               #Initialize motor speed based off of turning radius, desired velocity, wheel radius, and track width
               target_speed_B = -(Set_Speed)
               
               #Initialize encoder at timer 2 with specified pins
               encoder_B = Encoder(timer_num=3, pinA=Pin.cpu.B4, pinB=Pin.cpu.B5)
               
               #Setup PI-control object using motor and encoder objects - used for updating speed based off of desired speed, kp, and ki
               controller_B = ProportionalIntegralController(motor_B, encoder_B, target_speed_B, kp = motor_kp, ki = motor_ki)
               
               #Next loop goes to state 1, standby
               t2state = S1
               print("motor B init")
               
            elif t2state == S1: #STANDBY
                #Test if the IMU is calibrated and the motor run flag has been acknowledged by Motor A
                if calibration_flag.get() == 1 and motor_run_flag.get() == 3:
                    #Enable motor
                    motor_B.enable()
                    print('motor B on')
                    #Start timer for speed control purposes
                    controller_B.ticks = ticks_us()
                    #Next loop goes to state 2, Go in a circle
                    t2state=S2
                    
            elif t2state == S2: #GO IN A CIRCLE
                # Update motor speed based on errors and turn signal (turning adjustment)
                controller_B.target_speed = -(Set_Speed)*(1+(turn_signal.get()*Set_Turn))
                controller_B.update()  # Update PI control to adjust speed
                
                # Record the delta encoder value for Motor B to track movement
                deltaB.put(encoder_B.get_delta())
                
                # If motor_run_flag has been turned off (0), stop the motor and return to standby
                if motor_run_flag.get() == 0:
                    t2state = S1
                    motor_B.set_duty(0)
                
                # If a bump is detected, transition to Object Detect state (S3)
                if motor_run_flag.get() == S_BUMP_STOP:
                    t2state = S3
                    motor_B.set_duty(0)
                
                # If a black line is detected, transition to Return to Home state (S4)
                if motor_run_flag.get() == S_BLACK_LINE:
                    t2state = S4
                    
            elif t2state == S3:  # Object Avoidance state (after bump detection)
                if motor_run_flag.get() == S_BUMP_STOP:  # Go backwards after bumping into an obstacle
                    controller_B.target_speed = (2*Set_Speed)  # Set motor to go backwards faster
                    controller_B.update()  # Update PI controller with new speed
                
                elif motor_run_flag.get() == S_BUMP_TURN:  # Turn right after bump detection
                    controller_B.target_speed = heading_error.get()*(0.0125*Set_Speed)  # Turn with proportional heading control
                    controller_B.update()
                
                elif motor_run_flag.get() == S_BUMP_WAIT:  # Wait for a short time to stabilize after turning
                    motor_B.set_duty(0)  # Stop the motor during the wait
                    
                elif motor_run_flag.get() == S_BUMP_CIRCLE:  # Move in a circle around the object
                    # Update speed to make a circular motion around the obstacle (adjust based on track width and impact radius)
                    controller_B.target_speed = -2*(Set_Speed)*(1+(Track_Width/(2*impact_radius*25.4)))
                    controller_B.update()
                
                else:  # If no bump or object is detected, return to normal run (S2)
                    t2state = S2
                    controller_B.ticks = ticks_us()  # Reset control ticks for next cycle
                    
            elif t2state == S4:  # Return to Home state (after black line detection)
                if motor_run_flag.get() == S_BLACK_LINE:  # Robot moves forward a bit after black line detection
                    controller_B.target_speed = -(Set_Speed)  # Move forward with the set speed
                    controller_B.update()
                
                elif motor_run_flag.get() == S_TURN_180:  # Robot needs to turn 180 degrees back to initial heading
                    # Turn back to the initial heading using closed-loop control for heading
                    controller_B.target_speed = heading_error.get()*(Turn_180_heading_gain*Set_Speed)
                    controller_B.update()
                
                elif motor_run_flag.get() == S_GO_STRAIGHT:  # Move straight towards the home position
                    # Adjust the speed to ensure robot is moving straight using heading correction
                    controller_B.target_speed = -(Set_Speed)*(1-(heading_error.get()*Return_to_home_heading_gain*Set_Turn))
                    controller_B.update()
                
                elif motor_run_flag.get() == S_RTH_NUDGE:  # Final nudge forward to reach home position
                    controller_B.target_speed = -(Set_Speed)
                    controller_B.update()
                
                else:  # If no condition matches, return to Standby state (S1) and stop the motor
                    t2state = S1
                    motor_B.set_duty(0)  # Stop the motor
                    controller_B.ticks = ticks_us()  # Reset control ticks for next cycle
            
            else:
            # If the state isnt 0, 1, or 2 we have an invalid state
                raise ValueError("Invalid state")
        except ValueError:
            motor_B.disable()
            break
        yield 0

def IMU_update(shares):
    t3state = 0
    heading_error, black_line_flag, heading, gyroZ, calibration_flag, motor_run_flag = shares
    while True:
        try:
            #print("t3state:",t3state)
            # Implement FSM inside while loop
            if t3state == S0_INIT: #INIT
                #Initialize controller I2C Bus
                controller=pyb.I2C(1, pyb.I2C.MASTER, baudrate=100000)
                #Initialize IMU object as I2C Peripheral
                IMU = BNO055(controller,BNO055_ADDRESS)
                #Next loop goes to calibrate
                t3state = S1
                
            elif t3state == S1: #CALIBRATE
                #Read current status byte of calibration and print i
                if IMU.try_calibration()==0:
                    print("not calibrated")
                    pass
                    #If the proper devices are calibrated in the IMU, raise the calibration flag, and next loop goes to State 2, Wait for button
                else:
                    print("calibrated")
                    t3state = S2
                    calibration_flag.put(True)
                    
            elif t3state == S2: #Wait for button
                heading.put(int(IMU.read_euler_angles())) #Read heading
                gyroZ.put(int(IMU.read_velocity())) #Read Gyro for print
                #Check If the motor run flag is raised
                if motor_run_flag.get() == 1: 
                    #Enable run
                    motor_run_flag.put(2)
                    #Record initial heading
                    IMU_heading_start = (heading.get())
                    #initialize heading offsets
                    IMU_heading_reset_offset = 0
                    IMU_heading_turn_offset = 0
                    #if statements catch edge cases where initial IMU reading would produce impossible heading values down the road
                    if (IMU_heading_start+turn_angle)>360:
                        IMU_heading_turn_offset = 360
                    elif (IMU_heading_start-reset_angle)<0:
                        IMU_heading_reset_offset = 360
                    #Next loop goes to run state
                    t3state = S3
                    #Disable black line detection
                    black_line_flag.put(0)
                    
                    
            elif t3state == S3: #Run
                current_heading = (int(IMU.read_euler_angles())) #Read current heading
                heading.put(current_heading)
                
                if motor_run_flag.get() == 0: #If the motor run flag is 0, go back to waiting for button
                    t3state = S2
                    
                elif motor_run_flag.get() == 1: # If the motor run flag is 1, stop the motors and go back to waiting for button
                    motor_run_flag.put(0)
                    t3state = S2
                    
                elif motor_run_flag.get() == S_BUMP_TURN: #If the robot is currently doing the turn after bumping into the box
                    #Share the heading error for heading control at the motors
                    heading_error.put(IMU_heading_start+turn_angle-(current_heading+IMU_heading_turn_offset)) 
                    # Test to see if the heading value is within a buffer of the initial heading + 90 degrees.
                    if (IMU_heading_start+turn_angle+3) >= (current_heading+IMU_heading_turn_offset) >= (IMU_heading_start+turn_angle-3) :
                        #If so, progress to next bump sequence, waiting. 
                        motor_run_flag.put(S_BUMP_WAIT)
                        
                elif motor_run_flag.get() == S_BUMP_CIRCLE: #Test to see if the robot is running the circle around the object
                    #Test to see if the heading value is within a buffer of the initial heading - 45 degrees
                    if (IMU_heading_start-reset_angle-3) <= (current_heading-IMU_heading_reset_offset) <= (IMU_heading_start-reset_angle+3):
                        #If so, reset the motors so they are in their normal run state
                        motor_run_flag.put(2)
                        #enable full black line detection
                        black_line_flag.put(1)
                        
                elif motor_run_flag.get() == S_TURN_180: #Test to see if ROMI is currently doing 180 spin at the end
                    #Share heading error for closed loop heading control
                    heading_error.put(IMU_heading_start-(current_heading))
                    # Test to see if the heading value is within a buffer of the initial heading
                    if (IMU_heading_start-3) <= current_heading <= (IMU_heading_start+3):
                        #If so, make ROMI go straight back to start
                        motor_run_flag.put(S_GO_STRAIGHT)
                        #Disable black line detection until motor values indicate enough time has passed to sense start box
                        black_line_flag.put(0)
                    
                elif motor_run_flag.get() == S_GO_STRAIGHT: #Test to see if ROMI is currently doing straight run to end
                    #Share heading error for closed loop heading control
                    heading_error.put(IMU_heading_start-(current_heading))
                    
                else: 
                    gyroZ.put(int(IMU.read_velocity())) #Read Gyro for print
                    
            else: #Catch invalid state
                raise ValueError("Invalid state")
        except ValueError:
            break
        yield 0
        
def print_status(shares):
    heading, gyroZ, deltaA, deltaB, calibration_flag, motor_run_flag = shares
    t4state = 0
    while True:
        try:
            #print("t4state:",t4state)
            # Implement FSM inside while loop
            if t4state == S0_INIT: #INIT
                t4state=S1
            elif t4state == S1: #STANDBY FOR CALIBRATION
                if calibration_flag.get() == True:
                    t4state=S2
            elif t4state == S2: # PRINT STATUS
                print(f"heading: {heading.get ()}, yaw speed: {gyroZ.get ()}, deltaA: {deltaA.get ()}, deltaB: {deltaB.get ()}, motor run flag: {motor_run_flag.get()}")
            else:
            # If the state isnt 0, 1, or 2 we have an
            # invalid state
                raise ValueError("Invalid state")
        except ValueError:
            break
        yield 0
        
def line_sensor_update(shares):
    t5state = 0
    black_line_flag, turn_signal, motor_run_flag = shares
    while True:
        try:
            #print("t5state:",t5state)
            #print("t2state:",t2state)
            # Implement FSM inside while loop
            if t5state == S0_INIT: #INIT
                #Setup line sensor object on the necessary pins
                my_linesensor = linesensor(control_pin_odd = Pin.cpu.H1, 
                                           control_pin_even = Pin.cpu.B2, 
                                           output_pins_64 = [Pin.cpu.C2, Pin.cpu.C1, Pin.cpu.C0], 
                                           output_pins_31 = [Pin.cpu.A5, Pin.cpu.A6, Pin.cpu.C4],
                                           )
                #Next state goes to S1, Calibration_standby
                t5state = S1
                
            elif t5state == S1:
                #Checks if IMU is calibrated and motors are on
                if calibration_flag.get() == 1 and motor_run_flag.get() == 3:
                    #If so, Line sensor goes to S2, read.
                    t5state = S2
            elif t5state == S2:
                # Record the current total of all IR readings and calculated line centroid
                total, centroid = (my_linesensor.read_lineArray())
                # If most of the sensors are reading black and the robot is ready to accept a black line, 
                # then the black_line flag gets ticked up to properly handle turning around
                if total>5 and (black_line_flag.get() == 1):
                    print("Black Line!")
                    black_line_flag.put(2)
                else:
                    #Share the centroid as turn signal to manipulate the turn radius.
                    turn_signal.put(centroid*abs(centroid))
                #If the motors are stopping for whatever reason, go back to waiting for run
                if motor_run_flag.get() == (0 or S_BUMP_STOP):
                    #print("line sensor stopped!")
                    t5state = S1
            else:
            # If the state isnt 0, 1, or 2 we have an
            # invalid state
                raise ValueError("Invalid state")
        except ValueError:
            break
        yield 0

def button_pressed_handler(the_pin): #Handles interrupts related to buttons
    if the_pin == 13: # If the user pressed the blue button on the nucleo, indicate so 
        motor_run_flag.put(1)
        print("button pressed")
        button_ok.put(0)
    elif button_ok.get() >= 1: #Else, check if the robot is clear for object detection
        motor_run_flag.put(S_BUMP_STOP)
        #if so, start the bump movement sequence.
        print("button pressed")
        #Reset button ok
        button_ok.put(0)
    return

if __name__ == "__main__":
    
    #Setup Shares
    black_line_flag = task_share.Share('i', thread_protect=False, name="black_line_flag")
    deltaA = task_share.Share('i', thread_protect=False, name="Position A")
    deltaB = task_share.Share('i', thread_protect=False, name="Position B")
    heading = task_share.Share('h', thread_protect=False, name="Heading")
    gyroZ = task_share.Share('h', thread_protect=False, name="GyroZ")
    calibration_flag = task_share.Share('h', thread_protect=False, name="calibration flag")
    motor_run_flag = task_share.Share('h', thread_protect=False, name="motor_run_flag")
    turn_signal = task_share.Share('f', thread_protect=False, name="turn_signal")
    button_ok = task_share.Share('h', thread_protect=False, name="button_ok")
    heading_error = task_share.Share('f', thread_protect=False, name="heading_error")
    
    #Setup tasks
    task1 = cotask.Task(motor_update_A, name="Task_1", priority=1, period=25,
                        profile=True, trace=False, shares=(heading_error, black_line_flag, button_ok, turn_signal, deltaA, calibration_flag, motor_run_flag))
    task2 = cotask.Task(motor_update_B, name="Task_2", priority=1, period=25,
                        profile=True, trace=False, shares=(heading_error, button_ok, turn_signal, deltaB, calibration_flag, motor_run_flag))
    task3 = cotask.Task(IMU_update, name="Task_3", priority=2, period=50,
                        profile=True, trace=False, shares=(heading_error, black_line_flag, heading, gyroZ, calibration_flag, motor_run_flag))
    task4 = cotask.Task(print_status, name="Task_4", priority=3, period=1000,
                        profile=True, trace=False, shares=(heading, gyroZ, deltaA, deltaB, calibration_flag, motor_run_flag))
    task5 = cotask.Task(line_sensor_update, name="Task_5", priority=3, period=25,
                        profile=True, trace=False, shares=(black_line_flag, turn_signal, motor_run_flag))
    
    #Setup Button interrupts
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, button_pressed_handler)
    bump_left = ExtInt(Pin.cpu.C10, ExtInt.IRQ_FALLING, Pin.PULL_UP, button_pressed_handler)
    bump_right = ExtInt(Pin.cpu.C12, ExtInt.IRQ_FALLING, Pin.PULL_UP, button_pressed_handler)
    
    #Add tasks to scheduler
    cotask.task_list.append(task1)
    cotask.task_list.append(task2)
    cotask.task_list.append(task3)
    cotask.task_list.append(task4)
    cotask.task_list.append(task5)
    
    #clean up any goblins hiding somewhere
    gc.collect()
    
    #Attempt to run scheduler
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break
        
    #Print scheduler statistics 
    print('\n' + str(cotask.task_list))
    print(task_share.show_all())
    print(task1.get_trace())
    print('')
