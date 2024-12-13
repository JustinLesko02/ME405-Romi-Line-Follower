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
from time import sleep_ms, ticks_ms, ticks_diff, ticks_us
from DRV8838 import DRV8838
from Encoder import Encoder
from BNO055 import BNO055
from controller import ProportionalController, ProportionalIntegralController
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
# Obstacle handling states
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

Turn_Radius = 6 # inches
Velocity = 20
print(Velocity)
Cycle_Time = Turn_Radius/(Velocity/(2*math.pi)) #seconds
print(Cycle_Time)
Track_Width = 137.5 #mm
Wheel_Radius = 70 #mm
impact_radius = 11
turn_angle = 90
reset_angle = 43
# Variables for obstacle avoidance
reverse_distance = 1000   # Encoder ticks to go backward
circle_time = 3000        # ms to drive in a circle
line_reacquire_time = 3000 # ms after circle to re-enable line sensor

# Shares


# Bump sensors (active low)


# Global variables for obstacle handling
backward_start = 0
circle_start = 0
wait_line_start = 0

def motor_update_A(shares):
    t1state = 0
    heading_error, black_line_flag, button_ok, turn_signal, deltaA, calibration_flag, motor_run_flag= shares
    print("motor A initialize!")
    while True:
        try:
            #print("t1state:",t1state)
            # Implement FSM inside while loop
            if t1state == S0_INIT:
               #Setup Motor A on Timer 8 Channel 3, with the specified pins. 
               tim_A = Timer(8, freq=20_000)
               CH_A = 3
               motor_A = DRV8838(CH_A, tim_A, Pin.cpu.C5, Pin.cpu.C6, Pin.cpu.C8)
               #Disable the motor and set duty to 0 to initialize motors to off
               motor_A.set_duty(0)
               motor_A.disable()
               #Initialize motor speed based off of turning radius, desired velocity, wheel radius, and track width
               target_speed_A = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
               print("A target speed:", target_speed_A)
               #Initialize encoder at timer 2 with specified pins
               encoder_A = Encoder(timer_num=2, pinA=Pin.cpu.A0, pinB=Pin.cpu.A1)
               #Setup PI-control object using motor and encoder objects - used for updating speed based off of desired speed, kp, and ki
               controller_A = ProportionalIntegralController(motor_A, encoder_A, target_speed_A, kp=0.02, ki = 0.05)
               #Next loop goes to state 1, standby
               t1state = S1
               print("motorA init")
               button_ok.put(1)
            elif t1state == S1: #Standby
                #Test if the IMU is calibrated and the motor run flag has been acknowledged by IMU
                if calibration_flag.get() == 1 and motor_run_flag.get() == 2:
                    #Enable Motor
                    motor_A.enable()
                    
                    print('motor a on')
                    #Start timer for speed control purposes
                    controller_A.ticks = ticks_us()
                    #Set motor run flag to 3 for other motor
                    motor_run_flag.put(3)
                    
                    button_ok.put(1)
                    #Next loop goes to state 2, Go in a circle
                    t1state=S2
            elif t1state == S2: #Go in a circle
                #Update motor speed based off of errors
                controller_A.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*(1-(turn_signal.get()*Track_Width/(2*Turn_Radius*25.4)))*1440
                #print("target speed a", target_speed_A)
                controller_A.update()
                deltaA.put(encoder_A.get_delta())
                #If motor_run_flag has been turned off by IMU, set motor to stop and next loop goes to standby
                if motor_run_flag.get() == 0:
                    t1state = S1
                    motor_A.set_duty(0)
                    print('motor a off')
                    
                elif motor_run_flag.get() == S_BUMP_STOP:
                    t1state = S3
                    motor_A.set_duty(0)
                    print('motor a off')
                    motor_ticks = ticks_ms()
                    wait_time = 0
                    
                elif black_line_flag.get() == 2:
                    t1state = S4
                    motor_A.set_duty(0)
                    motor_ticks = ticks_ms()
                    motor_run_flag.put(S_BLACK_LINE)
                    print("motor flag changed:", motor_run_flag.get())
                    black_line_buffer = 50

                    
            elif t1state == S3: #Object Detect
                if motor_run_flag.get() == S_BUMP_STOP:
                    controller_A.target_speed = (2*Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_A.update()
                    if abs(ticks_diff(motor_ticks, ticks_ms()))>(1000*7/Velocity):
                        motor_run_flag.put(S_BUMP_TURN)
                        print("motor flag changed: bump turn")
                elif motor_run_flag.get() == S_BUMP_TURN:
                    controller_A.target_speed = -heading_error.get()*(0.0125*Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_A.update()
                elif motor_run_flag.get() == S_BUMP_WAIT:
                    motor_A.set_duty(0)
                    if wait_time == 0:
                        wait_time= 10
                        motor_run_flag.put(S_BUMP_CIRCLE)
                        print("motor flag changed:", motor_run_flag.get())
                    else:
                        wait_time-=1
                    
                elif motor_run_flag.get() == S_BUMP_CIRCLE:
                    controller_A.target_speed = -2*(Velocity*25.4/(Wheel_Radius*2*math.pi))*(1-(Track_Width/(2*impact_radius*25.4)))*1440
                    controller_A.update()
                else:
                    t1state = S2
                    motor_run_flag.put(3)
                    button_ok.put(1)
                    controller_A.ticks = ticks_us()
                    
            elif t1state == S4: # Go back to home
                if motor_run_flag.get() == S_BLACK_LINE:
                    controller_A.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_A.update()
                    if abs(ticks_diff(motor_ticks, ticks_ms()))>(1000*10/Velocity):
                        motor_run_flag.put(S_TURN_180)
                        print("motor flag changed:", motor_run_flag.get())
                elif motor_run_flag.get() == S_TURN_180:
                    controller_A.target_speed = -heading_error.get()*(0.02*Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_A.update()
                elif motor_run_flag.get() == S_GO_STRAIGHT:
                    controller_A.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*(1+(heading_error.get()*0.2*Track_Width/(2*impact_radius*25.4)))*1440
                    controller_A.update()
                    if black_line_buffer != 0:
                        black_line_buffer-=1
                    elif black_line_flag.get() == 2:
                        motor_run_flag.put(S_RTH_NUDGE)
                        black_line_flag.put(0)
                        motor_ticks = ticks_ms()
                    else:
                        black_line_flag.put(1)
                elif motor_run_flag.get() == S_RTH_NUDGE:
                    controller_A.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_A.update()
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
               target_speed_B = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
               
               #Initialize encoder at timer 2 with specified pins
               encoder_B = Encoder(timer_num=3, pinA=Pin.cpu.B4, pinB=Pin.cpu.B5)
               #Setup PI-control object using motor and encoder objects - used for updating speed based off of desired speed, kp, and ki
               controller_B = ProportionalIntegralController(motor_B, encoder_B, target_speed_B, kp=0.02, ki = 0.05)
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
                #Update motor speed based off of errors
                controller_B.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*(1+(turn_signal.get()*Track_Width/(2*Turn_Radius*25.4)))*1440
                #print("target speed b", target_speed_B)
                controller_B.update()
                deltaB.put(encoder_B.get_delta())
                #If motor_run_flag has been turned off by IMU, set motor to stop and next loop goes to standby
                if motor_run_flag.get() == 0:
                    t2state = S1
                    motor_B.set_duty(0)
                    print('motor b off,')
                if motor_run_flag.get() == S_BUMP_STOP:
                    t2state = S3
                    motor_B.set_duty(0)
                if motor_run_flag.get() == S_BLACK_LINE:
                    t2state = S4
                    
            elif t2state == S3: #Object Detect
                if motor_run_flag.get() == S_BUMP_STOP:
                    controller_B.target_speed = (2*Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_B.update()
                elif motor_run_flag.get() == S_BUMP_TURN:
                    controller_B.target_speed = heading_error.get()*(0.0125*Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_B.update()
                elif motor_run_flag.get() == S_BUMP_WAIT:
                    motor_B.set_duty(0)
                elif motor_run_flag.get() == S_BUMP_CIRCLE:
                    controller_B.target_speed = -2*(Velocity*25.4/(Wheel_Radius*2*math.pi))*(1+(Track_Width/(2*impact_radius*25.4)))*1440
                    controller_B.update()
                else:
                    t2state = S2
                    controller_B.ticks = ticks_us()
                    
            elif t2state == S4:
                if motor_run_flag.get() == S_BLACK_LINE:
                    controller_B.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_B.update()
                elif motor_run_flag.get() == S_TURN_180:
                    
                    controller_B.target_speed = heading_error.get()*(0.02*Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_B.update()
                    
                elif motor_run_flag.get() == S_GO_STRAIGHT:
                    controller_B.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*(1-(heading_error.get()*0.2*Track_Width/(2*impact_radius*25.4)))*1440
                    controller_B.update()
                elif motor_run_flag.get() == S_RTH_NUDGE:
                    controller_B.target_speed = -(Velocity*25.4/(Wheel_Radius*2*math.pi))*1440
                    controller_B.update()
                else:
                    t2state = S1
                    motor_B.set_duty(0)
                    controller_B.ticks = ticks_us()
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
                gyroZ.put(int(IMU.read_velocity())) #Read Gyro
                #If the motor run flag is raised, increase flag for motor A, record IMU heading start, and increase the initial IMU count
                if motor_run_flag.get() == 1: 
                    motor_run_flag.put(2)
                    IMU_heading_start = (heading.get())
                    IMU_heading_reset_offset = 0
                    IMU_heading_turn_offset = 0
                    
                    if (IMU_heading_start+turn_angle)>360:
                        IMU_heading_turn_offset = 360
                    elif (IMU_heading_start-reset_angle)<0:
                        IMU_heading_reset_offset = 360
                    t3state = S3
                    initial_IMU_count = 6
                    black_line_flag.put(0)
                    
                    
            elif t3state == S3: #Run
                current_heading = (int(IMU.read_euler_angles())) #Read heading
                heading.put(current_heading)
                #Counts down from 6 so that the IMU heading condition does not immediately trigger to stop the motors
                if initial_IMU_count>0:
                    initial_IMU_count-=1
                    #print("IMU count:", initial_IMU_count)
                #Tests if the heading is the same as the initial heading and the imu count is 0 or if the button was pressed again
                elif motor_run_flag.get() == 0:
                    t3state = S2
                elif motor_run_flag.get() == 1:
                    #If so, stop the motors and next loop goes to state 2, wait for button
                    motor_run_flag.put(0)
                    t3state = S2
                elif motor_run_flag.get() == S_BUMP_TURN:
                    heading_error.put(IMU_heading_start+turn_angle-(current_heading+IMU_heading_turn_offset))
                    if (IMU_heading_start+turn_angle+3) >= (current_heading+IMU_heading_turn_offset) >= (IMU_heading_start+turn_angle-3) :
                        motor_run_flag.put(S_BUMP_WAIT)
                        print("motor flag changed:", motor_run_flag.get())
                elif motor_run_flag.get() == S_BUMP_CIRCLE:
                    heading_error.put(IMU_heading_start-reset_angle-(current_heading+IMU_heading_reset_offset))
                    if (IMU_heading_start-reset_angle-3) <= (current_heading-IMU_heading_reset_offset) <= (IMU_heading_start-reset_angle+3):
                        motor_run_flag.put(2)
                        black_line_flag.put(1)
                        print("motor flag changed:", motor_run_flag.get())
                elif motor_run_flag.get() == S_TURN_180:
                    heading_error.put(IMU_heading_start-(current_heading))
                    if (IMU_heading_start-3) <= current_heading <= (IMU_heading_start+3):
                        motor_run_flag.put(S_GO_STRAIGHT)
                        print("motor flag changed:", motor_run_flag.get())
                        black_line_flag.put(0)
                elif motor_run_flag.get() == S_GO_STRAIGHT:
                    heading_error.put(IMU_heading_start-(current_heading))
                else: 
                    gyroZ.put(int(IMU.read_velocity())) #Read Gyro
                #print(IMU.read_euler_angles())
                    
            else:
            # If the state isnt 0, 1, or 2 we have an
            # invalid state
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
            elif t4state == S1: #STANDBY
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
                my_linesensor = linesensor(control_pin_odd = Pin.cpu.H1, 
                                           control_pin_even = Pin.cpu.B2, 
                                           output_pins_64 = [Pin.cpu.C2, Pin.cpu.C1, Pin.cpu.C0], 
                                           output_pins_31 = [Pin.cpu.A5, Pin.cpu.A6, Pin.cpu.C4],
                                           )
                print("line sensor initialize!")
                print("IR light on!")
                t5state = S1
                
            elif t5state == S1:
                if calibration_flag.get() == 1 and motor_run_flag.get() == 3:
                    t5state = S2
            elif t5state == S2:
                total, centroid = (my_linesensor.read_lineArray())
                if total>5 and (black_line_flag.get() == 1):
                    print("Black Line!")
                    black_line_flag.put(2)
                if centroid == 7:
                    turn_signal.put(0)
                else:
                    turn_signal.put(centroid*abs(centroid))
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

def button_pressed_handler(the_pin):
    print(the_pin)
    if the_pin == 13:
        motor_run_flag.put(1)
        print("button pressed")
        button_ok.put(0)
    elif button_ok.get() >= 1:
        motor_run_flag.put(S_BUMP_STOP)
        print("button pressed")
        button_ok.put(0)
    return

if __name__ == "__main__":
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

    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, button_pressed_handler)
    bump_left = ExtInt(Pin.cpu.C10, ExtInt.IRQ_FALLING, Pin.PULL_UP, button_pressed_handler)
    bump_right = ExtInt(Pin.cpu.C12, ExtInt.IRQ_FALLING, Pin.PULL_UP, button_pressed_handler)
    cotask.task_list.append(task1)
    cotask.task_list.append(task2)
    cotask.task_list.append(task3)
    cotask.task_list.append(task4)
    cotask.task_list.append(task5)

    gc.collect()

    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break

    print('\n' + str(cotask.task_list))
    print(task_share.show_all())
    print(task1.get_trace())
    print('')