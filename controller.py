from Encoder import Encoder
from DRV8838 import DRV8838
from pyb import Timer, Pin
from time import ticks_us, ticks_diff

class ProportionalController:
    def __init__(self, motor, encoder, target_speed, kp):
        """
        Initialize the proportional controller.
        
        Parameters:
            motor (DRV8838): The motor driver instance to control.
            encoder (Encoder): The encoder instance providing position feedback.
            target_position (int): The desired position to reach.
            kp (float): The proportional gain for the controller.
        """
        self.motor = motor
        self.encoder = encoder
        self.target_speed = target_speed
        self.kp = kp
        self.ticks = 0
    def update(self):
        """
        Update the control output based on the current position.
        """
        
        # Read the current position from the encoder
        self.ticksDiff = ticks_diff(ticks_us(), self.ticks)
        
        self.encoder.update()
        
        speed = self.encoder.get_delta()*1000000/self.ticksDiff
        
        # Calculate the error
        error = self.target_speed - speed
        
        # Calculate the proportional control signal
        control_signal = self.kp * error

        # Clamp the control signal within the allowable motor range (-100 to 100)
        control_signal = max(min(control_signal, 100), -100)
        
        # Send the control signal to the motor
        self.motor.set_duty(control_signal)
        
        # Optional: Print status for debugging
        print(f"Speed: {speed}, Target: {self.target_speed}, Error: {error}, Control Signal: {control_signal}, Time: {self.ticksDiff}")
        self.ticks = ticks_us()
    
class ProportionalIntegralController:
    def __init__(self, motor, encoder, target_speed, kp, ki):
        """
        Initialize the proportional controller.
        
        Parameters:
            motor (DRV8838): The motor driver instance to control.
            encoder (Encoder): The encoder instance providing position feedback.
            target_position (int): The desired position to reach.
            kp (float): The proportional gain for the controller.
        """
        self.motor = motor
        self.encoder = encoder
        self.target_speed = target_speed
        self.kp = kp
        self.ki = ki
        self.ticks = ticks_us()
        self.error_I = 0
    def update(self):
        """
        Update the control output based on the current position.
        """
        # Read the current position from the encoder
        self.ticksDiff = ticks_diff(ticks_us(), self.ticks)
        
        self.encoder.update()
        self.error_I = self.error_I + (self.target_speed*self.ticksDiff/1000000) - self.encoder.get_delta() 
        speed = self.encoder.get_delta()*1000000/self.ticksDiff
        # Calculate the error
        error_p = self.target_speed - speed
        error = self.target_speed - speed
        # Calculate the proportional control signal
        control_signal = self.kp * error_p + self.ki*self.error_I

        # Clamp the control signal within the allowable motor range (-100 to 100)
        control_signal = max(min(control_signal, 100), -100)
        
        # Send the control signal to the motor
        self.motor.set_duty(control_signal)
        
        # Optional: Print status for debugging
        #print(f"Speed: {speed}, Target: {self.target_speed}, Error: {error}, Control Signal: {control_signal}, Time: {self.ticksDiff}")
        self.ticks = ticks_us()
# Define tasks to use the controller class for each motor
# Target positions for motor A and motor B
# for both, target position needs to be updated based on:
# physical requirements, physical constraints, system capabilities

def motor_A_task(shares):
    motor = DRV8838(PWM_CH=3, PWM_tim=Timer(8, freq=20_000), 
                    EN_pin=Pin.cpu.C5, Dir_Pin=Pin.cpu.C6, Effort_Pin=Pin.cpu.C8)
    motor.enable()
    encoder = Encoder(timer_num=2, pinA=Pin.cpu.A0, pinB=Pin.cpu.A1)
    
    # Initialize the controller for motor A
    controller_A = ProportionalController(motor, encoder, target_position=1000, kp=0.)
    
    while True:
        controller_A.update()
        yield 0

def motor_B_task(shares):
    motor = DRV8838(PWM_CH=3, PWM_tim=Timer(1, freq=20_000), 
                    EN_pin=Pin.cpu.B13, Dir_Pin=Pin.cpu.B14, Effort_Pin=Pin.cpu.B15)
    motor.enable()
    encoder = Encoder(timer_num=3, pinA=Pin.cpu.B4, pinB=Pin.cpu.B5)
    
    # Initialize the controller for motor B
    controller_B = ProportionalController(motor, encoder, target_position=1000, kp=0.5)
    
    while True:
        controller_B.update()
        yield 0
