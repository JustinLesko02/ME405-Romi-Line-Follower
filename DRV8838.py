'''!@file L6206.py
'''
from pyb import Pin, Timer
import time

class DRV8838:
    '''!@brief A driver class for one channel of the L6206.
        @details    Objects of this class can be used to apply PWM to a given
                    DC motor on one channel of the L6206 from ST Microelectronics.
'''
    def __init__ (self, PWM_CH, PWM_tim, EN_pin, Dir_Pin, Effort_Pin):
        self.Eff_Pin = Pin(Effort_Pin, mode=Pin.OUT_PP)
        self.EN = Pin(EN_pin, mode=Pin.OUT_PP)
        self.Dir = Pin(Dir_Pin, mode=Pin.OUT_PP)
        self.CH = PWM_tim.channel(PWM_CH, pin=self.Eff_Pin, mode=Timer.PWM)
        #print('Motor Initialized:', self)
        pass
    
    def set_duty (self, duty):
        if duty>100:
            print('Positive duty too high')
        elif duty>=0:
            self.CH.pulse_width_percent(duty)
            self.Dir.high()
        elif duty<0:
            self.CH.pulse_width_percent(-duty)
            self.Dir.low()
        else:
            print('Negative duty too high')
        #print('duty:', duty)
        pass
    
    def enable(self):
        self.EN.high()
        #print('enabled')
        pass
    
    def disable(self):
        self.EN.low()
        #print('disabled')
        pass
    

if __name__ == '__main__':
    tim_A = Timer(8, freq=20_000)
    CH_A = 3
    motor_A = DRV8838(CH_A, tim_A, Pin.cpu.C5, Pin.cpu.C6, Pin.cpu.C8)
    
    tim_B = Timer(1, freq=20_000)
    CH_B = 3
    motor_B = DRV8838(CH_B, tim_B, Pin.cpu.B13, Pin.cpu.B14, Pin.cpu.B15)

    # Test Motor A
    motor_A.enable()
    motor_A.set_duty(60)  # Spin forward at 60%
    time.sleep(2)  # Run for 2 seconds
    motor_A.set_duty(0)  # Stop motor A
    motor_A.disable()

    # Test Motor B
    motor_B.enable()
    motor_B.set_duty(60)  # Spin forward at 60%
    time.sleep(2)  # Run for 2 seconds
    motor_B.set_duty(0)  # Stop motor B
    motor_B.disable()