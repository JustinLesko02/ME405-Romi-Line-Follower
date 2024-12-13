#-------------------------------------------------------------------------------------------------*
# linesensor (linesensor.py)
#
# Description:
# This module defines a class `linesensor` to interface with a line sensor. The line sensor
# measures the reflectance of the surface opposite the sensor through IR light. 

# Key Responsibilities:
#   - Initialize the encoder using hardware timers for quadrature decoding.
#   - Update the encoder's position and velocity (delta) based on the timer counts.
#   - Handle overflow and underflow in the timer to ensure correct position tracking.
#   - Provide methods to get the current position, delta (velocity), and reset position to zero.
#
# What could I add to improve this file:
#    1. Add error handling to ensure valid encoder values and edge cases.
#    2. Implement more detailed logging for position and delta values, especially during rapid movement.
#    3. Provide an option to choose different timer configurations or alternative counting modes (e.g., 4X quadrature decoding).
#    4. Add support for handling multiple encoders if needed in more complex systems.
#-------------------------------------------------------------------------------------------------*

from pyb import Pin, Timer, ADC
import time
class linesensor:
    """
    A class to read from a quadrature encoder using a timer in MicroPython. This class tracks
    the position and velocity of the motor and handles overflow/underflow in the timer.
    """

    def __init__(self, control_pin_odd, control_pin_even, output_pins_64, output_pins_31):
        """
        Initializes the encoder by configuring the timer and pins for quadrature decoding.

        Parameters:
        timer_num (int): The timer number (e.g., Timer 2, Timer 3) to be used for quadrature decoding.
        pinA (Pin): Pin object for channel A of the encoder.
        pinB (Pin): Pin object for channel B of the encoder.
        prescaler (int): Optional, timer prescaler to control the count rate (default is 0).
        period (int): Optional, period (auto-reload) value of the timer (default is 0xFFFF for 16-bit timers).
        """
        self.odd_controlpin = Pin(control_pin_odd, mode = Pin.OUT_PP)
        self.odd_controlpin.high()
        self.even_controlpin = Pin(control_pin_even, mode = Pin.OUT_PP)
        self.even_controlpin.high()
        self.odd_output= 4*[0]
        self.Pins = output_pins_64+output_pins_31
        self.PinOutput =[0]*len(self.Pins)
        self.ADC_obj =  [0]*len(self.Pins)
        self.ADC_read = [0]*len(self.Pins)
        self.ADC_reading = [0]*len(self.Pins)
        self.ADC_threshold = [2000, 1000, 1000, 1000, 1000, 1000]
    def read_lineArray(self):
        time.sleep(0.00003)
        for i, pin in enumerate(self.Pins):
            self.PinOutput[i] = Pin(pin, mode = Pin.OUT_PP)
            self.PinOutput[i].high()
            self.PinOutput[i] = Pin(pin, mode = Pin.IN)
            self.ADC_obj[i] = ADC(self.PinOutput[i])
            time.sleep(0.000005)
            self.ADC_reading[i] = self.ADC_obj[i].read()
            self.ADC_read[i] = self.ADC_threshold[i]<self.ADC_reading[i]
        #print(self.ADC_reading)
        centroid = 0
        weight = 0
        total = 0
        for ADCindex, ADCval in enumerate(self.ADC_read):
            weight += ADCval*(ADCindex-2.5)
            total += ADCval
        if total == 0:
            centroid = 7
        else:
            centroid = weight/total
        #print(centroid)
        return total, centroid
        
        
if __name__ == '__main__':
    
    my_linesensor = linesensor(control_pin_odd = Pin.cpu.H1, 
                               control_pin_even = Pin.cpu.B2, 
                               output_pins_64 = [Pin.cpu.C1, Pin.cpu.C0], 
                               output_pins_31 = [Pin.cpu.A5, Pin.cpu.A6, Pin.cpu.C4],
                               )
    my_linesensor.odd_controlpin.high()
    my_linesensor.even_controlpin.high()
    '''for x in range(len(ADC_time)):
        
        my_linesensor.odd_output[1] = Pin(Pin.cpu.C2, mode = Pin.OUT_PP)
        my_linesensor.odd_output[1].high()
        my_linesensor.odd_output[1] = Pin(Pin.cpu.C2, mode = Pin.IN)
        my_ADC = ADC(my_linesensor.odd_output[1])
        for i in range(len(ADC_read)):
            ADC_read[i] = my_ADC.read()
            time.sleep(0.00001)
            print(ADC_read[i])
            if ADC_read[i]<400:
                ADC_time[x] = (i*0.00001)
                print("It took", i*0.00001, "seconds to reach 10")
                break
    
    print("avg =", sum(ADC_time)/len(ADC_time))'''
    while True:
        my_linesensor.read_lineArray()    
        time.sleep(0.5)



    