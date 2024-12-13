#-------------------------------------------------------------------------------------------------*
# linesensor (linesensor.py)
# Description:
# This module defines a class `linesensor` to interface with a line sensor. The line sensor
# measures the reflectance of the surface opposite the sensor through IR light. 
#
# Key Responsibilities:
#   - Initialize line sensor objects and thresholds for recording values
#   - 
# What could I add to improve this file:
#    1.
#-------------------------------------------------------------------------------------------------*

from pyb import Pin, ADC
import time
class linesensor:
    """
    A class to read from a line sensor using a timer in MicroPython. 
    """

    def __init__(self, control_pin_odd, control_pin_even, output_pins_64, output_pins_31):
        """
        Initializes the encoder by configuring the timer and pins for quadrature decoding.

        Parameters:
        control_pin_odd (Pin):Pin for controlling the IR source for odd pins
        control_pin_even (Pin): Pin for controlling the IR source for even pins
        output_pins_64(Pin): List of pins for left side of line sensor
        output_pins_31(Pin): List of pins for right side of line sensor
        
        
        """
        #CONFIGURE BOTH CONTROL PINS AS OUTPUTS SO THAT ALL IR LIGHTS TURN ON
        odd_controlpin = Pin(control_pin_odd, mode = Pin.OUT_PP)
        odd_controlpin.high()
        even_controlpin = Pin(control_pin_even, mode = Pin.OUT_PP)
        even_controlpin.high()
        # MAKE ATTRIBUTE FOR PIN LIST
        self.Pins = output_pins_64+output_pins_31
        # MAKE OUTPUT PIN ATTRIBUTE
        self.PinOutput =[0]*len(self.Pins)
        self.ADC_obj =  [0]*len(self.Pins)
        self.ADC_read = [0]*len(self.Pins)
        self.ADC_reading = [0]*len(self.Pins)
        self.ADC_threshold = [2000, 1000, 1000, 1000, 1000, 1000]
        
    def read_lineArray(self):
        # SLEEP FOR A SMALL AMOUNT OF TIME TO PREVENT FIRST PIN FROM READING HIGH (???)
        time.sleep(0.00003)
        # FOR EACH OUTPUT PIN
        for i, pin in enumerate(self.Pins):
            # SET PIN TO OUT AND DRAW IT HIGH
            self.PinOutput[i] = Pin(pin, mode = Pin.OUT_PP)
            self.PinOutput[i].high()
            # SET PIN TO INPUT AND CONFIGURE ADC
            self.PinOutput[i] = Pin(pin, mode = Pin.IN)
            self.ADC_obj[i] = ADC(self.PinOutput[i])
            #WAIT FOR 5 uS
            time.sleep(0.000005)
            #READ ADC VALUE AND COMPARE IT TO THRESHOLD TO SEE IF IT IS READING A BLACK LINE
            self.ADC_reading[i] = self.ADC_obj[i].read()
            self.ADC_read[i] = self.ADC_threshold[i]<self.ADC_reading[i]
        #RESET CENTROID CALCULATIONS
        centroid = 0
        weight = 0
        total = 0
        # FIND THE WEIGHTED TOTAL VALUE OF THE SENSORS, AND THE FLAT TOTAL
        for ADCindex, ADCval in enumerate(self.ADC_read):
            weight += ADCval*(ADCindex-2.5) 
            total += ADCval
        # IF NO LINE WAS SENSED OUTPUT A CENTROID OF 0
        if total == 0:
            centroid = 0
        # ELSE, CALCULATE CENTROID USING WEIGHTED TOTAL AND TOTAL
        else:
            centroid = weight/total
        # RETURN 
        return total, centroid
        
        
if __name__ == '__main__':
    
    my_linesensor = linesensor(control_pin_odd = Pin.cpu.H1, 
                               control_pin_even = Pin.cpu.B2, 
                               output_pins_64 = [Pin.cpu.C1, Pin.cpu.C0], 
                               output_pins_31 = [Pin.cpu.A5, Pin.cpu.A6, Pin.cpu.C4],
                               )
    my_linesensor.odd_controlpin.high()
    my_linesensor.even_controlpin.high()
    while True:
        my_linesensor.read_lineArray()    
        time.sleep(0.5)



    