#-------------------------------------------------------------------------------------------------*
# ENCODER MODULE (Encoder.py)
#
# Description:
# This module defines a class `Encoder` to interface with a quadrature encoder. The encoder
# tracks the position and velocity of a motor by counting pulses from the encoder using hardware 
# timers configured in quadrature decoding mode.
#
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

from pyb import Pin, Timer
import time
class Encoder:
    """
    A class to read from a quadrature encoder using a timer in MicroPython. This class tracks
    the position and velocity of the motor and handles overflow/underflow in the timer.
    """

    def __init__(self, timer_num, pinA, pinB, prescaler=0, period=0xFFFF):
        """
        Initializes the encoder by configuring the timer and pins for quadrature decoding.

        Parameters:
        timer_num (int): The timer number (e.g., Timer 2, Timer 3) to be used for quadrature decoding.
        pinA (Pin): Pin object for channel A of the encoder.
        pinB (Pin): Pin object for channel B of the encoder.
        prescaler (int): Optional, timer prescaler to control the count rate (default is 0).
        period (int): Optional, period (auto-reload) value of the timer (default is 0xFFFF for 16-bit timers).
        """
        # Initialize the timer with the specified prescaler and period
        self.timer = Timer(timer_num, prescaler=prescaler, period=period)

        # Set timer channels for quadrature decoding mode (ENC_AB mode)
        self.timer.channel(1, mode=Timer.ENC_AB, pin=pinA)
        self.timer.channel(2, mode=Timer.ENC_AB, pin=pinB)

        # Initialize position, previous counter value, and delta (velocity)
        self.position = 0
        self.prev_count = 0
        self.delta = 0

    def update(self):
        """
        Updates the encoder's position by reading the timer's counter value and calculating the change in position (delta).
        Handles overflow/underflow in the timer to ensure accurate tracking.
        """
        # Read the current counter value from the timer
        current_count = self.timer.counter()

        # Calculate the change in position (delta)
        self.delta = current_count - self.prev_count

        # Check for overflow or underflow and adjust delta accordingly
        if self.delta > (self.timer.period() // 2):
            self.delta -= self.timer.period() + 1
        elif self.delta < -(self.timer.period() // 2):
            self.delta += self.timer.period() + 1

        # Update the total position by adding delta to the current position
        self.position += self.delta

        # Store the current count as the previous count for the next update
        self.prev_count = current_count

    def get_position(self):
        """
        Returns the current position of the encoder.
        The position is the total accumulated movement since the last reset or initialization.
        """
        return self.position

    def get_delta(self):
        """
        Returns the change in position (delta) since the last update.
        The delta represents the velocity of the motor based on the encoder's count.
        """
        return self.delta

    def zero(self):
        """
        Resets the encoder's position to zero.
        This method adjusts the reference so future updates start from the new zero point.
        """
        self.position = 0
        self.prev_count = self.timer.counter()
if __name__ == '__main__':
    encoder_A = Encoder(timer_num=2, pinA=Pin.cpu.A0, pinB=Pin.cpu.A1)
    encoder_B = Encoder(timer_num=3, pinA=Pin.cpu.B4, pinB=Pin.cpu.B5)
    while True:
        encoder_A.update()
        encoder_B.update()
        print(encoder_A.get_position())
        print(encoder_B.get_position())
        time.sleep(0.5)



    