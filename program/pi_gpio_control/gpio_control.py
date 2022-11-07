import time
import RPi.GPIO as GPIO

# This library controls the GPIO pins of the Raspberry Pi.
# It uses basic functions to create an object that will utilize one or more gpio pins from the Rpi.
# For example, if a mosfet needs to be controlled with a gpio signal, a mosfet object can be created to use funtions to turn it on or off accordingly.
# This way all the necesarry code to control a GPIO signal will be managed by the class.

class Gpio_Control():
    
    # The class takes the pin to be used on the Rpi and the type of pin numbering to be used.
    def __init__(self, pin_mode, pins):
        
        self.pin_mode = str(pin_mode).upper() # Turn the input to upper to avoid erros and set it as a string.
        self.pins = list(pins) # Input should be a list

        GPIO.setwarnings(False) # Avoid warnings messages at start
        
        # In case the input is misspelled, return an error
        if self.pin_mode != "BOARD" and self.pin_mode != "BCM":

            return "Pin type set incorrectly"

        # The Rpi takes pins of two types, BMC or BOARD, refering to the physical numbering or the digital GPIO.
        elif self.pin_mode == "BOARD":

            GPIO.setmode(GPIO.BOARD) # Set to read by number of pin

        elif self.pin_mode == "BCM":

            GPIO.setmode(GPIO.BCM) # Set to read by GPIO number

        for self.pin in self.pins:
        
            # Configure the pins to be output and initialize in low
            GPIO.setup(self.pin, GPIO.OUT, initial = GPIO.LOW)

    def state_on(self):
        
        for self.pin in self.pins:

            # Set the output of the pins
            GPIO.output(self.pin, GPIO.HIGH)        
            time.sleep(0.1) # delay
            
    def state_off(self):

        for self.pin in self.pins:
        
            # Set the output of the pins
            GPIO.output(self.pin, GPIO.LOW)

            time.sleep(0.1) # delay

