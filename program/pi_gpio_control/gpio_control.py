import time
import RPi.GPIO as GPIO

# This library controls the GPIO pins of the Raspberry Pi.
# It uses basic functions to create an object that will utilize one or more gpio pins from the Rpi.
# For example, if a mosfet needs to be controlled with a gpio signal, a mosfet object can be created to use funtions to turn it on or off accordingly.
# This way all the necesarry code to control a GPIO signal will be managed by the class.

class Gpio_Control():
    
    # The class takes the pin to be used on the Rpi and the type of pin numbering to be used.
    def __init__(self, pin_mode, pin_type, pins):
        
        self.pin_mode = str(pin_mode).upper() # Turn the input to upper to avoid erros and set it as a string.
        self.pins = list(pins) # Input should be a list
        self.pin_type = str(pin_type).upper() # to know the configuration type of the pin or pins as input or output

        GPIO.setwarnings(False) # Avoid warnings messages at start
        
        # In case the input is misspelled, return an error
        if self.pin_mode != "BOARD" and self.pin_mode != "BCM":

            return "Pin mode set incorrectly"

        elif self.pin_type != "OUT" and self.pin_type != "IN":

            return "Pin type set incorrectly"

        # The Rpi takes pins of two types, BMC or BOARD, refering to the physical numbering or the digital GPIO.
        elif self.pin_mode == "BOARD":

            GPIO.setmode(GPIO.BOARD) # Set to read by number of pin

        elif self.pin_mode == "BCM":

            GPIO.setmode(GPIO.BCM) # Set to read by GPIO number

        for self.pin in self.pins:

            if self.pin_type == "OUT":
        
                # Configure the pins to be output and initialize in low
                GPIO.setup(self.pin, GPIO.OUT, initial = GPIO.LOW)

            elif self.pin_type == "IN":

                # Configure the pins to be input
                GPIO.setup(self.pin, GPIO.IN)

    def state_on(self):

        if self.pin_type == "OUT":
            for self.pin in self.pins:

                # Set the output of the pins
                GPIO.output(self.pin, GPIO.HIGH)        
                time.sleep(0.1) # delay

        elif self.pin_type == "IN":
            return "Error: Pins set as input"
            
    def state_off(self):

        if self.pin_type == "OUT":
            for self.pin in self.pins:
        
                # Set the output of the pins
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(0.1) # delay

        elif self.pin_type == "IN":
            return "Error: Pins set as input"

    def read_pin(self):

        values = []

        if self.pin_type == "IN":
            for self.pin in self.pins:
        
                # Set the output of the pins
                value = GPIO.input(self.pin)
                time.sleep(0.1) # delay
                values.append(value)

                return values

        elif self.pin_type == "OUT":
            return "Error: Pins set as output"

    def set_pwm(self, frequency, duty_cycle, stop = False):

        self.frequency = int(frequency)
        self.duty_cycle = int(duty_cycle)
        self.stop = bool(stop)

        if self.duty_cycle not in range(0,101):
            return "Error: Duty cycle out of range [0-100]"

        if self.pin_type == "OUT":
            for self.pin in self.pins:
        
                # Set the output of the pins
                pwm_pin = GPIO.PWM(self.pin, self.frequency)

                if self.stop == False:

                    pwm_pin.start(self.duty_cycle)
                    time.sleep(0.1) # delay

                elif self.stop == True:

                    pwm_pin.stop()
                    time.sleep(0.1) # delay

        elif self.pin_type == "IN":
            return "Error: Pins set as input"
