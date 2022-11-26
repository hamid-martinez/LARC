import serial
import time

#'/dev/ttyUSB0', 9600, timeout=1, for arduino mega at Rpi 4
# This class will handle the communication with the Raspberry Pi
# The objective is to send the commands to control the robot

class Rpi_Comm():

    # Upon start, the object asks for the usb port to send data to, the baudrate and the timeout
    def __init__(self, usb_port, baud_rate, time_out):

        # Variable declaration for usage
        self.usb_port = str(usb_port)
        self.baud_rate = int(baud_rate)
        self.time_out = int(time_out)

        # Set the communication to the port with the specified parameters using the pySerial library
        self.arduino = serial.Serial(self.usb_port, self.baud_rate, timeout=self.time_out)
        self.arduino.reset_input_buffer() # flush input buffer, discarding all its contents

        # The list of available comnmands to send to control the robot
        self.cmd_list = ["F","B","R","L","TR","TL","PU","PD","PZ","EM"]
        self.special_cmd_list = ["PID","T1","T2","T3","T4","M"]

    def send_command(self, cmd):

        # Variable declaration for usage
        self.cmd = cmd
        self.cmd_to_send = self.cmd.upper() # convert entered command to all caps
        self.cmd_to_analyze = self.cmd_to_send.split(",") # split the command at the comma, returns a list

        if self.cmd_to_analyze[0] in self.special_cmd_list:
            self.cmd_to_analyze = self.cmd_to_analyze[1].split("#")
        

        # If the first part of the command is not in the list return error
        elif self.cmd_to_analyze[0] not in self.cmd_list and self.cmd_to_analyze[0] not in self.special_cmd_list:
            return "Error: Command not recognized"
        
        # If the second part of the command is not a numeric value return error, M,F#1200
        elif self.cmd_to_analyze[1].isnumeric() != True:
            return "Error: Incorrect numeric value"

        # If all checks are passed, then write to the serial port the given command
        self.arduino.write(self.cmd_to_send.encode("utf-8"))
        time.sleep(0.5) #wait for port to answer

    def read_command(self):

        while True:
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode("utf-8").rstrip()
                if line != "Ready":
                    continue
                elif line == "Ready":
                    return line
