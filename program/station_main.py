import RPi.GPIO as GPIO
from google_sheet_comm.sheet_comm import Sheet_Comm
from time import sleep

sheet = "Inventory"
work_sheet = "Products"
sensor_state_sheet = [2,4] # Location at google sheet [row,column]
incoming_sheet = [2,5]

# Sheet communication
sheet = Sheet_Comm(sheet, work_sheet)

enable = 11
dir = 16
pwm = 12
sensor = 37

GPIO.setwarnings(False) # Avoid warnings messages at start
GPIO.setmode(GPIO.BOARD)

GPIO.setup(dir, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(pwm, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(enable, GPIO.OUT, initial = GPIO.HIGH)
GPIO.setup(sensor, GPIO.IN)

motor = GPIO.PWM(pwm,100)

while True:

    incoming = sheet.get_cell_value(incoming_sheet[0], incoming_sheet[1])
    sleep(0.5)

    while incoming == "1":

        sensor_state = GPIO.input(sensor)

        if sensor_state == 1:
            motor.start(0)
            GPIO.output(enable, GPIO.LOW)
            GPIO.output(dir, GPIO.HIGH)
            motor.ChangeDutyCycle(30)

        elif sensor_state == 0:
            motor.ChangeDutyCycle(0)
            GPIO.output(enable, GPIO.HIGH)
            motor.stop()
            sheet.update_cell_value(sensor_state_sheet[0], sensor_state_sheet[1], "1")
            sleep(0.5)
            sheet.update_cell_value(incoming_sheet[0], incoming_sheet[1], "0")
            sleep(0.5)
            
            break
