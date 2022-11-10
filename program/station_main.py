import RPi.GPIO as GPIO
from google_sheet_comm.sheet_comm import Sheet_Comm
from time import sleep

sheet = "Inventory"
work_sheet = "Products"
sensor_state_sheet = [2,4] # Location at google sheet [row,column]

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
motor.start(0)

while True:

    #sensor_state = sensor.read_pin() #[0]
    sensor_state = GPIO.input(sensor)
    sleep(0.5)

    if sensor_state == 0:
        GPIO.output(dir, GPIO.HIGH)
        motor.ChangeDutyCycle(15)
        GPIO.output(enable, GPIO.LOW)
        sleep(0.5)

    if sensor_state == 1:
        GPIO.output(enable, GPIO.HIGH)
        motor.stop()

