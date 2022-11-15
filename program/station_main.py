import RPi.GPIO as GPIO
from google_sheet_comm.sheet_comm import Sheet_Comm
from qr_scanner.qr_code_scanner import qr_code_scanner
from time import sleep

sheet = "Inventory"
work_sheet = "Products"
sensor_state_sheet = [2,4] # Location at google sheet [row,column]
incoming_sheet = [2,5]
analysis_sheet = [2,6]
product_id = []

# Sheet communication
sheet = Sheet_Comm(sheet, work_sheet)

# Qr code scanner directory
qr = qr_code_scanner("/home/pi/Desktop/LARC/LARC/program/qr_scanner/qr_code.png")

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
    sleep(1)
    analysis = sheet.get_cell_value(analysis_sheet[0], analysis_sheet[1])
    sleep(1)

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

    if analysis == "1":

        print("Analyzing picture...\n")
        qr.take_picture()
        extracted_qr = qr.read_qr_code()

        if extracted_qr != "":
            data = extracted_qr.split(",")
            row_to_insert = len(sheet.get_all_data()) + 1
            sleep(0.5)
            sheet.insert_single_row(data,row_to_insert)
            sleep(0.5)

        else:
            print("ERROR: Failed to analyze\n")

        sheet.update_cell_value(analysis_sheet[0], analysis_sheet[1], "0")
        sleep(0.5)
