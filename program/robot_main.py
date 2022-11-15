from communication.rpi_comm import Rpi_Comm
from google_sheet_comm.sheet_comm import Sheet_Comm
from time import sleep

sheet = "Inventory"
work_sheet = "Products"
sensor_state_sheet = [2,4]
analysis_sheet = [2,6]

# Sheet communication
sheet = Sheet_Comm(sheet, work_sheet)

# Arduino serial communication
arduino = Rpi_Comm("/dev/ttyUSB0", 9600, 1)

while True:

    sensor_state = sheet.get_cell_value(sensor_state_sheet[0], sensor_state_sheet[1])
    sleep(1)

    if sensor_state == "1":

        #### Move towards conveyor ####
        print("Moving towards conveyor\n")
        sleep(2)

        # Zero the platform 
        print("Moving conveyor to zero\n")
        arduino.send_command("PZ,0")
        arduino.read_command()
        sleep(1)

        # Lower conveyor to appropiate distance
        print("Moving conveyor to container\n")
        arduino.send_command("PD,13")
        arduino.read_command()
        sleep(1)

        # Grab with electromagnets
        print("Activating electromagnets\n")
        arduino.send_command("EM,1")
        arduino.read_command()
        sleep(1)

        # Lift up for analysis
        print("Lifting container\n")
        arduino.send_command("PZ,0")
        arduino.read_command()
        sleep(1)

        #### Move forward for camera ####
        print("Moving to camera\n")
        sleep(2)

        # Update sheet value to begin camera analysis for qr code
        sheet.update_cell_value(sensor_state_sheet[0], sensor_state_sheet[1], "0")
        sleep(0.5)
        sheet.update_cell_value(analysis_sheet[0], analysis_sheet[1], "1")
        sleep(0.5)

    else:

        print("Waiting...")
        sleep(1)
