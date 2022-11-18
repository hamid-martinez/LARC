from communication.rpi_comm import Rpi_Comm
from google_sheet_comm.sheet_comm import Sheet_Comm
from time import sleep

sheet = "Inventory"
work_sheet = "Products"
sensor_state_sheet = [2,4]
analysis_sheet = [2,6]
done_analyzing_sheet = [2,7]

# Sheet communication
sheet = Sheet_Comm(sheet, work_sheet)

# Arduino serial communication
arduino = Rpi_Comm("/dev/ttyUSB0", 9600, 1)

while True:

    sensor_state = sheet.get_cell_value(sensor_state_sheet[0], sensor_state_sheet[1])
    sleep(1)
    done_analyzing = sheet.get_cell_value(done_analyzing_sheet[0], done_analyzing_sheet[1])
    sleep(1)

    if sensor_state == "1":

        # Zero the platform 
        print("Moving conveyor to zero\n")
        arduino.send_command("PZ,0")
        arduino.read_command()
        sleep(2)

        #### Move straight towards conveyor ####
        print("Moving straight towards conveyor\n")
        arduino.send_command("M,F#1300")
        sleep(2)
        arduino.read_command()
        sleep(2)

        #### Move left towards conveyor ####
        print("Moving left towards conveyor\n")
        arduino.send_command("M,L#500")
        sleep(2)
        arduino.read_command()
        sleep(2)

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
        arduino.send_command("PU,7")
        arduino.read_command()
        sleep(1)

        #### Move forward for camera ####
        print("Moving right to camera\n")
        arduino.send_command("M,R#700")
        sleep(2)
        arduino.read_command()
        sleep(2)

        print("Moving forward to camera\n")
        arduino.send_command("M,F#200")
        sleep(2)
        arduino.read_command()
        sleep(2)

        # Lower conveyor to appropiate distance for camera
        print("Moving conveyor to container\n")
        arduino.send_command("PD,13")
        arduino.read_command()
        sleep(1)

        # Update sheet value to begin camera analysis for qr code
        sheet.update_cell_value(sensor_state_sheet[0], sensor_state_sheet[1], "0")
        sleep(0.5)
        sheet.update_cell_value(analysis_sheet[0], analysis_sheet[1], "1")
        sleep(0.5)

    elif (done_analyzing == "1"):
        
        # Read which band it should move to
        row_to_insert = len(sheet.get_all_data()) + 2
        band = sheet.get_cell_value(sensor_state_sheet[0], sensor_state_sheet[1])
        sleep(1)

        ### Move to corresponding conveyor ###

        sheet.update_cell_value(done_analyzing_sheet[0], done_analyzing_sheet[1], "0")
        sleep(0.5)


    else:

        print("Waiting...")
        sleep(1)
