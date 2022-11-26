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

    sensor_state = "1"

    if sensor_state == "1":

        # Zero the platform 
        print("Moving platform to zero\n")
        arduino.send_command("PZ,0")
        arduino.read_command()

        #### Move straight towards conveyor ####
        print("Moving straight towards conveyor\n")
        arduino.send_command("F,1116")
        arduino.read_command()

        # Lower platform to appropiate distance
        print("Lowering platform to conveyor\n")
        arduino.send_command("PD,23")
        arduino.read_command()

        # Grab with electromagnets
        print("Activating electromagnets\n")
        arduino.send_command("EM,1")
        arduino.read_command()

        # Lift up for analysis
        print("Lifting container\n")
        arduino.send_command("PU,10")
        arduino.read_command()

        #### Move forward for camera ####
        print("Moving forward to camera\n")
        arduino.send_command("F,635")
        arduino.read_command()

        # Lower platform to appropiate distance
        print("Lowering platform to camera view\n")
        arduino.send_command("PD,18")
        arduino.read_command()
        
        # Update sheet value to begin camera analysis for qr code
        sheet.update_cell_value(sensor_state_sheet[0], sensor_state_sheet[1], "0")
        sleep(0.5)
        sheet.update_cell_value(analysis_sheet[0], analysis_sheet[1], "1")
        sleep(0.5)

    elif (done_analyzing == "1"):

        #### Move forward to turn ####
        print("Moving forward to turn\n")
        arduino.send_command("F,360")
        sleep(2)
        arduino.read_command()
        sleep(2)

        #### Turn towards conveyors 1 and 2 ####
        print("Rotating to face conveyors\n")
        arduino.send_command("TR,832") # to turn 180 deg
        sleep(2)
        arduino.read_command()
        sleep(2)

        # Moving platform to appropiate distance
        print("Moving platform to conveyor distance\n")
        arduino.send_command("PU,18")
        arduino.read_command()
        sleep(1)

        #### Move left towards conveyor ####
        print("Moving left towards conveyors\n")
        arduino.send_command("M,L#1500")
        sleep(2)
        arduino.read_command()
        sleep(2)
        
        # Read which band it should move to
        row_to_read = len(sheet.get_all_data()) + 1
        sleep(0.5)
        col_to_read = 3
        conveyor = sheet.get_cell_value(row_to_read,col_to_read)
        sleep(0.5)

        if conveyor == "1":

            ### MOVE TO CONVEYOR 1 ###
            print("Putting in conveyor 1\n")

            # Grab with electromagnets
            print("Deactivating electromagnets\n")
            arduino.send_command("EM,0")
            arduino.read_command()
            sleep(1)

        elif conveyor == "2":

            ### MOVE TO CONVEYOR 2 ###
            print("Moving to conveyor 2\n")
            arduino.send_command("M,F#1500")
            sleep(2)
            arduino.read_command()
            sleep(2)

            # Grab with electromagnets
            print("Deactivating electromagnets\n")
            arduino.send_command("EM,0")
            arduino.read_command()
            sleep(1)

        sheet.update_cell_value(done_analyzing_sheet[0], done_analyzing_sheet[1], "0")
        sleep(0.5)

    else:

        print("Waiting...")
        sleep(1)
