from communication.rpi_comm import Rpi_Comm
from google_sheet_comm.sheet_comm import Sheet_Comm
from time import sleep

sheet = "Inventory"
work_sheet = "Products"
sensor_state_sheet = [2,4]

# Sheet communication
sheet = Sheet_Comm(sheet, work_sheet)

# Arduino serial communication
arduino = Rpi_Comm("/dev/ttyUSB0", 9600, 1)

arduino.send_command("PD,4")
sleep(1)
