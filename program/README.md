# Program
The arduino program for the main robot is contained in the *robot_code* folder.

There are 3 main classes that handle specific tasks for the combined program:

- The *communication* folder contains the class that hanldes the serial communication between the Raspberry and the Arduino Mega.

- The *google_sheet_comm* folder contains the class that handles the google cloud service with the spread sheet. For further reference visit the gspread library documentation.

- The *qr_scanner* folder contains the class that takes the picture of the qr code in the container and uses the cv2 library to analize it and get its data.

The three classes are combined and used in the main python files for the conveyor and the robot.

The *station_main.py* file, should be running in the main conveyor.

The *robot_main.py* file, should be running in the main robot.

Run the programs by entering into VS Code and connecting to both Raspberries through SSH protocol in different windows.

That way, both programs will be able to run simultaneously.

All other folders in the program are either tests for specific funtions that were integrated or not into the main files.

The *requirements.txt* file contains some of the programs nad libraries needed to run the files, as well as their version.
