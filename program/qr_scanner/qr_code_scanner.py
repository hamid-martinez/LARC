import cv2
from picamera import PiCamera

# Suggested location for qr_code: "/home/pi/Desktop/LARC/LARC/program/qr_scanner/qr_code.png"

class qr_code_scanner():

    def __init__(self, file_location):
        
        self.file_location = file_location

    def take_picture(self):

        camera = PiCamera()
        camera.capture(self.file_location)

        return f"Picture taken at {self.file_location}"

    def read_qr_code(self):

        try:
            img = cv2.imread(self.file_location)
            detect = cv2.QRCodeDetector()
            value, points, straight_qrcode = detect.detectAndDecode(img)

            return value

        except:
            return "ERROR: No QR-code detected"
