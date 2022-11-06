import cv2
from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.capture("/home/pi/Desktop/LARC/LARC/program/qr_scanner/qr_code.png")

#qr = read_qr_code()

#print(qr)

def read_qr_code(filename):
    """Read an image and read the QR code.
    
    Args:
        filename (string): Path to file
    
    Returns:
        qr (string): Value from QR code
    """
    
    try:
        img = cv2.imread(filename)
        detect = cv2.QRCodeDetector()
        value, points, straight_qrcode = detect.detectAndDecode(img)
        return value
    except:
        return "Error"