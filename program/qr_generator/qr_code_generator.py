import qrcode
import random

# Path to save the image to
save_path = "/home/pi/Desktop/LARC/LARC/program/qr_generator/"

# This script will create the random qrcodes to place in the containers for the robot

# Create a random number for the first column of the google sheet, the product ID. 
product_id = str(random.randint(1000,2000))

# Create a list for the different types of products and select a random number to get a random index to call
product_types = ["Bolts", "Nuts", "Bearings", "Allen-keys", "Batteries", "Screws"]
random_product = random.randint(0, len(product_types) - 1)

# The last column indicates the band it should go to, make it random. There are only two bands.
band_number = str(random.randint(1,2))

# Create the qr code with the data
data = qrcode.make(f"{product_id},{product_types[random_product]},{band_number}")

# Save the code to an image file in the specified location
data.save(f"{save_path}generated_qr_code.png")
