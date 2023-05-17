import serial
from time import process_time

# Defining the Arduino port
<<<<<<< HEAD
<<<<<<< HEAD
#arduino_port = "COM4"
arduino_port = "/dev/cu.wchusbserial54750076121"
=======
arduino_port = "/dev/cu.usbmodem14201"
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
=======
arduino_port = "/dev/cu.wchusbserial54750076121"
>>>>>>> 564e173 (Massive changes to code, and perhaps a new success???)

# Baud rate
baud = 115200

# File name for the .csv file
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
file_name = "python_scripts/test_bias12.csv"
=======
file_name = "python_scripts/test1.csv"
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)
=======
file_name = "python_scripts/test3.csv"
>>>>>>> 564e173 (Massive changes to code, and perhaps a new success???)
=======
file_name = "python_scripts/Estimate_R.csv"
>>>>>>> 63c81e0 (estimate of sensor meassurment)
=======
file_name = "python_scripts/Estimate_R_series.csv"
>>>>>>> d0ba3a8 (estimate R changes)

# Start the serial port
ser = serial.Serial(port = arduino_port, baudrate = baud, timeout = 0.0005)
print("Connected to Arduino port: " + arduino_port)

# Open the csv file
file = open(file_name, "a")
print("Created file: " + file_name)

# Sensor data string
sensor_data_str = ""
sensor_data = []

# Start the process
start_time = process_time()
print("Data measurement starts now!")

<<<<<<< HEAD
# Take measurements for 3 minutes
while (process_time() - start_time) < 180.0:
=======
# Take measurements for 4 minutes
<<<<<<< HEAD
while (process_time() - start_time) < 240.0:
>>>>>>> 564e173 (Massive changes to code, and perhaps a new success???)
=======
while (process_time() - start_time) < 180.0:
>>>>>>> 63c81e0 (estimate of sensor meassurment)
    if(ser.inWaiting() > 0):
        sensor_data_str += ser.read(ser.inWaiting()).decode('utf-8')

print("Measurement complete")

with open(file_name, 'w', encoding='UTF8') as f:
    f.write("top, topMid, botMid, bot\n")
    f.write(sensor_data_str.replace(";", "\n"))

print("CSV file complete")
file.close()