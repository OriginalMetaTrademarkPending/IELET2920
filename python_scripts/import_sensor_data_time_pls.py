import serial
from time import process_time

# Defining the Arduino port
<<<<<<< HEAD
#arduino_port = "COM4"
arduino_port = "/dev/cu.wchusbserial54750076121"
=======
arduino_port = "/dev/cu.usbmodem14201"
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)

# Baud rate
baud = 9600

# File name for the .csv file
<<<<<<< HEAD
file_name = "python_scripts/test_bias12.csv"
=======
file_name = "python_scripts/test1.csv"
>>>>>>> 3c45509 (Opened a new branch for parameter estimation. Purpose of this branch is to fix the estimation algorithm)

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

# Take measurements for 3 minutes
while (process_time() - start_time) < 180.0:
    if(ser.inWaiting() > 0):
        sensor_data_str += ser.read(ser.inWaiting()).decode('utf-8')

print("Measurement complete")

with open(file_name, 'w', encoding='UTF8') as f:
    f.write("top, topMid, botMid, bot\n")
    f.write(sensor_data_str.replace(";", "\n"))

print("CSV file complete")
file.close()