import serial

ser = serial.Serial('/dev/ttyUSB0', 115200) #function used to create a serial port object for communication a device


to_be_printed_ = b'THE DATA IS NOW TRANSMITTED'
ser.write(to_be_printed_)

ser.close()

