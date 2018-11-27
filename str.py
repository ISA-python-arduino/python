import struct
import serial

#ser = serial.Serial(port='/dev/ttyACM0', baudrate=57600, timeout=0.05)
ser = serial.Serial('COM4', 57600, timeout=0.05)
#bajty = bytearray(1, 'ascii')
num = int(100)
ser.write(struct.pack('>ii', num, int(102)))