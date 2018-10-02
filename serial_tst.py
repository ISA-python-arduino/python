import serial
import time
import re

print('Serial script started');

ser = serial.Serial('/dev/ttyACM0', 9600);

while True:
    ser.writeline('echo');
    out = ser.readline().decode("utf-8")
    #num = re.findall(r"[-+]?\d*\.\d+|\d+", out);
    print('Arduino ' + out);
