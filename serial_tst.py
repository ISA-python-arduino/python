import serial

print('Serial script started');

ser = serial.Serial('/dev/ttyACM0', 9600);

print(ser.readline());
