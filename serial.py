import serial

print('Serial script started');

ser = serial.Serial('/dev/tty.usbserial', 9600);

print(ser.readline());
