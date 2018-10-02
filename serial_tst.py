import serial

print('Serial script started');

ser = serial.Serial('/dev/ttyACM0', 9600);

while True:
    print(ser.readline());
