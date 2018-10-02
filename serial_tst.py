import serial

print('Serial script started');

ser = serial.Serial('/dev/ttyACM0', 9600);

while True:
    out = ser.readline()
    out = (out.replace('b X=\t').replace(' , out=').replace(' wart_star=').replace('\n'));
    print(out);
