import serial

print('Serial script started');

ser = serial.Serial('/dev/ttyACM0', 9600);

while True:
    out = ser.readline().decode("utf-8") 
    out = out.replace('X=', '').replace(', out=', '').replace('wart_star=', '');
    print(out);
