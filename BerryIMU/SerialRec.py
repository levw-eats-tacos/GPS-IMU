import serial
import time

s = serial.Serial("COM8")

s.flushInput()
s.flushOutput()

s.write('1'.encode('utf-8'))

while True:
    bytesToRead = s.inWaiting()
    print(bytesToRead)
    if(bytesToRead > 40):
        help = s.read(bytesToRead)
        decoded = help.decode('utf-8', 'ignore')
        print(decoded)
        s.write('1'.encode('utf-8'))
        time.sleep(1)