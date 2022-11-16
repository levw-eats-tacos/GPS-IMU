import serial
import time

s = serial.Serial("COM8")

s.flushInput()
s.flushOutput()
x=0
s.write('1'.encode('utf-8'))

while True:
    bytesToRead = s.inWaiting()
    print(bytesToRead)
    x=x+1
    if(x>1):
        s.write('1'.encode('utf-8'))
    if(bytesToRead > 0):
        x = 0
    if(bytesToRead > 26):
        help = s.read(bytesToRead)
        decoded = help.decode('utf-8', 'ignore')
        print(decoded)
        s.write('1'.encode('utf-8'))
        time.sleep(1)
        x= 0