import serial

s = serial.Serial("COM8",2000000)

s.flushInput()
s.flushOutput()

while True:
    bytesToRead = s.inWaiting()
    s.read(bytesToRead)