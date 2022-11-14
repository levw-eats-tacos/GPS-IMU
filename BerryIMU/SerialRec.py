import serial

s = serial.Serial("COM8")

s.flushInput()
s.flushOutput()

while True:
    bytesToRead = s.inWaiting()
    help = s.read(bytesToRead)
    decoded = help.decode()
    print(decoded)