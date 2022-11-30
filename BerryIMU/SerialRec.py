import serial
import time

#check com port before running
s = serial.Serial("COM5")

s.flushInput()
s.flushOutput()
x=0
s.write('1'.encode('utf-8'))

def buffercheck():
    global s
    stringOfStuff = ""
    while(True):
        nextcharByte = s.read(1)
        nextchar = nextcharByte.decode('utf-8', 'ignore')
        if(nextchar == "}"):
            stringOfStuff = stringOfStuff + nextchar
            return stringOfStuff
        else:
            stringOfStuff = stringOfStuff + nextchar


while True:
    bytesToRead = s.inWaiting()
    x=x+1
    if(x>1):
        s.write('1'.encode('utf-8'))
    if(bytesToRead > 0):
        x = 0
    #format decimal places so we dont have a caniption
    #if(bytesToRead > 154):
        #help = s.read(bytesToRead)
        #decoded = help.decode('utf-8', 'ignore')
        #print(decoded)
        #s.write('1'.encode('utf-8'))
        #time.sleep(2)
    if(bytesToRead >10):
        output = buffercheck()
        print(output)
        s.write('1'.encode('utf-8'))
        time.sleep(.25)