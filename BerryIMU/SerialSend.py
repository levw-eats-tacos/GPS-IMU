#/dev/Serial

import serial

class SerialWrapper:
    def __init__(self, device):
        self.ser = serial.Serial(device, 9600)

    def sendData(self, data):
        self.ser.write(data.encode('utf-8'))

def main():
    ser = serial.Serial('/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTBHb116L16-if00-port0',9600)
    while True:
        if(ser.in_waiting > 0):
            data = 'this is a really long scentence plox'
            ser.write(data.encode('utf-8'))
            ser.reset_input_buffer()


main()