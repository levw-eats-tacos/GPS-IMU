#/dev/Serial

import serial

class SerialWrapper:
    def __init__(self, device):
        self.ser = serial.Serial(device, 115200)

    def sendData(self, data):
        data += "\r\n"
        self.ser.write(data.encode())

def main():
    ser = SerialWrapper('/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTBHb116L16-if00-port0')
    while True:
        data = "hi"
        ser.sendData(data)


main()