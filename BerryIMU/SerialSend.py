#/dev/Serial

import serial
import json

def main():
    ser = serial.Serial('/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTBHb116L16-if00-port0',9600)
    while True:
        if(ser.in_waiting > 0):
            data = 'this is a really long scentence plox'
            jason = json.dumps(data)
            ser.write(jason.encode('utf-8'))
            ser.reset_input_buffer()


main()