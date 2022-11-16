import socket
#pip install 
from gps import *
import time
import time
#pip install
import serial
import json

HOST = "127.0.0.1"
PORT = 65432

def serialOut(data):
	ser = serial.Serial('/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTBHb116L16-if00-port0',9600)
	if(ser.in_waiting > 0):
		jason = json.dumps(data)
		ser.write(jason.encode('utf-8'))
		ser.reset_input_buffer()

ser = serial.Serial(
        port='/dev/ttyS0', #check to see what we are outputting maybe /dev/USB0?
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
        
def socketRec():
	"""Talks to the server and attempts to recieve a message. It then prints what it recieves.

	Returns:
		Bytes: Recieved data 
	"""
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.connect((HOST,PORT))
		s.sendall(b"Hello,world")
		data = s.recv(1024)
	#print(f"Received {data!r}")
	return data


#starts GPS stuff
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 
print('latitude\tlongitude\ttime utc\t\t\taltitude\tepv\tept\tspeed\tclimb') # '\t' = TAB to try and output the data in columns.

#loop through the GPS and Socket. First through Socket. Then through GPS
try:
 
	while True:
		#Socket data
		data = socketRec()
		report = gpsd.next() #
		r = data.decode()
		serialOut(r)
		print(r)
		#GPS Data
		if report['class'] == 'TPV':
             
			print(getattr(report,'lat',0.0),"\t"),
			print(getattr(report,'lon',0.0),"\t"),
			print(getattr(report,'time',''),"\t"),
			print(getattr(report,'alt','nan'),"\t\t"),
			print(getattr(report,'epv','nan'),"\t"),
			print(getattr(report,'ept','nan'),"\t"),
			print(getattr(report,'speed','nan'),"\t"),
			print(getattr(report,'climb','nan'),"\t")
		#ser.write("test test") replace with JSON when ready.
		time.sleep(1)
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("Done.\nExiting.")