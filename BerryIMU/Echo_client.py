import socket
#pip install 
from gps import *
import time
#pip install
import serial
import json

HOST = "127.0.0.1"
PORT = 65432

ser = serial.Serial('/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CTBHb116L16-if00-port0',9600)

def serialOut(data):
	global ser
	if(ser.in_waiting > 0):
		jason = json.dumps(data)
		ser.write(jason.encode('utf-8'))
		ser.reset_input_buffer()
	bytesToRead = ser.inWaiting()
	print("Bytes in inbox:" +  str(bytesToRead))
        
def socketRec():
	"""Talks to the server and attempts to recieve a message. It then prints what it recieves.

	Returns:
		Bytes: Recieved data 
	"""
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.connect((HOST,PORT))
		s.sendall(b"Hello,world")
		data = s.recv(1024)
		s.close()
	#print(f"Received {data!r}")
	return data


#starts GPS stuff
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 
print('latitude\tlongitude\ttime utc\t\t\taltitude\tepv\tept\tspeed\tclimb') # '\t' = TAB to try and output the data in columns.

#loop through the GPS and Socket. First through Socket. Then through GPS
try:
 
	while True:
		#Socket data (IMU)
		data = socketRec()
		report = gpsd.next() #
		r = data.decode()
		loads = json.loads(r)
		print(loads)
		
		print(r)
		#GPS Data
		if report['class'] == 'TPV':
			jason = {"kalmanx":loads['kalmanx'],
					"kalmany":loads['kalmany'],
					"kalmanz":loads['kalmanz'],
					"gyroX":loads['gyroX'],
					"gyroY":loads['gyroY'],
					"gyroZ":loads['gyroZ'],
					"lat":getattr(report,'lat',0.0),
					"lon":getattr(report,'lon',0.0),
					"alt":getattr(report,'alt','nan'),
					"time":getattr(report,'time','')}
			serialOut(jason)
		else:
			serialOut(r)
		time.sleep(1)
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("Done.\nExiting.")