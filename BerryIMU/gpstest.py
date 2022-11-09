from gps import *
import time
    
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 
print('latitude\tlongitude\ttime utc\t\t\taltitude\tepv\tept\tspeed\tclimb') # '\t' = TAB to try and output the data in columns.
   
try:
 
    while True:
        report = gpsd.next() #
        if report['class'] == 'TPV':
             
            print(getattr(report,'lat',0.0)+"\t"+getattr(report,'lon',0.0)+"\t"+getattr(report,'time','')+"\t"+getattr(report,'alt','nan')+"\t\t"+getattr(report,'epv','nan')+"\t"+getattr(report,'ept','nan')+"\t"+getattr(report,'speed','nan')+"\t"+ getattr(report,'climb','nan')+"\t")
        time.sleep(1) 
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("Done.\nExiting.")