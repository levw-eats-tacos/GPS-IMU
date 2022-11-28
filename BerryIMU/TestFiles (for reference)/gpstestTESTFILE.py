from gps import *
import time
    
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 
print('latitude\tlongitude\ttime utc\t\t\taltitude\tepv\tept\tspeed\tclimb') # '\t' = TAB to try and output the data in columns.
   
try:
 
    while True:
        report = gpsd.next() #
        if report['class'] == 'TPV':
             
            print(str(getattr(report,'lat',0.0))+"\t"+str(getattr(report,'lon',0.0))+"\t"+str(getattr(report,'time',''))+"\t"+str(getattr(report,'alt','nan'))+"\t\t"+str(getattr(report,'epv','nan'))+"\t"+str(getattr(report,'ept','nan'))+"\t"+str(getattr(report,'speed','nan'))+"\t"+ str(getattr(report,'climb','nan')+"\t"))
        time.sleep(1) 
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("Done.\nExiting.")