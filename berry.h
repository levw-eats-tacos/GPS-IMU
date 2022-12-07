#ifndef BERRY_H
#define BERRY_H

#include "device.h"
#include <windows.h>

/*
    The Device class is an interface used for the GPS/IMU
    devices that will be tested. If any device needs to be
    tested, one only needs to make a subclass of this and
    inherit the
*/
class Berry:public Device{
public slots:

    // function for updating the ypr instance variable
    virtual void updateYPR(){}

    // function for updating the gps instance variable
    virtual void updateGPS(){}
public:
    // constructor
    Berry():Device(){}

    // function for connecting to the device
    virtual void connect(){}

    // function for disconnecting the device
    virtual void disconnect(){}
    
private:
    HANDLE m_serialHandle;


};

#endif // BERRY_H