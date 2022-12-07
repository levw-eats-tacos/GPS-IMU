#ifndef DEVICE_H
#define DEVICE_H

#include <QObject>

/*
    The Device class is an interface used for the GPS/IMU
    devices that will be tested. If any device needs to be
    tested, one only needs to make a subclass of this and
    inherit the
*/
class Device:public QObject{
public slots:

    // function for updating the ypr instance variable
    virtual void updateYPR();

    // function for updating the gps instance variable
    virtual void updateGPS();
public:
    // constructor
    Device():QObject();

    // function for connecting to the device
    virtual void connect();

    // function for disconnecting the device
    virtual void disconnect();

    // getter function for ypr
    virtual float *getArr(){
        return arr;
    }


protected:
    // stores the current yaw, pitch, and roll of the device
    float arr[3];

};

#endif // DEVICE_H