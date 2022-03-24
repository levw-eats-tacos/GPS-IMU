#include "device_server.h"
#include <iostream>
#include <sstream>
#include <QDateTime>
#include <chrono>
#include <zmq.hpp>
//#include "ASL.pb.h"
//#include <thread>
#include <ctime>


/***********************************************************************************
 * device Class Member Functions
 */

//Default constructor for empty device
device::device(){
    device_name = "";
    device_manufacturer = "";
    device_model = "";
    device_type = no_device_type;
    device_subtype = no_device_subtype;
    device_initialized = false;
    device_detected = false;
    device_busy = false;
}



void device::Detect_Device(){
    if(console_output) cout << "Base device class cannot detect device." << endl;
    device_detected = false;
}

void device::Initialize_Device(){
    if(console_output) cout << "Base device class cannot be initialized." << endl;
    device_initialized = false;
}

string device::Get_Timestamp(bool filename_safe){

    QDateTime ts = QDateTime::currentDateTime();
    int year = ts.date().year();
    int month = ts.date().month();
    int day = ts.date().day();
    int hour = ts.time().hour();
    int min = ts.time().minute();
    int sec = ts.time().second();

    stringstream ss;
    if(filename_safe){
        ss << year << "-";
        if(month<10) ss << "0" << month << "-";
        else ss << month << "-";
        if(day<10) ss << "0" << day << "-";
        else ss << day << "_";
        if(hour<10) ss << "0" << hour << "-";
        else ss << hour << "-";
        if(min<10) ss << "0" << min << "-";
        else ss << min << "-";
        if(sec<10) ss << "0" << sec;
        else ss << sec;
    }
    else{
        ss << year << "-";
        if(month<10) ss << "0" << month << "-";
        else ss << month << "-";
        if(day<10) ss << "0" << day << " ";
        else ss << day << " ";
        if(hour<10) ss << "0" << hour << ":";
        else ss << hour << ":";
        if(min<10) ss << "0" << min << ":";
        else ss << min << ":";
        if(sec<10) ss << "0" << sec;
        else ss << sec;
    }
    return ss.str();
}

//Returns the Device Type as its String Enumeration Name
string device::Get_Device_Type_As_String(){
    if(device_type == camera)
        return "camera";
    else if(device_type == motion_control_stage)
        return "motion_control_stage";
    else if(device_type == calibration_source)
        return "calibration_source";
    else return "no_device_type";
}

//Returns the Device Type as its String Enumeration Name
string device::Get_Device_Subtype_As_String(){
    if(device_subtype == visible_mono)
        return "visible_mono";
    else if(device_subtype == visible_color)
        return "visible_color";
    else if(device_subtype == visible_mono_dot_polarimeter)
        return "visible_mono_dot_polarimeter";
    else if(device_subtype == visible_mono_microgrid_polarimeter)
        return "visible_mono_microgrid_polarimeter";
    else if(device_subtype == visible_color_microgrid_polarimeter)
        return "visible_color_microgrid_polarimeter";
    else if(device_subtype == rotary_stage)
        return "rotary_stage";
    else if(device_subtype == translation_stage)
        return "translation_stage";
    else if(device_subtype == robotic_arm)
        return "robotic_arm";
    else if(device_subtype == integrating_sphere)
        return "integrating_sphere";
    else if(device_subtype == blackbody)
        return "blackbody";
    else return "no_device_subtype";
}


//This function handles basic requests and composes a response for TCP functionality
int device::Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::Response &resp){
    cout << "Invalid Request Received for Empty Device. No action taken." << endl;
    resp.set_response(ASLpb::Response::INVALID_REQUEST);
    resp.clear_double_param();
    resp.clear_int_param();
    resp.clear_string_param();
    resp.clear_bool_param();
    resp.set_status("INVALID_REQUEST");
    cout << "Device Response: INVALID_REQUEST for " << dev_ptr->Get_Device_Name() << endl;
    return -1;
}


//This function handles image requests and composes an image data response for TCP functionality
int device::Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::ImageResponse &imgresp){
    cout << "Invalid Request Received for Empty Device. No action taken." << endl;
    imgresp.set_camera(ASLpb::ImageResponse::NON_CAMERA_DEVICE);
    imgresp.set_status("INVALID_REQUEST");
    cout << "Device Response: INVALID_REQUEST for device " << dev_ptr->Get_Device_Name() << endl;
    return -1;
}



/***********************************************************************************
 * zaberRotationStage Class Member Functions
 */

//Constructor
zaberRotationStage::zaberRotationStage(){


    //Set default device parameters
    device_name = "Rotary Stage";
    device_manufacturer = "Zaber";
    device_model = "X-RST120AK-E04";
    device_type = motion_control_stage;
    device_subtype = rotary_stage;
    serial_port_name = "";
}

//Set's the serial port name. Also sets the detection and initialized flags to false
void zaberRotationStage::Set_Serial_Port(string serialPort){
    serial_port_name = serialPort;
}

//Thread that will make multiple attempt to connect to a device
void zaberRotationStage::Detect_Device(){

    cout << "============================================" << endl;
    cout << "| Attempting to Detect Zaber Rotary Stage: |" << endl;
    cout << "============================================" << endl;

    //Attempt to update zaber device database (connect to internet)
    cout << " ->Attempting to update Zaber device database..." << std::endl;
    Library::enableDeviceDbStore();

    //Attempt to detect and initialize device. Will make multiple attempts
    for(unsigned int k=0; k<number_of_detection_tries; k++){
        if((!Is_Device_Detected()) && (!Is_Device_Initialized())){

            //Find Zaber serial connection
            bool found = Detect_Zaber_Serial_Port();

            if(found){
                //Attempt to connect to the device through the detected serial port
                cout << " ->Attempting connection to Zaber device(s) on serial port " << serial_port_name << ". Attempt " << k+1 << "of " << number_of_detection_tries << "." << endl;
                connection = Connection::openSerialPort(serial_port_name);

                //Look for devices connected to the specified COM port
                deviceList = connection.detectDevices();
                if(deviceList.size()<1){
                    cout << " ->No Zaber devices found. Trying again." << endl;
//                    sleep_until(system_clock::now() + 1s); //Timeout for 1 second
                    continue;
                }
                else{
                    cout << "   ->Found " << deviceList.size() << " Zaber device(s)." << endl << endl;

                    //Set device and device axes to point to local class variables
                    rotation_stage_device = new Device(deviceList[0]);
                    rotation_stage_axis = new Axis(rotation_stage_device->getAxis(1));
                    device_detected = true;

                    //Set up base device parameters
                    device_model = rotation_stage_device->getName();
                    cout << "Device Information:" << endl;
                    cout << "------------------" << endl;
                    cout << " ->Device Name: " << device_model << endl;
                    cout << " ->Device Address: " << rotation_stage_device->getDeviceAddress() << endl;
                    cout << " ->Device ID: " << rotation_stage_device->getDeviceId() << endl;
                    cout << " ->Device Serial Number: " << rotation_stage_device->getSerialNumber() << endl;
                    cout << " ->" << rotation_stage_device->getFirmwareVersion().toString() << endl;
                    cout << " ->Number of Axes: " << rotation_stage_device->getAxisCount() << endl;
                    cout << "Device detection successful." << endl << endl;

                    cout << "Attempting to initialize device:" << endl;
                    cout << "-------------------------------" << endl;
                    Initialize_Device();
                    if(Is_Device_Initialized()){
                        if(console_output) cout << "Device initialization successful." << endl << endl;
                        break;
                    }
                    else if(console_output) cout << "Device initialization failed." << endl << endl;
                }

            }
        }
        // If device is detected and not initialized, keep trying to initialize the
        // device until successful.
        else if((Is_Device_Detected()) && (!Is_Device_Initialized())){
            // Re-attempt to initialize device
            cout << "Re-attempting to initialize device..." << endl << endl;
            Initialize_Device();
            if(Is_Device_Initialized()){
                cout << "Device initialization successful." << endl << endl;
                break;
            }
            else cout << "Device initialization failed." << endl;
        }
        // If somehow the device is not detected but somehow is initialized set
        // the initialization flag to false.
        else if((!Is_Device_Detected()) && (Is_Device_Initialized())){
            device_initialized = false;
        }
        // Otherwise, all is well when the device is both detected and initialized.
        else{
            break;
        }
    }

    //If number of attempts exhausted and device not detected...
    if(!Is_Device_Detected() & !Is_Device_Initialized()){
        cout << "Device detection timed out. Device not detected or initialized." << endl << endl;
    }

}

//Will search any connected serial port connections and identify my specific zaber rotary device
bool zaberRotationStage::Detect_Zaber_Serial_Port(){

// Zaber Serial Port Connection Information:
// Port: COM9
// Location: \\.\COM9
// Description: USB Serial Port
// Manufacturer: FTDI
// Serial number: A10K5TSMA
// Note: On Ubuntu the Serial number is: A10K5TSM
// Vendor Identifier: 403
// Product Identifier: 6001

    //First, we check for all serial port devices that are available.
    //Then, we determine if the zaber device is one of them.

    cout << " ->Scanning all serial ports for Zaber device..." << endl;

    const auto serialPortInfos = QSerialPortInfo::availablePorts();

//        cout << "Total number of ports available: " << serialPortInfos.count() << endl;

    const QString blankString = "N/A";
    QString portname;
    QString portlocation;
    QString description;
    QString manufacturer;
    QString serialnumber;
    QString vendor;
    QString identifier;

    for (const QSerialPortInfo &serialPortInfo : serialPortInfos) {
        portname = serialPortInfo.portName();
        portlocation = serialPortInfo.systemLocation();
        description = serialPortInfo.description();
        manufacturer = serialPortInfo.manufacturer();
        serialnumber = serialPortInfo.serialNumber();
        vendor = (serialPortInfo.hasVendorIdentifier() ? QByteArray::number(serialPortInfo.vendorIdentifier(), 16) : blankString);
        identifier = (serialPortInfo.hasProductIdentifier() ? QByteArray::number(serialPortInfo.productIdentifier(), 16) : blankString);

        serial_port_name = portlocation.toStdString();

        //Output Information to console
        cout << "Detected Serial Port Info:" << endl;
        QTextStream out(stdout);
        out << endl
            << "Port: " << portname << endl
            << "Location: " << portlocation << endl
            << "Description: " << description << endl
            << "Manufacturer: " << manufacturer << endl
            << "Serial number: " << serialnumber << endl
            << "Vendor Identifier: " << vendor << endl
            << "Product Identifier: " << identifier << endl
            << "Busy: " << (serialPortInfo.isBusy() ? "Yes" : "No") << endl;        

        //Determine if device is the particular zaber turntable device
        //If found, set the serial port name and return success.
        //The zaber library will then establish the connection to the device.
        //QSerialPort serialport;
        if((serialnumber == "A10K5TSM") && (identifier=="6001")){
            out << "   ->Zaber rotary device found on port " << portlocation << "." << endl;
            Set_Serial_Port(portlocation.toStdString());
            return true;
        }
    }
    cout << " ->Zaber device not detected." << endl;
//    sleep_until(system_clock::now() + 1s); //Timeout for 1 second
    return false;
}



//Performs device initialization. Currently just moves the device to home position.
void zaberRotationStage::Initialize_Device(){

    if(Is_Device_Detected()){
        if(console_output) cout << " ->Homing device." << endl;
        try {
            rotation_stage_axis->home();
            device_initialized = true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
        }
    }
    else{
        if(console_output) cout << " ->Homing failed. Device not detected." << endl;
    }
}

//Will move device back to home position
//Device must be detected, initialized, and not busy
bool zaberRotationStage::Home_Device(){
    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        if(console_output) cout << " ->Homing device." << endl;
        try {
            rotation_stage_axis->home();
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << " ->Homing failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Issues a command to stop any device movement
//Device must be detected and initialized
bool zaberRotationStage::Stop_Device(){
    if(Is_Device_Detected() & Is_Device_Initialized()){
        if(console_output) cout << "-->Stopping device... " << endl;
        try {
            rotation_stage_axis->stop();
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Stop failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Will start rotatation of the stage to a specified absolute angle
//Device must be detected, initialized, and idle
bool zaberRotationStage::Set_Angular_Position(double degrees){
    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        try {
            if(console_output) cout << "-->Moving to " << degrees << " degrees (Absolute)." << endl;
            rotation_stage_axis->moveAbsolute(degrees, Units::ANGLE_DEGREES,false);
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Device rotation (absolute) failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Will start rotation of the stage by a specified relative angular amount
//Device must be detected, initialized, and idle
bool zaberRotationStage::Set_Relative_Angular_Position(double degrees){
    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        try {
            if(console_output) cout << "-->Moving " << degrees << " degrees (Relative)." << endl;
            rotation_stage_axis->moveRelative(degrees, Units::ANGLE_DEGREES,false);
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Device rotation (relative) failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Start rotation at contant angular velocity.
//Device must be detected, initialized, and idle
bool zaberRotationStage::Rotate_At_Constant_Velocity(double velocity){
    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        try {

            //If velocity is greater than maximum velocity possible, set to max
            if(velocity > Get_Maximum_Angular_Velocity()) velocity = Get_Maximum_Angular_Velocity();

            if(console_output) cout << "-->Rotating at a constant velocity of " << velocity << "[deg/s]. " << endl;
            rotation_stage_axis->moveVelocity(velocity, Units::ANGULAR_VELOCITY_DEGREES_PER_SECOND);
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Device rotation at constant velocity failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Checks whether device is idle and ready for next command
bool zaberRotationStage::Is_Device_Busy(){
    if(Is_Device_Detected() & Is_Device_Initialized()){
        return rotation_stage_axis->isBusy();
    }
    else{
        if(console_output) cout << "Device busy query failed. Device not detected and initialized." << endl;
        return true;
    }
}

//Returns the current position of rotation stage in degrees
double zaberRotationStage::Get_Angular_Position(){
    if(Is_Device_Detected() & Is_Device_Initialized()){
        double pos = rotation_stage_axis->getPosition(Units::ANGLE_DEGREES);
      //  if(console_output) cout << "-->Current position at " << pos << " degrees." << endl;
        return pos;
    }
    else{
        if(console_output) cout << "Device position query failed. Device not detected and initialized." << endl;
        return -1;
    }
}

//Returns the maximum angular velocity
double zaberRotationStage::Get_Maximum_Angular_Velocity(){

    double speed = rotation_stage_axis->getSettings().get("maxspeed", Units::ANGULAR_VELOCITY_DEGREES_PER_SECOND);
    if(console_output) cout << "-->Maximum speed [deg/s]: " << speed << std::endl;

    if(Is_Device_Detected() & Is_Device_Initialized()){
        try {

            double temperature = rotation_stage_axis->getSettings().get("driver.temperature");
            if(console_output) cout << "Driver temperature [°C]: " << temperature << std::endl;

            double speed = rotation_stage_axis->getSettings().get("maxspeed", Units::ANGULAR_VELOCITY_DEGREES_PER_SECOND);
            if(console_output) cout << "-->Maximum speed [deg/s]: " << speed << std::endl;

            return speed;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return 0;
        } catch (...) {
            if(console_output) cout << "Exception Unknown" << endl;
            return -1;
        }
    }
    else{
        if(console_output) cout << "Can't get maximum velocity. Device not detected and initialized." << endl;
        return 0;
    }
}


/*Functions for controlling the zaber rotary device via TCP protobuf commands
 * Overloaded for zaber device class
 * Decodes the request and forms an appropriate response
 * Possible Turntable Responses
    INVALID_REQUEST = 0;
    DEVICE_CONNECTED = 1; //bool
    DEVICE_INITIALIZED = 2; //bool
    DEVICE_BUSY = 3; //bool
    DEVICE_NAME = 4; //string
    DEVICE_MANUFACTURER = 5; //string
    DEVICE_MODEL = 6; //string
    DEVICE_TYPE = 7; //string
    DEVICE_SUBTYPE = 8; //string
    SERVER_NAME = 9; //string
    TURNTABLE_POSITION_DEGREES = 100; //double
    TURNTABLE_MAXIMUM_VELOCITY_DEGREES_PER_SECOND = 101; //double
    TURNTABLE_MOVE_STARTED = 110; //bool
    TURNTABLE_CONSTANT_ROTATION_ENABLED = 111; //bool
    TURNTABLE_STOP_ISSUED = 112; //bool
*/
int zaberRotationStage::Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::Response &resp){

    //These requests can be performed regardless of device status
    if(req.request() == ASLpb::Request::IS_DEVICE_CONNECTED){
        cout << "Turntable Request: IS_DEVICE_CONNECTED" << endl;
        resp.set_response(ASLpb::Response::DEVICE_CONNECTED);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        resp.set_bool_param(Is_Device_Detected());
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE_CONNECTED = " << Is_Device_Detected() << endl;
    }
    else if(req.request() == ASLpb::Request::IS_DEVICE_INITIALIZED){
        cout << "Turntable Request: IS_DEVICE_INITIALIZED" << endl;
        resp.set_response(ASLpb::Response::DEVICE_INITIALIZED);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        resp.set_bool_param(Is_Device_Initialized());
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE_INITIALIZED = " << Is_Device_Initialized() << endl;
    }
    else if(req.request() == ASLpb::Request::IS_DEVICE_BUSY){
        cout << "Turntable Request: IS_DEVICE_BUSY" << endl;
        resp.set_response(ASLpb::Response::DEVICE_BUSY);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        resp.set_bool_param(Is_Device_Busy());
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE_BUSY = " << Is_Device_Busy() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_NAME){
        cout << "Turntable Request: GET_DEVICE_NAME" << endl;
        resp.set_response(ASLpb::Response::DEVICE_NAME);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.set_string_param(Get_Device_Name());
        resp.clear_bool_param();
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE_NAME = " << Get_Device_Name() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_MANUFACTURER){
        cout << "Turntable Request: GET_DEVICE_MANUFACTURER" << endl;
        resp.set_response(ASLpb::Response::DEVICE_MANUFACTURER);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.set_string_param(Get_Device_Manufacturer());
        resp.clear_bool_param();
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE_MANUFACTURER = " << Get_Device_Manufacturer() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_MODEL){
        cout << "Turntable Request: GET_DEVICE_MODEL" << endl;
        resp.set_response(ASLpb::Response::DEVICE_MODEL);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.set_string_param(Get_Device_Model());
        resp.clear_bool_param();
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE_MODEL = " << Get_Device_Model() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_TYPE){
        cout << "Turntable Request: GET_DEVICE_TYPE" << endl;
        resp.set_response(ASLpb::Response::DEVICE__TYPE);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.set_string_param(Get_Device_Type_As_String());
        resp.clear_bool_param();
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE__TYPE = " << Get_Device_Type_As_String() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_SUBTYPE){
        cout << "Turntable Request: GET_DEVICE_SUBTYPE" << endl;
        resp.set_response(ASLpb::Response::DEVICE_SUBTYPE);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.set_string_param(Get_Device_Subtype_As_String());
        resp.clear_bool_param();
        resp.set_status("OK");
        cout << "Turntable Response: DEVICE_SUBTYPE = " << Get_Device_Subtype_As_String() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_TURNTABLE_POSITION_DEGREES){
        cout << "Turntable Request: GET_TURNTABLE_POSITION_DEGREES" << endl;
        resp.set_response(ASLpb::Response::TURNTABLE_POSITION_DEGREES);
        resp.add_double_param(Get_Angular_Position());
        resp.clear_int_param();
        resp.clear_string_param();
        resp.clear_bool_param();
        resp.set_status("OK");
        cout << "Turntable Response: TURNTABLE_POSITION_DEGREES = " << Get_Angular_Position() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_TURNTABLE_ROTATION_ENABLED){
        cout << "Turntable Request: GET_TURNTABLE_ROTATION_ENABLED" << endl;
        resp.set_response(ASLpb::Response::TURNTABLE_CONSTANT_ROTATION_ENABLED);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        resp.set_bool_param(Is_Device_Rotating_At_Constant_Velocity());
        resp.set_status("OK");
        cout << "Turntable Response: TURNTABLE_CONSTANT_ROTATION_ENABLED = " << Is_Device_Rotating_At_Constant_Velocity() << endl;
    }
    else if(req.request() == ASLpb::Request::HOME_TURNTABLE){
        cout << "Turntable Request: HOME_TURNTABLE" << endl;
        resp.set_response(ASLpb::Response::TURNTABLE_MOVE_STARTED);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        if(Home_Device()){
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Turntable Response: TURNTABLE_HOME_STARTED = " << true << endl;
        }
        else{
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Turntable Response: TURNTABLE_HOME_STARTED = " << false << endl;
            return 0;
        }
    }
    else if(req.request() == ASLpb::Request::SET_TURNTABLE_ABSOLUTE_POSITION_DEGREES){
        double abspos = req.double_param(0);
        cout << "Turntable Request: SET_TURNTABLE_ABSOLUTE_POSITION_DEGREES to " << abspos << " degrees"<< endl;
        resp.set_response(ASLpb::Response::TURNTABLE_MOVE_STARTED);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        if(Set_Angular_Position(abspos)){
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Turntable Response: TURNTABLE_MOVE_STARTED to " << abspos << " degrees (absolute) = " << true << endl;
        }
        else{
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Turntable Response: TURNTABLE_MOVE_STARTED to " << abspos << " degrees (absolute) = " << false << endl;
            return 0;
        }
    }
    else if(req.request() == ASLpb::Request::SET_TURNTABLE_RELATIVE_POSITION_DEGREES){
        double relpos = req.double_param(0);
        cout << "Turntable Request: SET_TURNTABLE_RELATIVE_POSITION_DEGREES" << endl;
        resp.set_response(ASLpb::Response::TURNTABLE_MOVE_STARTED);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        if(Set_Relative_Angular_Position(relpos)){
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Turntable Response: TURNTABLE_MOVE_STARTED to " << relpos << " degrees (relative) = " << true << endl;
        }
        else{
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Turntable Response: TURNTABLE_MOVE_STARTED to " << relpos << " degrees (relative) = " << false << endl;
            return 0;
        }
    }
    else if(req.request() == ASLpb::Request::START_ROTATION_AT_CONSTANT_VELOCITY){
        double vel = req.double_param(0);
        cout << "Turntable Request: START_ROTATION_AT_CONSTANT_VELOCITY" << endl;
        resp.set_response(ASLpb::Response::TURNTABLE_CONSTANT_ROTATION_ENABLED );
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        if(Rotate_At_Constant_Velocity(vel)){
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Turntable Response: TURNTABLE_CONSTANT_ROTATION_ENABLED at " << vel << " degrees per second = " << true << endl;
            turntable_rotation_enabled = true;
        }
        else{
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Turntable Response: TURNTABLE_CONSTANT_ROTATION_ENABLED at " << vel << " degrees per second = " << false << endl;
            return 0;
        }
    }
    else if(req.request() == ASLpb::Request::STOP_TURNTABLE){
        cout << "Turntable Request: STOP_TURNTABLE" << endl;
        resp.set_response(ASLpb::Response::TURNTABLE_STOP_ISSUED);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        if(Stop_Device()){
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Turntable Response: TURNTABLE_STOP_ISSUED = " << true << endl;
            turntable_rotation_enabled = false;
        }
        else{
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Turntable Response: TURNTABLE_STOP_ISSUED = " << true << endl;
            return 0;
        }
    }
    else{ //Invalid request for this device
        cout << "Invalid Turntable Request Received. No action taken." << endl;
        resp.set_response(ASLpb::Response::INVALID_REQUEST);
        resp.clear_double_param();
        resp.clear_int_param();
        resp.clear_string_param();
        resp.clear_bool_param();
        resp.set_status("INVALID_REQUEST");
        cout << "Turntable Response: INVALID_REQUEST for device " << dev_ptr->Get_Device_Name() << endl;
        return -1;
    }
    return 1;
}

//Functions for sending TCP requests to a given device
// Overloaded for zaber device class
// Since this is a non-imaging device it will return a simple image response
// that indicates this is a non-imaging device
int zaberRotationStage::Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::ImageResponse &imgresp){
    cout << "Invalid Turntable Request Received. No action taken." << endl;
    imgresp.set_camera(ASLpb::ImageResponse::NON_CAMERA_DEVICE);
    imgresp.set_status("INVALID_REQUEST");
    cout << "Turntable Response: INVALID_REQUEST for device " << dev_ptr->Get_Device_Name() << endl;
    return -1;
}




/***********************************************************************************
 * blackflyCamera Class Member Functions
 *
 * NOTES:
 * 1.) Can I detect if a camera disconnects so that I can set detect and init flags
 *      to false so that it will constantly try to reconnect?
 *      -If so, could probably remove the detection thread and only do detection when
 *        a camera is plugged into the system. How do I detect this?
 */


blackflyCamera::blackflyCamera(){

    //Set Device Class Default Values
    device_name = "General Blackfly Camera";
    device_manufacturer = "FLIR";
    device_model = "Undefined Blackfly Model";
    device_type = camera;
    device_subtype = no_device_subtype;
    device_initialized = false;
    device_detected = false;

    //Set Blackfly Class Default Values
    image_width = 2448;
    image_height = 2048;
    number_of_channels = 1;
    image_npixels = image_width*image_height*number_of_channels;
    image.resize(image_npixels);
    dimage.resize(image_npixels);

    exposure_time = 1000;
    gain = 1.0;
    gamma = 1.0;
    adc_bit_depth = 12;
    manual_stream_buffer_count = 1;
    number_of_frames_to_average = 1;
}

//Set image width
void blackflyCamera::Set_Image_Width(unsigned int w){
    image_width = w;
    image_npixels = image_width*image_height;
    image.resize(image_npixels*number_of_channels);
    dimage.resize(image_npixels*number_of_channels);
}

//Set image height
void blackflyCamera::Set_Image_Height(unsigned int h){
    image_height = h;
    image_npixels = image_width*image_height;
    image.resize(image_npixels*number_of_channels);
    dimage.resize(image_npixels*number_of_channels);
}

//Set number of channels
void blackflyCamera::Set_Number_Of_Channels(unsigned int num_channels){
    number_of_channels = num_channels;
    image.resize(image_npixels*number_of_channels);
    dimage.resize(image_npixels*number_of_channels);
}



// Starts a device detection thread that will continually attempt to detect and then initialize
// a device.
//void blackflyCamera::Start_Device_Detection(){
//    device_detection = thread(&blackflyCamera::Detect_Device,this);
//    device_detection.join();
//}

// Attempts to connect to a blackfly camera. If no camera is detected, it will attempt
// to connect to a device 20 times before giving up.
void blackflyCamera::Detect_Device(){

    for(unsigned int k=0; k<number_of_detection_tries; k++){
        if((!Is_Device_Detected()) && (!Is_Device_Initialized())){

            cout << "Attempting to Detect Blackfly Camera Device:" << endl;
            cout << "===========================================" << endl;
            cout << "Attempting to detect device... " << k+1 << " of " << number_of_detection_tries << "."<< endl;
            cout << " ->Getting system instance." << endl;
            this->system = System::GetInstance();
            cout << " ->Getting camera list." << endl;
            this->camList = this->system->GetCameras();
            const unsigned int numCameras = camList.GetSize();
            cout << " ->Number of cameras detected: " << numCameras << "." << endl;
            if (numCameras == 0)
            {
                camList.Clear();
                system->ReleaseInstance();
                cout << " ->No cameras detected!" << endl << endl;
//                sleep_until(system_clock::now() + 1s); //Timeout for 1 second
            }
            else
            {
                //Make sure that we detect the camera we are looking for
                for(unsigned int k=0; k<numCameras; k++){
                    pCam = camList.GetByIndex(k); //This picks the first detected camera in the list...
                    INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
                    CStringPtr ptrDeviceModelName = nodeMapTLDevice.GetNode("DeviceModelName");
                    string detected_model_name = ptrDeviceModelName->ToString().c_str();
                    if(device_model == detected_model_name){
                        cout << "Device " << device_model << " Successfully Detected." << endl;
                        device_detected = true;
                        break;
                    }
                    else{
                        cout << " ->Incorrect camera detected: " << detected_model_name << endl;
                    }
                }
                if(device_detected){
                    cout << endl;

                    //Now, attempt to initialize_camera
                    cout << "Attempting to initialize device..." << endl;
                    pCam->Init();
                    Initialize_Device();
                    if(Is_Device_Initialized()){
                        cout << "Device successfully initialized." << endl << endl;
                        break;
                    }
                    else cout << "Device initialization failed." << endl << endl;
                }
            }
        }
        // If device is detected and not initialized, keep trying to initialize the
        // device until successful.
        else if((Is_Device_Detected()) && (!Is_Device_Initialized())){
            // Re-attempt to initialize device
            cout << "Re-attempting device initialization." << endl;
            pCam->Init();
            Initialize_Device();
            if(Is_Device_Initialized()){
                cout << "Device successfully initialized." << endl << endl;
            }
            else cout << "Device initialization failed." << endl << endl;
        }
        // If somehow the device is not detected but somehow is initialized set
        // the initialization flag to false.
        else if((!Is_Device_Detected()) && (Is_Device_Initialized())){
            device_initialized = false;
        }
        // Otherwise, all is well when the device is both detected and initialized.
        else{
            break;
        }
    }

    //If number of attempts exhausted and device not detected...
    if(!Is_Device_Detected() & !Is_Device_Initialized()){
        cout << "Device detection timed out. Device not detected or initialized." << endl << endl;
    }
}

// This function will attempt to initialize a Blackfly camera. Based upon the type of
// Blackfly camera it will initialize the default values of certain parameters differently.
// Many of the parameters are then meant to be dynamically updated, such as the exposure
// time, gain, and other parameters. The default camera values are as follows:
//
//  Sets acquisition mode to continuous
//  Sets pixel format to 16-bit (Mono16 for mono cameras, BayerRG16 for color microgrid)
//  Sets ADC bit depth to 12
//  Autoexposure set to Off
//  Sets exposure time to 1000 microseconds (1ms)
//  Autogain Set to Off
//  Gain value set to 1.0
//  Gamma functionality disabled
//  StreamBufferCountMode set to manual
//  StreamBufferCountManual set to 1
void blackflyCamera::Initialize_Device(){

        lock_guard<mutex> lock(this->mutex_camera_busy);
        INodeMap& nodeMap = pCam->GetNodeMap();

        // Turn Acquisition Mode to Continuous.
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) if(console_output) cout << "Unable to set acquisition mode to continuous" << endl;
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (IsAvailable(ptrAcquisitionModeContinuous) && IsReadable(ptrAcquisitionModeContinuous))
        {
            if(console_output) cout << " ->Acquisition mode set to continuous." << endl;
            const int64_t value = ptrAcquisitionModeContinuous->GetValue();
            ptrAcquisitionMode->SetIntValue(value);
        }


        // Set default Pixel Format to 16 bit
        CEnumerationPtr pixel_format_ptr = nodeMap.GetNode("PixelFormat");
        gcstring pixel_depth;
        if (this->device_model == "Blackfly S BFS-U3-51S5PC"){
            pixel_bit_depth = "BayerRG16";
            pixel_depth = "BayerRG16";
        }
        else{
            pixel_bit_depth = "Mono16";
            pixel_depth = "Mono16";
        }

        CEnumEntryPtr pixel_format_entry_ptr = pixel_format_ptr->GetEntryByName(pixel_depth);
        if (IsAvailable(pixel_format_entry_ptr) && IsReadable(pixel_format_entry_ptr)) {
            int64_t value = pixel_format_entry_ptr->GetValue();
            pixel_format_ptr->SetIntValue(value);
            if(console_output) cout << " ->Pixel format set to " << pixel_depth.c_str() << "." << endl;
        }
        else if(console_output) cout << " ->Failed to set pixel format to " << pixel_depth.c_str() << "." << endl;

        // Set Default ADC Bit Depth to 12 (other options are Bit8 and Bit10)
        CEnumerationPtr adc_ptr = nodeMap.GetNode("AdcBitDepth");
        CEnumEntryPtr adc_entry_ptr = adc_ptr->GetEntryByName("Bit12");
        if (IsAvailable(adc_entry_ptr) && IsReadable(adc_entry_ptr))
        {
            int64_t value = adc_entry_ptr->GetValue();
            adc_ptr->SetIntValue(value);
        }


        // Turn Autoexposure off and set the initial value.
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (IsAvailable(ptrExposureAuto) || IsWritable(ptrExposureAuto))
        {
            CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
            if (IsAvailable(ptrExposureAutoOff) || IsWritable(ptrExposureAutoOff)) ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
            else if(console_output) cout << " ->Automatic exposure successfully disabled..." << endl;
        }
        else if(console_output) cout << " ->Unable to disable automatic exposure (node retrieval). Aborting..." << endl;

        // Set Exposure Time
        CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
        if (IsAvailable(ptrExposureTime) || IsWritable(ptrExposureTime))
        {
            // Ensure desired exposure time does not exceed the maximum
            const double exposureTimeMax = ptrExposureTime->GetMax();
            double exposureTimeToSet = exposure_time;
            if (exposureTimeToSet > exposureTimeMax)
            {
                exposureTimeToSet = exposureTimeMax;
                if(console_output) cout << " ->Exposure time set to " << exposure_time << "." << endl;
            }
            ptrExposureTime->SetValue(exposureTimeToSet);
        }
        else if(console_output) cout << " ->Unable to set exposure time." << endl;

        // Turn AutoGain off
        CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
        CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
        if (IsAvailable(ptrGainAutoOff) || IsWritable(ptrGainAutoOff))
        {
            ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());
            if(console_output) cout << " ->Gain set to manual." << endl;
        }
        else if(console_output) cout << " ->Unable to disable automatic gain." << endl;

        //Set Initial Gain Value
        CFloatPtr ptrGain = nodeMap.GetNode("Gain");
        if (IsAvailable(ptrGain) || IsWritable(ptrGain))
        {
            // Ensure desired exposure time does not exceed the maximum
            const double gainMax = ptrGain->GetMax();
            double gainToSet = gain;
            if (gainToSet > gainMax) gainToSet = gainMax;
            if(console_output) cout << " ->Gain set to " << gain << "." << endl;
            ptrGain->SetValue(gainToSet);
        }
        else if(console_output) cout << " ->Unable to set gain." << endl;

        // Set gamma value
        CFloatPtr ptrGamma = nodeMap.GetNode("Gamma");
        if (IsAvailable(ptrGamma) || IsWritable(ptrGamma))
        {
            // Ensure desired exposure time does not exceed the maximum
            const double gammaMax = ptrGamma->GetMax();
            double gammaToSet = gamma;
            if (gammaToSet > gammaMax) gammaToSet = gammaMax;
            if(console_output) cout << " ->Gamma set to " << gamma << "." << endl;
            ptrGamma->SetValue(gammaToSet);
        }
        else if(console_output) cout << " ->Unable to set gamma." << endl;

        // Turn off gamma
        CBooleanPtr ptrGammaEnable = nodeMap.GetNode("GammaEnable");
        if (IsAvailable(ptrGammaEnable) || IsWritable(ptrGammaEnable))
        {
            ptrGammaEnable->SetValue(false);
            if(console_output) cout << " ->Gamma disabled." << endl;
        }
        else if(console_output) cout << " ->Unable to disable gamma." << endl;

        // Turn StreamBufferCountMode to manual
        INodeMap& nodeMapStream = pCam->GetTLStreamNodeMap();
        CEnumerationPtr ptrStream = nodeMapStream.GetNode("StreamBufferCountMode");
        CEnumEntryPtr ptrStreamManual = ptrStream->GetEntryByName("Manual");
        if (IsAvailable(ptrStreamManual) || IsWritable(ptrStreamManual))
        {
            ptrStream->SetIntValue(ptrStreamManual->GetValue());
            if(console_output) cout << " ->Stream mode set to manual." << endl;
        }
        else if(console_output) cout << " ->Unable to set stream mode to manual." << endl;

        // Set StreamBufferCountManual to 1
        CIntegerPtr ptrCount = nodeMapStream.GetNode("StreamBufferCountManual");
        if (IsAvailable(ptrCount) || IsWritable(ptrCount))
        {
            ptrCount->SetValue(manual_stream_buffer_count);
            if(console_output) cout << " ->Manual stream buffer count set to " << manual_stream_buffer_count << "." << endl;
        }
        else if(console_output) cout << " ->Unable to set stream buffer count." << endl;

        device_initialized = true;
}

// Returns camera busy state. Need to determine where to set this flag in the code.
bool blackflyCamera::Is_Device_Busy(){
    return device_busy;
}

// Updates the camera exposure time
// NOTE: Probably need to read the max and min values from the camera
bool blackflyCamera::Set_Exposure_Time(double et){

    if((et<6) || (et>30000000)){
        if(console_output) cout << " ->Unable to set exposure time. Specified exposure time is out of range [6, 3.0E7] microseconds." << endl;
        return false;
    }

    if(Is_Device_Detected() & Is_Device_Initialized()){
        lock_guard<mutex> lock(this->mutex_camera_busy);
        INodeMap& nodeMap = pCam->GetNodeMap();
        CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
        if (IsAvailable(ptrExposureTime) && IsWritable(ptrExposureTime))
        {
          // Ensure desired exposure time does not exceed the maximum
          const double exposureTimeMax = ptrExposureTime->GetMax();
          double exposureTimeToSet = et;
          if (exposureTimeToSet > exposureTimeMax) exposureTimeToSet = exposureTimeMax;
          exposure_time = et;
          ptrExposureTime->SetValue(exposureTimeToSet);
          return true;
        }
        else{
            if(console_output) cout << "Unable to set exposure time." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Unable to set exposure time. Device not initialized." << endl;
        return false;
    }
}


//Set camera gain value
// NOTE: Probably need to read the max and min values from the camera
bool blackflyCamera::Set_Gain(double g){

    if((g<0) || (g>47)){
        if(console_output) cout << " ->Unable to set gain value. Specified gain is out of range [0.0, 47.0]." << endl;
        return false;
    }

    if(Is_Device_Detected() & Is_Device_Initialized()){
        lock_guard<mutex> lock(this->mutex_camera_busy);
        INodeMap& nodeMap = pCam->GetNodeMap();
        CFloatPtr ptrGain = nodeMap.GetNode("Gain");
        if (IsAvailable(ptrGain) && IsWritable(ptrGain))
        {
          // Ensure desired gain does not exceed the maximum
          const double gainMax = ptrGain->GetMax();
          double gainToSet = g;
          if (gainToSet > gainMax) gainToSet = gainMax;
          gain = gainToSet;
          ptrGain->SetValue(gainToSet);
          return true;
        }
        else{
            if(console_output) cout << "Unable to set gain." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Unable to set gain. Device not initialized." << endl;
        return false;
    }
}

//Will enable or disable gamma scaling
bool blackflyCamera::Set_Gamma_Enable(bool enable){

    if((enable) && (Get_Gamma_Enable())){
        if(console_output) cout << " ->Gamma already enabled." << endl;
        return true;
    }
    else if((!enable) && (!Get_Gamma_Enable())){
        if(console_output) cout << " ->Gamma already disabled." << endl;
        return true;
    }

    if(Is_Device_Detected() & Is_Device_Initialized()){
        lock_guard<mutex> lock(this->mutex_camera_busy);
        INodeMap& nodeMap = pCam->GetNodeMap();
        CBooleanPtr ptrGammaEnable = nodeMap.GetNode("GammaEnable");
        if (IsAvailable(ptrGammaEnable) && IsWritable(ptrGammaEnable))
        {
            gamma_enable = enable;
            ptrGammaEnable->SetValue(enable);
            if(enable){ if(console_output) cout << " ->Gamma enabled." << endl;}
            else{ if(console_output) cout << " ->Gamma disabled." << endl;}
            return true;
        }
        else{
            if(enable){ if(console_output) cout << " ->Unable to enable gamma." << endl;}
            else{ if(console_output) cout << " ->Unable to disable gamma." << endl;}
            return false;
        }
    }
    else{
        if(enable){ if(console_output) cout << " ->Unable to enable gamma. Device not Initialized." << endl;}
        else{ if(console_output) cout << " ->Unable to disable gamma. Device not Initialized." << endl;}
        return false;
    }
}

//Set Gamma value on camera
// NOTE: Probably need to read the max and min values from the camera
bool blackflyCamera::Set_Gamma(double g){

    if(!Get_Gamma_Enable()){
        if(console_output) cout << " ->Unable to set gamma value. Gamma is not enabled." << endl;
        return false;
    }
    if((g<0.25) || (g>4)){
        if(console_output) cout << " ->Unable to set gamma value. Specified gamma is out of range [0.25, 4]." << endl;
        return false;
    }

    if(Is_Device_Detected() & Is_Device_Initialized()){
        lock_guard<mutex> lock(this->mutex_camera_busy);
        INodeMap& nodeMap = pCam->GetNodeMap();
        CFloatPtr ptrGamma = nodeMap.GetNode("Gamma");
        if (IsAvailable(ptrGamma) && IsWritable(ptrGamma))
        {
            const double gammaMax = ptrGamma->GetMax();
            double gammaToSet = g;
            if (gammaToSet > gammaMax) gammaToSet = gammaMax;
            if(console_output) cout << " ->Gamma set to " << g << "." << endl;
            gamma = gammaToSet;
            ptrGamma->SetValue(gammaToSet);            
            return true;
        }
        else{
            if(console_output) cout << " ->Unable to set gamma." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << " ->Unable to set gamma. Device not Initialized." << endl;
        return false;
    }
}

//Set ADC bit depth value on camera
bool blackflyCamera::Set_ADC_Bit_Depth(unsigned int adc){

    if(!((adc == 10) || (adc == 12))){
        if(console_output) cout << "Unable to set ADC bit depth. ADC value must be set to 10 or 12." << endl;
        return false;
    }

    if(Is_Device_Detected() & Is_Device_Initialized()){
        //Check that ADC value is 10 or 12
        INodeMap& nodeMap = pCam->GetNodeMap();
        CEnumerationPtr adc_ptr = nodeMap.GetNode("AdcBitDepth");
        if(adc==10){
            CEnumEntryPtr adc_entry_ptr = adc_ptr->GetEntryByName("Bit10");
            if (IsAvailable(adc_entry_ptr) && IsReadable(adc_entry_ptr))
            {
                int64_t value = adc_entry_ptr->GetValue();
                adc_ptr->SetIntValue(value);
                adc_bit_depth = adc;
            }
            return true;
        }
        else if(adc==12){
            CEnumEntryPtr adc_entry_ptr = adc_ptr->GetEntryByName("Bit12");
            if (IsAvailable(adc_entry_ptr) && IsReadable(adc_entry_ptr))
            {
                int64_t value = adc_entry_ptr->GetValue();
                adc_ptr->SetIntValue(value);
                adc_bit_depth = adc;
            }
            return true;
        }
        else {
            if(console_output) cout << "Unable to set ADC bit depth. Must be from set {8, 10, 12}." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Unable to set ADC bit depth. Device not initialized." << endl;
        return false;
    }
}

//Attempts to set pixel bit depth format to specified string
/*
Possible Enumerations. Need to determine which are acceptable for each camera
Mono8
Mono16
BayerGR8
BayerRG8
BayerGB8
BayerBG8
BayerGR16
BayerRG16
BayerGB16
BayerBG16
Polarized8
Polarized16
BayerRGPolarized8
BayerRGPolarized16
*/
bool blackflyCamera::Set_Pixel_Bit_Depth(string pbd){

    //Check that a valid format is selected for a given camera
    //Color Visible Microgrid
    if (this->device_model == "Blackfly S BFS-U3-51S5PC"){
        if(!(pbd == "BayerRG16" || pbd == "BayerRG8")){
            if(console_output) cout << "Unable to set pixel bit depth. Invalid pixel bit depth selected." << endl;
            return false;
        }
    }
    //Mono Visible Microgrid
    if (this->device_model == "Blackfly S BFS-U3-51S5P"){
        if(!(pbd == "Mono16" || pbd == "Mono8")){
            if(console_output) cout << "Unable to set pixel bit depth. Invalid pixel bit depth selected." << endl;
            return false;
        }
    }
    //Mono Visible Microgrid
    if (this->device_model == "Blackfly S BFS-U3-51S5M"){
        if(!(pbd == "Mono16" || pbd == "Mono8")){
            if(console_output) cout << "Unable to set pixel bit depth. Invalid pixel bit depth selected." << endl;
            return false;
        }
    }
    //Mono Visible DoT
    if (this->device_model == "Blackfly S BFS-U3-51S5M + Thorlabs ELL14K"){
        if(!(pbd == "Mono16" || pbd == "Mono8")){
            if(console_output) cout << "Unable to set pixel bit depth. Invalid pixel bit depth selected." << endl;
            return false;
        }
    }

    if(Is_Device_Detected() & Is_Device_Initialized()){
        INodeMap& nodeMap = pCam->GetNodeMap();
        CEnumerationPtr pixel_format_ptr = nodeMap.GetNode("PixelFormat");
        gcstring pixel_depth(pbd.c_str());
        pixel_bit_depth = pbd;

        CEnumEntryPtr pixel_format_entry_ptr = pixel_format_ptr->GetEntryByName(pixel_depth);
        if (IsAvailable(pixel_format_entry_ptr) && IsReadable(pixel_format_entry_ptr)) {
            int64_t value = pixel_format_entry_ptr->GetValue();
            pixel_format_ptr->SetIntValue(value);
            if(console_output) cout << " ->Pixel format set to " << pixel_depth.c_str() << "." << endl;
            return true;
        }
        else{
            if(console_output) cout << " ->Failed to set pixel format to " << pixel_depth.c_str() << "." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Unable to set pixel bit depth. Device not initialized." << endl;
        return false;
    }
}

//Set Manual Stream Buffer Count value on camera
bool blackflyCamera::Set_Manual_Stream_Buffer_Count(unsigned int bc){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        INodeMap& nodeMapStream = pCam->GetTLStreamNodeMap();
        CIntegerPtr ptrCount = nodeMapStream.GetNode("StreamBufferCountManual");
        if (IsAvailable(ptrCount) || IsWritable(ptrCount))
        {
            manual_stream_buffer_count = bc;
            ptrCount->SetValue(bc);
            if(console_output) cout << "Set stream buffer count to " << bc << endl << endl;
            return true;
        }
        else{
            if(console_output) cout << "Unable to set stream buffer count." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Unable to set stream buffer count. Device not initialized." << endl;
        return false;
    }
}

//Set number of frames to average per acquired scene image
void blackflyCamera::Set_Number_Of_Frames_To_Average(unsigned int num_frames_avg){
    number_of_frames_to_average = num_frames_avg;
}

// This function will collect an image from the current Blackfly device using the
// current camera settings. The acquired "averaged" image is stored in the blackFly
// class QVector class variable image.
bool blackflyCamera::Get_Image(){

    if(Is_Device_Detected() & Is_Device_Initialized()){

        //Begin Image Acquisition
        try {
            if(console_output) cout << "Acquiring Image..." << endl;
            pCam->BeginAcquisition();
//            mutex_camera_busy.lock();
//            mutex_camera_busy.unlock();
            dimage.fill(0);
            for (unsigned int imageCnt = 0; imageCnt < number_of_frames_to_average; imageCnt++)
            {
                try {
                    acquiredFrame  = this->pCam->GetNextImage(1000);
                    uint16_t* raw_data = (uint16_t*)acquiredFrame->GetData();

                    if (imageCnt == 0)
                    {
                        if(console_output) cout << " ->Pixel format: " << acquiredFrame->GetPixelFormatName() << endl;
                        if(console_output) cout << " ->Size of raw data: " << acquiredFrame->GetBufferSize() << endl;
                        if(console_output) cout << " ->Bits per pixel: " << acquiredFrame->GetBitsPerPixel() << endl;
                        if(console_output) cout << " ->Number of channels: " << acquiredFrame->GetNumChannels() << endl;
                        if(console_output) cout << " ->Number of images averaged: " << number_of_frames_to_average << endl;
                        if(console_output) cout << " ->Timestamp: " << Get_Timestamp(false) << endl << endl;
                    }

                    // Average frames
                    for (unsigned int n=0;n<image_npixels;n++){
                        dimage[n] = dimage[n]+double(raw_data[n])/number_of_frames_to_average;
                    }
                    acquiredFrame->Release();
                }
                catch (Spinnaker::Exception& e){
                    if(console_output) cout << "Error: " << e.what() << endl;
                }
            }
            this->pCam->EndAcquisition();

            // Convert averaged frame to 32-bit format.
            image.resize(image_npixels);
            for (unsigned int n=0;n<image_npixels;n++){
                image[n] = uint32_t(dimage[n]);
            }
            return true;
        }
        catch (Spinnaker::Exception& e)
        {
            if(console_output) cout << "Error: " << e.what() << endl;
            return false;
        }
    }
    else{
        if(!Is_Device_Detected()){
            if(console_output) cout << "Unable to acquire image. Device not detected." << endl;
        }
        else{ if(!Is_Device_Initialized())
            if(console_output) cout << "Unable to acquire image. Device not initialized." << endl;}
        return false;
    }

}

// This function will collect an image from the current Blackfly device using the
// current camera settings with the exception that only a single frame is acquired.
// This is meant to obtain a single exposure image for camera configuration on the
// client side. The acquired image is stored in the blackFly class QVector class
// variable image.
bool blackflyCamera::Get_Single_Image(){

    if(Is_Device_Detected() & Is_Device_Initialized()){

        //Begin Image Acquisition
        try {
            if(console_output) cout << "Acquiring Single Image..." << endl;
            pCam->BeginAcquisition();
//            mutex_camera_busy.lock();
//            mutex_camera_busy.unlock();
//            dimage.fill(0);

            try {
                acquiredFrame  = this->pCam->GetNextImage(1000);
                uint16_t* raw_data = (uint16_t*)acquiredFrame->GetData();

                if(console_output) cout << " ->Pixel format: " << acquiredFrame->GetPixelFormatName() << endl;
                if(console_output) cout << " ->Size of raw data: " << acquiredFrame->GetBufferSize() << endl;
                if(console_output) cout << " ->Bits per pixel: " << acquiredFrame->GetBitsPerPixel() << endl;
                if(console_output) cout << " ->Number of channels: " << acquiredFrame->GetNumChannels() << endl;
                if(console_output) cout << " ->Number of images averaged: " << number_of_frames_to_average << endl;
                if(console_output) cout << " ->Timestamp: " << Get_Timestamp(false) << endl << endl;

                // Convert frame to 32-bit format.
                image.resize(image_npixels);
                for (unsigned int n=0;n<image_npixels;n++){
                    image[n] = uint32_t(raw_data[n]);
                    if(n<10) cout << "raw_data[" << n << "] = " << raw_data[n] << "   image[" << n << "] = " << image[n] << endl;
                }
                acquiredFrame->Release();
            }
            catch (Spinnaker::Exception& e){
                if(console_output) cout << "Error: " << e.what() << endl;
            }
            this->pCam->EndAcquisition();
            return true;
        }
        catch (Spinnaker::Exception& e){
            if(console_output) cout << "Error: " << e.what() << endl;
            return false;
        }
    }
    else{
        if(!Is_Device_Detected()){
            if(console_output) cout << "Unable to acquire image. Device not detected." << endl;
        }
        else{ if(!Is_Device_Initialized())
            if(console_output) cout << "Unable to acquire image. Device not initialized." << endl;}
        return false;
    }
}


/*Functions for controlling the blackfly camera device via TCP protobuf commands
 * Overloaded for blackfly camera class
 * Decodes the request and forms an appropriate response
 * Possible Turntable Responses
        INVALID_REQUEST = 0;
        DEVICE_CONNECTED = 1; //bool
        DEVICE_INITIALIZED = 2; //bool
        DEVICE_BUSY = 3; //bool
        DEVICE_NAME = 4; //string
        DEVICE_MANUFACTURER = 5; //string
        DEVICE_MODEL = 6; //string
        DEVICE__TYPE = 7; //string
        DEVICE_SUBTYPE = 8; //string
        CAMERA_IMAGE_WIDTH = 50; //uint32
        CAMERA_IMAGE_HEIGHT = 51; //uint32
        CAMERA_EXPOSURE_TIME = 52; //double
        CAMERA_GAIN = 53; //double
        CAMERA_GAMMA = 54; //double
        CAMERA_ADC_BIT_DEPTH = 55; //uint32
        CAMERA_PIXEL_BIT_DEPTH = 56; //uint32
        CAMERA_NUMBER_FRAMES_TO_AVERAGE = 57; //uint32
        MANUAL_STREAM_BUFFER_COUNT = 58; //uint32
        NUMBER_OF_CAMERA_CHANNELS = 59; //uint32
        DOT_CAMERA_POLARIZER_ANGLES = 60; //repeated double
        CAMERA_SETTING_APPLIED_SUCCESSFULLY = 75; //bool
*/
int blackflyCamera::Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::Response &resp){

    //These requests can be performed regardless of device status
    if(req.request() == ASLpb::Request::IS_DEVICE_CONNECTED){
        cout << "Blackfly Camera Request: IS_DEVICE_CONNECTED" << endl;
        resp.set_response(ASLpb::Response::DEVICE_CONNECTED);
        resp.set_bool_param(Is_Device_Detected());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE_CONNECTED = " << Is_Device_Detected() << endl;
    }
    else if(req.request() == ASLpb::Request::IS_DEVICE_INITIALIZED){
        cout << "Blackfly Camera Request: IS_DEVICE_INITIALIZED" << endl;
        resp.set_response(ASLpb::Response::DEVICE_INITIALIZED);
        resp.set_bool_param(Is_Device_Initialized());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE_INITIALIZED = " << Is_Device_Initialized() << endl;
    }
    else if(req.request() == ASLpb::Request::IS_DEVICE_BUSY){
        cout << "Blackfly Camera Request: IS_DEVICE_BUSY" << endl;
        resp.set_response(ASLpb::Response::DEVICE_BUSY);
        resp.set_bool_param(Is_Device_Busy());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE_BUSY = " << Is_Device_Busy() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_NAME){
        cout << "Blackfly Camera Request: GET_DEVICE_NAME" << endl;
        resp.set_response(ASLpb::Response::DEVICE_NAME);
        resp.set_string_param(Get_Device_Name());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE_NAME = " << Get_Device_Name() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_MANUFACTURER){
        cout << "Blackfly Camera Request: GET_DEVICE_MANUFACTURER" << endl;
        resp.set_response(ASLpb::Response::DEVICE_MANUFACTURER);
        resp.set_string_param(Get_Device_Manufacturer());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE_MANUFACTURER = " << Get_Device_Manufacturer() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_MODEL){
        cout << "Blackfly Camera Request: GET_DEVICE_MODEL" << endl;
        resp.set_response(ASLpb::Response::DEVICE_MODEL);
        resp.set_string_param(Get_Device_Model());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE_MODEL = " << Get_Device_Model() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_TYPE){
        cout << "Blackfly Camera Request: GET_DEVICE_TYPE" << endl;
        resp.set_response(ASLpb::Response::DEVICE__TYPE);
        resp.set_string_param(Get_Device_Type_As_String());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE__TYPE = " << Get_Device_Type_As_String() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_DEVICE_SUBTYPE){
        cout << "Blackfly Camera Request: GET_DEVICE_SUBTYPE" << endl;
        resp.set_response(ASLpb::Response::DEVICE_SUBTYPE);
        resp.set_string_param(Get_Device_Subtype_As_String());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: DEVICE_SUBTYPE = " << Get_Device_Subtype_As_String() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_IMAGE_WIDTH){
        cout << "Blackfly Camera Request: GET_CAMERA_IMAGE_WIDTH" << endl;
        resp.set_response(ASLpb::Response::CAMERA_IMAGE_WIDTH);
        resp.add_int_param(Get_Image_Width());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_IMAGE_WIDTH = " << Get_Image_Width() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_IMAGE_HEIGHT){
        cout << "Blackfly Camera Request: GET_CAMERA_IMAGE_HEIGHT" << endl;
        resp.set_response(ASLpb::Response::CAMERA_IMAGE_HEIGHT);
        resp.add_int_param(Get_Image_Height());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_IMAGE_HEIGHT = " << Get_Image_Height() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_EXPOSURE_TIME){
        cout << "Blackfly Camera Request: GET_CAMERA_EXPOSURE_TIME" << endl;
        resp.set_response(ASLpb::Response::CAMERA_EXPOSURE_TIME);
        resp.add_double_param(Get_Exposure_Time());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_EXPOSURE_TIME = " << Get_Exposure_Time() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_GAIN){
        cout << "Blackfly Camera Request: GET_CAMERA_GAIN" << endl;
        resp.set_response(ASLpb::Response::CAMERA_GAIN);
        resp.add_double_param(Get_Gain());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_GAIN = " << Get_Gain() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_GAMMA_ENABLE){
        cout << "Blackfly Camera Request: GET_CAMERA_GAMMA_ENABLE" << endl;
        resp.set_response(ASLpb::Response::CAMERA_GAMMA_ENABLE);
        resp.set_bool_param(Get_Gamma_Enable());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_GAMMA_ENABLE = " << Get_Gamma() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_GAMMA){
        cout << "Blackfly Camera Request: GET_CAMERA_GAMMA" << endl;
        resp.set_response(ASLpb::Response::CAMERA_GAMMA);
        resp.add_double_param(Get_Gamma());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_GAMMA = " << Get_Gamma() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_ADC_BIT_DEPTH){
        cout << "Blackfly Camera Request: GET_CAMERA_ADC_BIT_DEPTH" << endl;
        resp.set_response(ASLpb::Response::CAMERA_ADC_BIT_DEPTH);
        resp.add_int_param(Get_ADC_Bit_Depth());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_ADC_BIT_DEPTH = " << Get_ADC_Bit_Depth() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_PIXEL_BIT_DEPTH){
        cout << "Blackfly Camera Request: GET_CAMERA_PIXEL_BIT_DEPTH" << endl;
        resp.set_response(ASLpb::Response::CAMERA_PIXEL_BIT_DEPTH);
        resp.set_string_param(Get_Pixel_Bit_Depth());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_PIXEL_BIT_DEPTH = " << Get_Pixel_Bit_Depth() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_NUMBER_FRAMES_TO_AVERAGE){
        cout << "Blackfly Camera Request: GET_CAMERA_NUMBER_FRAMES_TO_AVERAGE" << endl;
        resp.set_response(ASLpb::Response::CAMERA_NUMBER_FRAMES_TO_AVERAGE);
        resp.add_int_param(Get_Number_Of_Frames_To_Average());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: CAMERA_NUMBER_FRAMES_TO_AVERAGE = " << Get_Number_Of_Frames_To_Average() << endl;
    }
    else if(req.request() == ASLpb::Request::GET_MANUAL_STREAM_BUFFER_COUNT){
        cout << "Blackfly Camera Request: GET_MANUAL_STREAM_BUFFER_COUNT" << endl;
        resp.set_response(ASLpb::Response::MANUAL_STREAM_BUFFER_COUNT);
        resp.add_int_param(Get_Manual_Stream_Buffer_Count());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: MANUAL_STREAM_BUFFER_COUNT = " << Get_Manual_Stream_Buffer_Count() << endl;

    }
    else if(req.request() == ASLpb::Request::GET_NUMBER_OF_CAMERA_CHANNELS){
        cout << "Blackfly Camera Request: GET_NUMBER_OF_CAMERA_CHANNELS" << endl;
        resp.set_response(ASLpb::Response::NUMBER_OF_CAMERA_CHANNELS);
        resp.add_int_param(Get_Number_Of_Channels());
        resp.set_status("OK");
        cout << "Blackfly Camera Response: NUMBER_OF_CAMERA_CHANNELS = " << Get_Number_Of_Channels() << endl;
    }    
    else if(req.request() == ASLpb::Request::GET_DOT_CAMERA_POLARIZER_ANGLES){        
        if(device_model == "Blackfly S BFS-U3-51S5M + Thorlabs ELL14K"){
            cout << "Blackfly Camera Request: GET_DOT_CAMERA_POLARIZER_ANGLES" << endl;
            blackflyVisibleMonoDoT* temp = static_cast<blackflyVisibleMonoDoT*>(dev_ptr);
            QVector<double> pangles;
            temp->Get_Polarizer_Angles(pangles);
            resp.set_response(ASLpb::Response::DOT_CAMERA_POLARIZER_ANGLES);
            resp.set_status("OK");
            cout << "Blackfly Camera Response: DOT_CAMERA_POLARIZER_ANGLES {";
            for(int k=0; k<pangles.size(); k++){
                resp.add_double_param(pangles[k]);
                if(k==pangles.size()-1) cout << pangles[k] << "}" << endl;
                else cout << pangles[k] << ", ";
            }
        }
        else{
            cout << "Invalid Blackfly Camera Request Received. No action taken." << endl;
            resp.set_response(ASLpb::Response::INVALID_REQUEST);
            resp.set_status("INVALID_REQUEST");
            cout << "Blackfly Camera Response: INVALID_REQUEST" << endl;
            return -1;
        }
    }
    else if(req.request() == ASLpb::Request::GET_MICROGRID_CAMERA_POLARIZER_ANGLES){
        if(device_model == "Blackfly S BFS-U3-51S5P"){ //Blackfly Mono Microgrid
            cout << "Blackfly Camera Request: GET_MICROGRID_CAMERA_POLARIZER_ANGLES" << endl;
            blackflyVisibleMonoMicrogrid* temp = static_cast<blackflyVisibleMonoMicrogrid*>(dev_ptr);
            //Set up values
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_int_param(2);
            resp.add_int_param(2);
            resp.set_string_param("MMMM");
            resp.set_response(ASLpb::Response::MICROGRID_CAMERA_POLARIZER_ANGLES);
            resp.set_status("OK");
            cout <<  "Blackfly Camera Response: MICROGRID_CAMERA_POLARIZER_ANGLES: [2x2] -> {0,135,45,90}" << endl;
        }
        else if(device_model == "Blackfly S BFS-U3-51S5PC"){ //Blackfly RGB Microgrid
            cout << "Blackfly Camera Request: GET_MICROGRID_CAMERA_POLARIZER_ANGLES" << endl;
            blackflyVisibleRGBMicrogrid* temp = static_cast<blackflyVisibleRGBMicrogrid*>(dev_ptr);
            //Set up values
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_int_param(4);
            resp.add_int_param(4);
            resp.set_string_param("RRGGRRGGGGBBGGBB");

            resp.set_response(ASLpb::Response::MICROGRID_CAMERA_POLARIZER_ANGLES);
            resp.set_status("OK");
            cout <<  "Blackfly Camera Response: MICROGRID_CAMERA_POLARIZER_ANGLES: [4x4] -> {0,135,0,135,45,90,45,90,0,135,0,135,45,90,45,90} x {RRGGRRGGGGBBGGBB}" << endl;
        }
        else{
            cout << "Invalid Blackfly Camera Request Received. No action taken." << endl;
            resp.set_response(ASLpb::Response::INVALID_REQUEST);
            resp.set_status("INVALID_REQUEST");
            cout << "Blackfly Camera Response: INVALID_REQUEST" << endl;
            return -1;
        }
    }
    //This will send all camera settings parameters in a single message. The structure is
    //different depending upon the camera and thus is imporant for decoding the structure
    else if(req.request() == ASLpb::Request::GET_ALL_CAMERA_PARAMETERS){
        cout << "Blackfly Camera Request: GET_ALL_CAMERA_PARAMETERS" << endl;

        //Integer Parameters
        resp.add_int_param(Get_Image_Width());
        resp.add_int_param(Get_Image_Height());
        resp.add_int_param(Get_ADC_Bit_Depth());
        resp.add_int_param(Get_Number_Of_Frames_To_Average());
        resp.add_int_param(Get_Manual_Stream_Buffer_Count());
        resp.add_int_param(Get_Number_Of_Channels());

        //Double Parameters
        resp.add_double_param(Get_Exposure_Time());
        resp.add_double_param(Get_Gain());
        resp.add_double_param(Get_Gamma());

        //String Parameters
        resp.set_string_param(Get_Pixel_Bit_Depth());

        //Boolean Parameters
        resp.set_bool_param(Get_Gamma_Enable());


        if(device_model == "Blackfly S BFS-U3-51S5M + Thorlabs ELL14K"){
            blackflyVisibleMonoDoT* temp = static_cast<blackflyVisibleMonoDoT*>(dev_ptr);
            QVector<double> pangles;
            temp->Get_Polarizer_Angles(pangles);
            for(int k=0; k<pangles.size(); k++) resp.add_double_param(pangles[k]);
        }
        if(device_model == "Blackfly S BFS-U3-51S5P"){ //Blackfly Mono Microgrid
            //Set up values
            resp.add_int_param(2);
            resp.add_int_param(2);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.set_string_param(Get_Pixel_Bit_Depth() + ",MMMM");
        }
        else if(device_model == "Blackfly S BFS-U3-51S5PC"){ //Blackfly RGB Microgrid
            //Set up values
            resp.add_int_param(4);
            resp.add_int_param(4);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(0);
            resp.add_double_param(135);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.add_double_param(45);
            resp.add_double_param(90);
            resp.set_string_param(Get_Pixel_Bit_Depth() + ",RRGGRRGGGGBBGGBB");
        }
        resp.set_response(ASLpb::Response::ALL_CAMERA_PARAMETERS);
        resp.set_status("OK");
        cout <<  "Blackfly Camera Response: ALL_CAMERA_PARAMETERS sent successfully." << endl;
    }
    else if(req.request() == ASLpb::Request::SET_CAMERA_EXPOSURE_TIME){
        cout << "Blackfly Camera Request: SET_CAMERA_EXPOSURE_TIME" << endl;
        double exposure = req.double_param(0);
        bool success = Set_Exposure_Time(exposure);
        if(success){
            resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Blackfly Camera Response: CAMERA_EXPOSURE_TIME set to " << exposure << endl;
        }
        else{
            resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Blackfly Camera Response: CAMERA_INTEGRATION_TIME failed to set." << endl;
            return -1;
        }
    }
    else if(req.request() == ASLpb::Request::SET_CAMERA_GAIN){
        cout << "Blackfly Camera Request: SET_CAMERA_GAIN" << endl;
        double gain = req.double_param(0);
        bool success = Set_Gain(gain);
        if(success){
            resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Blackfly Camera Response: CAMERA_GAIN set to " << gain << endl;
        }
        else{
            resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Blackfly Camera Response: CAMERA_GAIN failed to set." << endl;
            return -1;
        }
    }
    else if(req.request() == ASLpb::Request::SET_CAMERA_ADC_BIT_DEPTH){
        cout << "Blackfly Camera Request: SET_CAMERA_ADC_BIT_DEPTH" << endl;
        unsigned int adc = req.int_param(0);
        bool success = Set_ADC_Bit_Depth(adc);
        if(success){
            resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Blackfly Camera Response: CAMERA_ADC_BIT_DEPTH set to " << adc << endl;
        }
        else{
            resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Blackfly Camera Response: CAMERA_ADC_BIT_DEPTH failed to set." << endl;
            return -1;
        }
    }
    else if(req.request() == ASLpb::Request::SET_CAMERA_PIXEL_BIT_DEPTH){
        cout << "Blackfly Camera Request: SET_CAMERA_PIXEL_BIT_DEPTH" << endl;
        string bitdepth = req.string_param();
        bool success = Set_Pixel_Bit_Depth(bitdepth);
        if(success){
            resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Blackfly Camera Response: CAMERA_PIXEL_BIT_DEPTH set to " << bitdepth << endl;
        }
        else{
            resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Blackfly Camera Response: CAMERA_PIXEL_BIT_DEPTH failed to set." << endl;
            return -1;
        }

    }
    else if(req.request() == ASLpb::Request::SET_CAMERA_NUMBER_FRAMES_TO_AVERAGE){
        cout << "Blackfly Camera Request: SET_CAMERA_NUMBER_FRAMES_TO_AVERAGE" << endl;
        unsigned int navg = req.int_param(0);
        bool rangeokay = false;
        if((navg>0) && (navg<5000)) rangeokay = true;
        if(rangeokay){
            Set_Number_Of_Frames_To_Average(navg);
            resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Blackfly Camera Response: CAMERA_NUMBER_FRAMES_TO_AVERAGE set to " << navg << endl;
        }
        else{
            resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Blackfly Camera Response: CAMERA_NUMBER_FRAMES_TO_AVERAGE failed to set. Must be in range of (1,5000) frames." << endl;
            return -1;
        }
    }
    else if(req.request() == ASLpb::Request::SET_DOT_CAMERA_POLARIZER_ANGLES){

        //Make sure device is the blackfly dot camera
        if(device_model == "Blackfly S BFS-U3-51S5M + Thorlabs ELL14K"){
            cout << "Blackfly Camera Request: SET_DOT_CAMERA_POLARIZER_ANGLES" << endl;
            QVector<double> pangles;
            bool invalid_angle = false;
            for(int k=0; k<req.double_param_size(); k++){
                if((req.double_param(k)<0) || (req.double_param(k)>180))
                    invalid_angle = true;
                pangles.append(req.double_param(k));
            }
            if(invalid_angle){//Angles invalid
                resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
                resp.set_bool_param(false);
                resp.set_status("FAIL");
                cout << "Blackfly Camera Response: DOT_CAMERA_POLARIZER_ANGLES out of range. Must be within [0,180] degrees." << endl;
                return -1;
            }
            else{//Angles are valid
                blackflyVisibleMonoDoT* temp = static_cast<blackflyVisibleMonoDoT*>(dev_ptr);
                temp->Set_Polarizer_Angles(pangles);
                resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
                resp.set_bool_param(true);
                resp.set_status("OK");
                cout << "Blackfly Camera Response: DOT_CAMERA_POLARIZER_ANGLES set to {";
                for(int k=0; k<pangles.size(); k++){
                    if(k==pangles.size()-1) cout << pangles[k] << "}" << endl;
                    else cout << pangles[k] << ", ";
                }
            }
        }
        else{
            cout << "Invalid Blackfly Camera Request Received. No action taken." << endl;
            resp.set_response(ASLpb::Response::INVALID_REQUEST);
            resp.set_status("INVALID_REQUEST");
            cout << "Blackfly Camera Response: INVALID_REQUEST. Device is not a DoT Polarimeter." << endl;
            return -1;
        }
    }
    else if(req.request() == ASLpb::Request::SET_CAMERA_GAMMA_ENABLE){
        cout << "Blackfly Camera Request: SET_CAMERA_GAMMA_ENABLE" << endl;
        double gammaon = req.bool_param();
        bool success = Set_Gamma_Enable(gammaon);
        if(success){
            resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
            resp.set_bool_param(true);
            resp.set_status("OK");
            if(gammaon)
                cout << "Blackfly Camera Response: CAMERA_GAMMA_ENABLE set to true." << endl;
            else
                cout << "Blackfly Camera Response: CAMERA_GAMMA_ENABLE set to false." << endl;
        }
        else{
            resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Blackfly Camera Response: CAMERA_GAMMA failed to set." << endl;
            return -1;
        }
    }
    else if(req.request() == ASLpb::Request::SET_CAMERA_GAMMA){
        cout << "Blackfly Camera Request: SET_CAMERA_GAMMA" << endl;
        double gamma = req.double_param(0);
        bool success = Set_Gamma(gamma);
        if(success){
            resp.set_response(ASLpb::Response::CAMERA_SETTING_APPLIED_SUCCESSFULLY);
            resp.set_bool_param(true);
            resp.set_status("OK");
            cout << "Blackfly Camera Response: CAMERA_GAMMA set to " << gamma << endl;
        }
        else{
            resp.set_response(ASLpb::Response::CAMERA_SETTING_FAILURE);
            resp.set_bool_param(false);
            resp.set_status("FAIL");
            cout << "Blackfly Camera Response: CAMERA_GAMMA failed to set." << endl;
            return -1;
        }
    }
    else{ //Invalid request for this device
        cout << "Invalid Blackfly Camera Request Received. No action taken." << endl;
        resp.set_response(ASLpb::Response::INVALID_REQUEST);
        resp.set_status("INVALID_REQUEST");
        cout << "Blackfly Camera Response: INVALID_REQUEST" << endl;
        return -1;
    }
    return 1;
}

int blackflyCamera::Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::ImageResponse &imgresp){

    string model = dev_ptr->Get_Device_Model();
    if(req.request() == ASLpb::Request::GET_CAMERA_SINGLE_IMAGE){
        if((model=="Blackfly S BFS-U3-51S5PC") || (model=="Blackfly S BFS-U3-51S5P") || (model=="Blackfly S BFS-U3-51S5M") || (model=="Blackfly S BFS-U3-51S5M + Thorlabs ELL14K")){
            cout << "Blackfly Camera Request: GET_CAMERA_SINGLE_IMAGE" << endl;

            blackflyCamera* camera = static_cast<blackflyCamera*>(dev_ptr);

            camera->Get_Single_Image();

            cout << "Blackfly Camera Response: IMAGE_ACQUIRED" << endl;
            if(model=="Blackfly S BFS-U3-51S5PC"){
                imgresp.set_camera(ASLpb::ImageResponse::RGB_VISIBLE_DOFP_POLARIMETER);
                cout << "   ->Camera Type: RGB_VISIBLE_DOFP_POLARIMETER" << endl;
            }
            else if(model=="Blackfly S BFS-U3-51S5P"){
                imgresp.set_camera(ASLpb::ImageResponse::MONO_VISIBLE_DOFP_POLARIMETER);
                cout << "   ->Camera Type: MONO_VISIBLE_DOFP_POLARIMETER" << endl;
            }
            else if(model=="Blackfly S BFS-U3-51S5M"){
                imgresp.set_camera(ASLpb::ImageResponse::MONO_VISIBLE_CAMERA);
                cout << "   ->Camera Type: MONO_VISIBLE_CAMERA " << endl;
            }
            else if(model=="Blackfly S BFS-U3-51S5M + Thorlabs ELL14K"){
                imgresp.set_camera(ASLpb::ImageResponse::MONO_VISIBLE_DOT_POLARIMETER);
                cout << "   ->Camera Type: MONO_VISIBLE_DOT_POLARIMETER " << endl;
            }
            imgresp.set_width(camera->Get_Image_Width());
            imgresp.set_height(camera->Get_Image_Height());
            cout << "   ->Image Size (WxH): " << imgresp.width() << " x " << imgresp.height() << endl;
            imgresp.set_adc_bit_depth(camera->Get_ADC_Bit_Depth());
            cout << "   ->ADC Bit Depth: " << imgresp.adc_bit_depth() << endl;
            imgresp.set_pixel_bit_depth(camera->Get_Pixel_Bit_Depth());
            cout << "   ->Pixel Bit Depth: " << imgresp.pixel_bit_depth() << endl;
            imgresp.set_channels(1);
            cout << "   ->Number of Image Channels: " << 1 << endl;
            imgresp.set_number_images_averaged(1);
            cout << "   ->Number of Images Averaged: " << 1 << endl;
            imgresp.set_exposure_time(camera->Get_Exposure_Time());
            cout << "   ->Exposure Time: " << imgresp.exposure_time() << endl;
            imgresp.set_gain(camera->Get_Gain());
            cout << "   ->Gain: " << imgresp.gain() << endl;
            imgresp.set_gamma_enable(camera->Get_Gamma_Enable());
            cout << "   ->Gamma Enabled: " << imgresp.gamma_enable() << endl;
            imgresp.set_gamma(camera->Get_Gamma());
            if(camera->Get_Gamma_Enable()){
                cout << "   ->Gamma: " << imgresp.gamma() << endl;
            }
            imgresp.set_stream_buffer_count(camera->Get_Manual_Stream_Buffer_Count());
            cout << "   ->Manual Stream Buffer Count: " << imgresp.stream_buffer_count() << endl;
            imgresp.set_timestamp(camera->Get_Timestamp(true));
            cout << "   ->Timestamp: " << imgresp.timestamp() << endl;
            imgresp.clear_dot_polarizer_angles();
            imgresp.mutable_imagedata()->Add(camera->image.begin(),camera->image.end());
            imgresp.set_success(true);
            imgresp.set_status("OK");
            cout << "   ->Status: " << imgresp.status() << endl;
        }
        //Unrecognized Device
        else{
            cout << "Can't Get Image From Unrecognized Device. No action taken." << endl;
            imgresp.set_camera(ASLpb::ImageResponse::NON_CAMERA_DEVICE);
            imgresp.set_status("INVALID_REQUEST");
            cout << "Device Response: INVALID_REQUEST for device " << dev_ptr->Get_Device_Name() << endl;
            return -1;
        }
        return 0;
    }
    else if(req.request() == ASLpb::Request::GET_CAMERA_IMAGE){
        if((model=="Blackfly S BFS-U3-51S5PC") || (model=="Blackfly S BFS-U3-51S5P") || (model=="Blackfly S BFS-U3-51S5M")){
            cout << "Blackfly Camera Request: GET_CAMERA_IMAGE" << endl;

            blackflyCamera* camera = static_cast<blackflyCamera*>(dev_ptr);

            camera->Get_Image();

            cout << "Blackfly Camera Response: IMAGE_ACQUIRED" << endl;
            if(model=="Blackfly S BFS-U3-51S5PC"){
                imgresp.set_camera(ASLpb::ImageResponse::RGB_VISIBLE_DOFP_POLARIMETER);
                cout << "   ->Camera Type: RGB_VISIBLE_DOFP_POLARIMETER" << endl;
            }
            else if(model=="Blackfly S BFS-U3-51S5P"){
                imgresp.set_camera(ASLpb::ImageResponse::MONO_VISIBLE_DOFP_POLARIMETER);
                cout << "   ->Camera Type: MONO_VISIBLE_DOFP_POLARIMETER" << endl;
            }
            else if(model=="Blackfly S BFS-U3-51S5M"){
                imgresp.set_camera(ASLpb::ImageResponse::MONO_VISIBLE_CAMERA );
                cout << "   ->Camera Type: MONO_VISIBLE_CAMERA " << endl;
            }
            imgresp.set_width(camera->Get_Image_Width());
            imgresp.set_height(camera->Get_Image_Height());
            cout << "   ->Image Size (WxH): " << imgresp.width() << " x " << imgresp.height() << endl;
            imgresp.set_adc_bit_depth(camera->Get_ADC_Bit_Depth());
            cout << "   ->ADC Bit Depth: " << imgresp.adc_bit_depth() << endl;
            imgresp.set_pixel_bit_depth(camera->Get_Pixel_Bit_Depth());
            cout << "   ->Pixel Bit Depth: " << imgresp.pixel_bit_depth() << endl;
            imgresp.set_channels(camera->Get_Number_Of_Channels());
            cout << "   ->Number of Image Channels: " << imgresp.pixel_bit_depth() << endl;
            imgresp.set_number_images_averaged(camera->Get_Number_Of_Frames_To_Average());
            cout << "   ->Number of Images Averaged: " << imgresp.number_images_averaged() << endl;
            imgresp.set_exposure_time(camera->Get_Exposure_Time());
            cout << "   ->Exposure Time: " << imgresp.exposure_time() << endl;
            imgresp.set_gain(camera->Get_Gain());
            cout << "   ->Gain: " << imgresp.gain() << endl;
            imgresp.set_gamma_enable(camera->Get_Gamma_Enable());
            cout << "   ->Gamma Enabled: " << imgresp.gamma_enable() << endl;
            imgresp.set_gamma(camera->Get_Gamma());
            if(camera->Get_Gamma_Enable()){
                cout << "   ->Gamma: " << imgresp.gamma() << endl;
            }
            imgresp.set_stream_buffer_count(camera->Get_Manual_Stream_Buffer_Count());
            cout << "   ->Manual Stream Buffer Count: " << imgresp.stream_buffer_count() << endl;
            imgresp.set_timestamp(camera->Get_Timestamp(true));
            cout << "   ->Timestamp: " << imgresp.timestamp() << endl;
            imgresp.clear_dot_polarizer_angles();
            imgresp.mutable_imagedata()->Add(camera->image.begin(),camera->image.end());
            imgresp.set_success(true);
            imgresp.set_status("OK");
            cout << "   ->Status: " << imgresp.status() << endl;
        }
        //If visible Mono DoT camera, cast to that class to call appropriate Get_Image function
        else if(model=="Blackfly S BFS-U3-51S5M + Thorlabs ELL14K"){
            cout << "Blackfly Camera Request: GET_CAMERA_IMAGE" << endl;

            blackflyVisibleMonoDoT* camera = static_cast<blackflyVisibleMonoDoT*>(dev_ptr);

            camera->Get_Image();

            cout << "Blackfly Camera Response: IMAGE_ACQUIRED" << endl;
            imgresp.set_camera(ASLpb::ImageResponse::MONO_VISIBLE_DOT_POLARIMETER);
            cout << "   ->Camera Type: MONO_VISIBLE_DOT_POLARIMETER" << endl;

            imgresp.set_width(camera->Get_Image_Width());
            imgresp.set_height(camera->Get_Image_Height());
            cout << "   ->Image Size (WxH): " << imgresp.width() << " x " << imgresp.height() << endl;
            imgresp.set_adc_bit_depth(camera->Get_ADC_Bit_Depth());
            cout << "   ->ADC Bit Depth: " << imgresp.adc_bit_depth() << endl;
            imgresp.set_pixel_bit_depth(camera->Get_Pixel_Bit_Depth());
            cout << "   ->Pixel Bit Depth: " << imgresp.pixel_bit_depth() << endl;
            imgresp.set_channels(camera->Get_Number_Of_Channels());
            cout << "   ->Number of Image Channels: " << imgresp.pixel_bit_depth() << endl;
            imgresp.set_number_images_averaged(camera->Get_Number_Of_Frames_To_Average());
            cout << "   ->Number of Images Averaged: " << imgresp.number_images_averaged() << endl;
            imgresp.set_exposure_time(camera->Get_Exposure_Time());
            cout << "   ->Exposure Time: " << imgresp.exposure_time() << endl;
            imgresp.set_gain(camera->Get_Gain());
            cout << "   ->Gain: " << imgresp.gain() << endl;
            imgresp.set_gamma_enable(camera->Get_Gamma_Enable());
            cout << "   ->Gamma Enabled: " << imgresp.gamma_enable() << endl;
            imgresp.set_gamma(camera->Get_Gamma());
            if(camera->Get_Gamma_Enable()){
                cout << "   ->Gamma: " << imgresp.gamma() << endl;
            }
            imgresp.set_stream_buffer_count(camera->Get_Manual_Stream_Buffer_Count());
            cout << "   ->Manual Stream Buffer Count: " << imgresp.stream_buffer_count() << endl;
            imgresp.set_timestamp(camera->Get_Timestamp(true));
            cout << "   ->Timestamp: " << imgresp.timestamp() << endl;
            QVector<double> pangles;
            camera->Get_Polarizer_Angles(pangles);
            cout << "   ->Polarizer Angles: {";
            for(int k=0; k<pangles.size(); k++){
                imgresp.add_dot_polarizer_angles(pangles[k]);
                if(k==pangles.size()-1) cout << pangles[k] << "}" << endl;
                else cout << pangles[k] << ", ";
            }
            imgresp.mutable_imagedata()->Add(camera->image.begin(),camera->image.end());
            imgresp.set_success(true);
            imgresp.set_status("OK");
            cout << "   ->Status: " << imgresp.status() << endl;
        }
        //Unrecognized Device
        else{
            cout << "Can't Get Image From Unrecognized Device. No action taken." << endl;
            imgresp.set_camera(ASLpb::ImageResponse::NON_CAMERA_DEVICE);
            imgresp.set_status("INVALID_REQUEST");
            cout << "Device Response: INVALID_REQUEST for device " << dev_ptr->Get_Device_Name() << endl;
            return -1;
        }
        return 0;
    }
    //Unrecognized Device
    else{
        cout << "Can't Get Image From Unrecognized Device. No action taken." << endl;
        imgresp.set_camera(ASLpb::ImageResponse::NON_CAMERA_DEVICE);
        imgresp.set_status("INVALID_REQUEST");
        cout << "Device Response: INVALID_REQUEST for device " << dev_ptr->Get_Device_Name() << endl;
        return -1;
    }
}





/***********************************************************************************
 * blackflyVisibleMonoMicrogrid Class Member Functions
 */
blackflyVisibleMonoMicrogrid::blackflyVisibleMonoMicrogrid(){

    //Set device Class Default Values
    device_name = "Blackfly Visible Mono Microgrid Camera";
    device_manufacturer = "FLIR";
    device_model = "Blackfly S BFS-U3-51S5P";
    device_type = camera;
    device_subtype = visible_mono_microgrid_polarimeter;
    device_initialized = false;
    device_detected = false;

    //Set blackflyCamera Class Values
    image_width = 2448;
    image_height = 2048;
    number_of_channels = 1;
    image_npixels = image_width*image_height;
    image.resize(image_npixels*number_of_channels);
    dimage.resize(image_npixels*number_of_channels);
    exposure_time = 1000;
    gain = 1.0;
    gamma = 1.0;
    gamma_enable = false;
    adc_bit_depth = 12;
    pixel_bit_depth = "Mono16";
    manual_stream_buffer_count = 1;
    number_of_frames_to_average = 1;
    number_of_channels = 1;
}

/***********************************************************************************
 * blackflyVisibleRGBMicrogrid Class Member Functions
 */
blackflyVisibleRGBMicrogrid::blackflyVisibleRGBMicrogrid(){

    //Set device Class Default Values
    device_name = "Blackfly Visible Mono Camera";
    device_manufacturer = "FLIR";
    device_model = "Blackfly S BFS-U3-51S5PC";
    device_type = camera;
    device_subtype = visible_color_microgrid_polarimeter;
    device_initialized = false;
    device_detected = false;

    //Set blackflyCamera Class Values
    image_width = 2448;
    image_height = 2048;
    number_of_channels = 1;
    image_npixels = image_width*image_height;
    image.resize(image_npixels*number_of_channels);
    dimage.resize(image_npixels*number_of_channels);
    exposure_time = 1000;
    gain = 1.0;
    gamma = 1.0;
    gamma_enable = false;
    adc_bit_depth = 12;
    pixel_bit_depth = "BayerRG16";
    manual_stream_buffer_count = 1;
    number_of_frames_to_average = 1;
    number_of_channels = 1;
}

/***********************************************************************************
 * blackflyVisibleMono Class Member Functions
 */
blackflyVisibleMono::blackflyVisibleMono(){
    //Set device Class Default Values
    device_name = "Blackfly Visible Mono Camera";
    device_manufacturer = "FLIR";
    device_model = "Blackfly S BFS-U3-51S5M";
    device_type = camera;
    device_subtype = visible_mono;
    device_initialized = false;
    device_detected = false;

    //Set blackflyCamera Class Values
    image_width = 2448;
    image_height = 2048;
    number_of_channels = 1;
    image_npixels = image_width*image_height;
    image.resize(image_npixels*number_of_channels);
    dimage.resize(image_npixels*number_of_channels);
    exposure_time = 1000;
    gain = 1.0;
    gamma = 1.0;
    gamma_enable = false;
    adc_bit_depth = 12;
    pixel_bit_depth = "Mono16";
    manual_stream_buffer_count = 1;
    number_of_frames_to_average = 1;
    number_of_channels = 1;
}

/***********************************************************************************
 * blackflyVisibleMonoDoT Class Member Functions
 *
 * This class inherits from the blackfly class and also incorporates control of a
 * Thorlabs ELL14K piezo rotation stage through an RS-232 connection.
 *
 * As such, it needs to override the Detection and Initialization to incorporate
 * both camera and rotation stage initialization. It also contains its own Get_Image()
 * function that will cause a sequence of images to be collected at each set polarizer
 * orientation.
 */
blackflyVisibleMonoDoT::blackflyVisibleMonoDoT(){
    //Set device Class Default Values
    device_name = "Blackfly Visible Mono DoT Camera";
    device_manufacturer = "FLIR + Thorlabs";
    device_model = "Blackfly S BFS-U3-51S5M + Thorlabs ELL14K";
    device_type = camera;
    device_subtype = visible_mono_dot_polarimeter;
    device_initialized = false;
    device_detected = false;

    //Set local class values
    polarizer_angles.clear();
    polarizer_angles.append(0);
    polarizer_angles.append(45);
    polarizer_angles.append(90);
    polarizer_angles.append(135);
    number_of_channels = polarizer_angles.size();

    //Set blackflyCamera Class Values
    image_width = 2448;
    image_height = 2048;
    image_npixels = image_width*image_height;
    image.resize(image_npixels*number_of_channels);
    dimage.resize(image_npixels*number_of_channels);
    exposure_time = 1000;
    gain = 1.0;
    gamma = 1.0;
    gamma_enable = false;
    adc_bit_depth = 12;
    pixel_bit_depth = "Mono16";
    manual_stream_buffer_count = 1;
    number_of_frames_to_average = 1;

    serial_port_name = "";
}

//Returns currently set polarizer angles in pangles
void blackflyVisibleMonoDoT::Get_Polarizer_Angles(QVector<double> &pangles){
    pangles.clear();
    for(int k=0; k<polarizer_angles.size(); k++) pangles.append(polarizer_angles[k]);
}

//Sets polarizer angles to those in pangles
void blackflyVisibleMonoDoT::Set_Polarizer_Angles(QVector<double> &pangles){
    if(pangles.size()>=1){
        polarizer_angles.clear();
        for(int k=0; k<pangles.size(); k++) polarizer_angles.append(pangles[k]);
        Set_Number_Of_Channels(pangles.size());
    }
    else{
        cout << " ->Specified polarizer angles are not valid." << endl;
    }
}

// Starts a device detection thread that will attempt to detect and then initialize
// a device.
//void blackflyVisibleMonoDoT::Start_Device_Detection(){
//    device_detection = thread(&blackflyVisibleMonoDoT::Detect_Device,this);
//    device_detection.join();
//}

// Attempts to connect to a blackfly camera and Thorlabs ELL14K piezorotation device.
// The thread will make multiple attempts to connect to both devices
void blackflyVisibleMonoDoT::Detect_Device(){

    for(unsigned int k=0; k<number_of_detection_tries; k++){
        if(!Is_Device_Detected() & !Is_Device_Initialized()){

            cout << "=========================================" << endl;
            cout << "| Attempting to Detect DoT Polarimeter: |" << endl;
            cout << "-----------------------------------------" << endl;
            cout << " ->Detection Attempt: " << ++k << " of " << number_of_detection_tries << "."<< endl << endl;

            bool camera_detected = false;
            bool ell14k_detected = false;

            cout << "Attempting to Detect Blackfly Camera Device:" << endl;
            cout << "-------------------------------------------" << endl;
            cout << " ->Getting system instance." << endl;
            this->system = System::GetInstance();
            cout << " ->Getting camera list." << endl;
            this->camList = this->system->GetCameras();
            const unsigned int numCameras = camList.GetSize();
            cout << " ->Number of cameras detected: " << numCameras << "." << endl;
            if (numCameras == 0)
            {
                camList.Clear();
                system->ReleaseInstance();
                cout << " ->No cameras detected!" << endl;
            }
            else
            {
                //Make sure that we detect the camera we are looking for
                for(unsigned int k=0; k<numCameras; k++){
                    pCam = camList.GetByIndex(k); //This picks the first detected camera in the list...
                    INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
                    CStringPtr ptrDeviceModelName = nodeMapTLDevice.GetNode("DeviceModelName");
                    string detected_model_name = ptrDeviceModelName->ToString().c_str();
                    if(detected_model_name == "Blackfly S BFS-U3-51S5M"){
                        cout << "Device " << detected_model_name << " Successfully Detected." << endl << endl;
                        camera_detected = true;
                        break;
                    }
                    else{
                        cout << " ->Incorrect camera detected: " << detected_model_name << endl << endl;
                    }
                }
            }

            //Now attempt to connect to ELL14K
            if(camera_detected){
                cout << "Attempting to Detect ELL14K Device:" << endl;
                cout << "----------------------------------" << endl;

                ell14k_detected = Connect_ELL14K();

                if(ell14k_detected==true){
                    cout << "Device ELL14K Successfully Detected." << endl << endl;
                    device_detected = true;
                }
                else{
                    cout << "Device ELL14K Not Detected." << endl << endl;
                }
            }


            //Now attempt to initialize camera and ELL14K
            if(Is_Device_Detected()){
                cout << "================================================================" << endl;
                cout << "| Attempting to Initialize Blackfly Camera and ELL14K Devices: |" << endl;
                cout << "----------------------------------------------------------------" << endl;

                cout << "Attempting to initialize DoT device..." << endl;
                pCam->Init();
                Initialize_Device();

                if(Is_Device_Initialized()){
                    cout << "DoT camera successfully initialized." << endl << endl;
                    break;
                }
                else cout << "DoT camera initialization failed." << endl << endl;
            }
        }
        // If device is detected and not initialized, keep trying to initialize the
        // device until successful.
        else if((Is_Device_Detected()) && (!Is_Device_Initialized())){
            // Re-attempt to initialize device
            cout << "Re-attempting device initialization." << endl;
            pCam->Init();
            Initialize_Device();
            if(Is_Device_Initialized()){
                cout << "Device successfully initialized." << endl << endl;
            }
            else cout << "Device initialization failed." << endl << endl;
        }
        // If somehow the device is not detected but somehow is initialized set
        // the initialization flag to false.
        else if((!Is_Device_Detected()) && (Is_Device_Initialized())){
            device_initialized = false;
        }
        // Otherwise, all is well when the device is both detected and initialized.
        else{
            break;
        }
    }

    //If number of attempts exhausted and device not detected...
    if((!Is_Device_Detected()) && (!Is_Device_Initialized())){
        cout << "Device detection timed out. Device not detected or initialized." << endl << endl;
    }

}

bool blackflyVisibleMonoDoT::Connect_ELL14K(){

// Zaber Serial Port Connection Information:
// Port: COM9
// Location: \\.\COM9
// Description: USB Serial Port
// Manufacturer: FTDI
// Serial number: A10K5TSMA
// Vendor Identifier: 403
// Product Identifier: 6001

// ELL14K Serial Port Connection Information:
// Port: COM3
// Location: \\.\COM3
// Description: USB Serial Port
// Manufacturer: FTDI
// Serial number: DT04MUFKA
// Vendor Identifier: 403
// Product Identifier: 6015

    //First, we check for all serial port devices that are available.
    //Then, we determine if the ELL14K device is one of them.
    //If successfully found, we then configure it properly.

    const auto serialPortInfos = QSerialPortInfo::availablePorts();


//        cout << "Total number of ports available: " << serialPortInfos.count() << endl;

    const QString blankString = "N/A";
    QString portname;
    QString portlocation;
    QString description;
    QString manufacturer;
    QString serialnumber;
    QString vendor;
    QString identifier;

    for (const QSerialPortInfo &serialPortInfo : serialPortInfos) {
        portname = serialPortInfo.portName();
        portlocation = serialPortInfo.systemLocation();
        description = serialPortInfo.description();
        manufacturer = serialPortInfo.manufacturer();
        serialnumber = serialPortInfo.serialNumber();
        vendor = (serialPortInfo.hasVendorIdentifier() ? QByteArray::number(serialPortInfo.vendorIdentifier(), 16) : blankString);
        identifier = (serialPortInfo.hasProductIdentifier() ? QByteArray::number(serialPortInfo.productIdentifier(), 16) : blankString);

        serial_port_name = portname.toStdString();

        //Output Information to console
        QTextStream out(stdout);
        out << endl
            << "Port: " << portname << endl
            << "Location: " << portlocation << endl
            << "Description: " << description << endl
            << "Manufacturer: " << manufacturer << endl
            << "Serial number: " << serialnumber << endl
            << "Vendor Identifier: " << vendor << endl
            << "Product Identifier: " << identifier << endl
            << "Busy: " << (serialPortInfo.isBusy() ? "Yes" : "No") << endl;

        //Determine if device is ELL14K
        //If found, configure port with ELL14K required settings
        //QSerialPort serialport;
        if((serialnumber == "DT04MUFK") && (identifier=="6015")){
            out << " ->ELL14K device found on port " << portname << "." << endl << endl;
            cout << " Serial Port Configuration:" << endl;
            cout << " -------------------------" << endl;
            serial_port.setPortName(portname);
            serial_port.setBaudRate(QSerialPort::Baud9600);
            out << " ->Baud Rate: 9600" << endl;
            serial_port.setDataBits(QSerialPort::Data8);
            out << " ->Data Bits: 8" << endl;
            serial_port.setStopBits(QSerialPort::OneStop);
            out << " ->Stop Bits: 1" << endl;
            serial_port.setParity(QSerialPort::NoParity);
            out << " ->Parity: None" << endl;
            serial_port.setFlowControl(QSerialPort::NoFlowControl);
            out << " ->Flow Control: None" << endl;
            serial_port.open(QIODevice::ReadWrite);
            out << " ->Mode: Read/Write" << endl << endl;

            if(serial_port.isOpen()){
                serial_port_open = true;
                Get_ELL14K_Device_Info();
                return true;
            }
        }
    }
    return false;
}

// This function will attempt to initialize the visible mono blackfly camera and ELL14K
// Thorlabs rotation stage. The default camera values are as follows:
//
//  Sets acquisition mode to continuous
//  Sets pixel format to 16-bit (Mono16 for mono cameras, BayerRG16 for color microgrid)
//  Sets ADC bit depth to 12
//  Autoexposure set to Off
//  Sets exposure time to 1000
//  Autogain Set to Off
//  Gain value set to 1.0
//  Gamma functionality disabled
//  StreamBufferCountMode set to manual
//  StreamBufferCountManual set to 1
void blackflyVisibleMonoDoT::Initialize_Device(){

    lock_guard<mutex> lock(this->mutex_camera_busy);
    INodeMap& nodeMap = pCam->GetNodeMap();

    // Turn Acquisition Mode to Continuous.
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) if(console_output) cout << "Unable to set acquisition mode to continuous" << endl;
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (IsAvailable(ptrAcquisitionModeContinuous) && IsReadable(ptrAcquisitionModeContinuous))
    {
        if(console_output) cout << " ->Acquisition mode set to continuous." << endl;
        const int64_t value = ptrAcquisitionModeContinuous->GetValue();
        ptrAcquisitionMode->SetIntValue(value);
    }


    // Set default Pixel Format to 16 bit
    CEnumerationPtr pixel_format_ptr = nodeMap.GetNode("PixelFormat");
    gcstring pixel_depth;
    if (this->device_model == "Blackfly S BFS-U3-51S5PC"){
        pixel_bit_depth = "BayerRG16";
        pixel_depth = "BayerRG16";
    }
    else{
        pixel_bit_depth = "Mono16";
        pixel_depth = "Mono16";
    }

    CEnumEntryPtr pixel_format_entry_ptr = pixel_format_ptr->GetEntryByName(pixel_depth);
    if (IsAvailable(pixel_format_entry_ptr) && IsReadable(pixel_format_entry_ptr)) {
        int64_t value = pixel_format_entry_ptr->GetValue();
        pixel_format_ptr->SetIntValue(value);
        if(console_output) cout << " ->Pixel format set to " << pixel_depth.c_str() << "." << endl;
    }
    else if(console_output) cout << " ->Failed to set pixel format to " << pixel_depth.c_str() << "." << endl;

    // Set Default ADC Bit Depth to 12 (other options are Bit8 and Bit10)
    CEnumerationPtr adc_ptr = nodeMap.GetNode("AdcBitDepth");
    CEnumEntryPtr adc_entry_ptr = adc_ptr->GetEntryByName("Bit12");
    if (IsAvailable(adc_entry_ptr) && IsReadable(adc_entry_ptr))
    {
        int64_t value = adc_entry_ptr->GetValue();
        adc_ptr->SetIntValue(value);
    }


    // Turn Autoexposure off and set the initial value.
    CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
    if (IsAvailable(ptrExposureAuto) || IsWritable(ptrExposureAuto))
    {
        CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
        if (IsAvailable(ptrExposureAutoOff) || IsWritable(ptrExposureAutoOff)) ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
        else if(console_output) cout << " ->Automatic exposure successfully disabled..." << endl;
    }
    else if(console_output) cout << " ->Unable to disable automatic exposure (node retrieval). Aborting..." << endl;

    // Set Exposure Time
    CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
    if (IsAvailable(ptrExposureTime) || IsWritable(ptrExposureTime))
    {
        // Ensure desired exposure time does not exceed the maximum
        const double exposureTimeMax = ptrExposureTime->GetMax();
        double exposureTimeToSet = exposure_time;
        if (exposureTimeToSet > exposureTimeMax)
        {
            exposureTimeToSet = exposureTimeMax;
            if(console_output) cout << " ->Exposure time set to " << exposure_time << "." << endl;
        }
        ptrExposureTime->SetValue(exposureTimeToSet);
    }
    else if(console_output) cout << " ->Unable to set exposure time." << endl;

    // Turn AutoGain off
    CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
    CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
    if (IsAvailable(ptrGainAutoOff) || IsWritable(ptrGainAutoOff))
    {
        ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());
        if(console_output) cout << " ->Gain set to manual." << endl;
    }
    else if(console_output) cout << " ->Unable to disable automatic gain." << endl;

    //Set Initial Gain Value
    CFloatPtr ptrGain = nodeMap.GetNode("Gain");
    if (IsAvailable(ptrGain) || IsWritable(ptrGain))
    {
        // Ensure desired exposure time does not exceed the maximum
        const double gainMax = ptrGain->GetMax();
        double gainToSet = gain;
        if (gainToSet > gainMax) gainToSet = gainMax;
        if(console_output) cout << " ->Gain set to " << gain << "." << endl;
        ptrGain->SetValue(gainToSet);
    }
    else if(console_output) cout << " ->Unable to set gain." << endl;

    // Set gamma value
    CFloatPtr ptrGamma = nodeMap.GetNode("Gamma");
    if (IsAvailable(ptrGamma) || IsWritable(ptrGamma))
    {
        // Ensure desired exposure time does not exceed the maximum
        const double gammaMax = ptrGamma->GetMax();
        double gammaToSet = gamma;
        if (gammaToSet > gammaMax) gammaToSet = gammaMax;
        if(console_output) cout << " ->Gamma set to " << gamma << "." << endl;
        ptrGamma->SetValue(gammaToSet);
    }
    else if(console_output) cout << " ->Unable to set gamma." << endl;

    // Turn off gamma
    CBooleanPtr ptrGammaEnable = nodeMap.GetNode("GammaEnable");
    if (IsAvailable(ptrGammaEnable) || IsWritable(ptrGammaEnable))
    {
        ptrGammaEnable->SetValue(false);
        if(console_output) cout << " ->Gamma disabled." << endl;
    }
    else if(console_output) cout << " ->Unable to disable gamma." << endl;

    // Turn StreamBufferCountMode to manual
    INodeMap& nodeMapStream = pCam->GetTLStreamNodeMap();
    CEnumerationPtr ptrStream = nodeMapStream.GetNode("StreamBufferCountMode");
    CEnumEntryPtr ptrStreamManual = ptrStream->GetEntryByName("Manual");
    if (IsAvailable(ptrStreamManual) || IsWritable(ptrStreamManual))
    {
        ptrStream->SetIntValue(ptrStreamManual->GetValue());
        if(console_output) cout << " ->Stream mode set to manual." << endl;
    }
    else if(console_output) cout << " ->Unable to set stream mode to manual." << endl;

    // Set StreamBufferCountManual to 1
    CIntegerPtr ptrCount = nodeMapStream.GetNode("StreamBufferCountManual");
    if (IsAvailable(ptrCount) || IsWritable(ptrCount))
    {
        ptrCount->SetValue(manual_stream_buffer_count);
        if(console_output) cout << " ->Manual stream buffer count set to " << manual_stream_buffer_count << "." << endl;
    }
    else if(console_output) cout << " ->Unable to set stream buffer count." << endl;


    //Home ELL14K Device
    if(console_output) cout << " ->Homing ELL14K device." << endl;
    Home_Device(true);
    Get_Angular_Position();

    device_initialized = true;
}

bool blackflyVisibleMonoDoT::Is_Device_Busy(){
    return device_busy;
}

//Acquires an image from the DoT camera. This function will iterate through the specified
// N polarizer angles and acquire an averaged image frame at each polarizer orientation.
// The output is stored as N channel images of size width x height.
bool blackflyVisibleMonoDoT::Get_Image(){

    if(Is_Device_Detected() & Is_Device_Initialized()){

        if(console_output) cout << "Acquiring DoT Image at " << polarizer_angles.length() << " polarizer orientations:" << endl;
        if(console_output) cout << "-----------------------------------------------" << endl;

        //Iterate through each polarizer orientation and acquire image
        image.resize(image_npixels*number_of_channels);
        for(int k=0; k<polarizer_angles.length(); k++){

            //Rotate to next polarizer angle
            Set_Angular_Position(polarizer_angles[k]);
            Get_Angular_Position();

            //Begin Image Acquisition
            try {
                pCam->BeginAcquisition();
    //            mutex_camera_busy.lock();
    //            mutex_camera_busy.unlock();
                dimage.fill(0);
                for (unsigned int imageCnt = 0; imageCnt < number_of_frames_to_average; imageCnt++)
                {
                    try {
                        acquiredFrame  = this->pCam->GetNextImage(1000);
                        uint16_t* raw_data = (uint16_t*)acquiredFrame->GetData();

                        if (imageCnt == 0)
                        {
                            if(console_output) cout << " ->Pixel format: " << acquiredFrame->GetPixelFormatName() << endl;
                            if(console_output) cout << " ->Size of raw data: " << acquiredFrame->GetBufferSize() << endl;
                            if(console_output) cout << " ->Bits per pixel: " << acquiredFrame->GetBitsPerPixel() << endl;
                            if(console_output) cout << " ->Number of channels: " << acquiredFrame->GetNumChannels() << endl;
                            if(console_output) cout << " ->Number of images averaged: " << number_of_frames_to_average << endl;
                            if(console_output) cout << " ->Timestamp: " << Get_Timestamp(false) << endl << endl;
                        }

                        // Average frames
                        for (unsigned int n=0;n<image_npixels;n++){
                            dimage[image_npixels*k + n] = dimage[image_npixels*k + n]+double(raw_data[n])/number_of_frames_to_average;
                        }
                        acquiredFrame->Release();
                    }
                    catch (Spinnaker::Exception& e){
                        if(console_output) cout << "Error: " << e.what() << endl;
                    }
                }
                this->pCam->EndAcquisition();

                // Convert averaged frame to 32-bit format.
                for (unsigned int n=0;n<image_npixels;n++){
                    image[image_npixels*k + n] = uint32_t(dimage[image_npixels*k + n]);
                }
            }
            catch (Spinnaker::Exception& e)
            {
                if(console_output) cout << "Error: " << e.what() << endl;
                return false;
            }
        }
    }
    else{
        if(!Is_Device_Detected()){
            if(console_output) cout << "Unable to acquire image. Device not detected." << endl;
        }
        else{ if(!Is_Device_Initialized())
            if(console_output) cout << "Unable to acquire image. Device not initialized." << endl;}
        return false;
    }
    return true;
}

QByteArray blackflyVisibleMonoDoT::Send_Receive_Command(const QByteArray tx){

    if(Is_Serial_Port_Open()){
        const qint64 bytesWritten = serial_port.write(tx);

        if (bytesWritten == -1) {
            if(console_output) cout << "Failed to send command to ELL14K on port " << Get_Serial_Port_Name() << "." << endl;
        }
        else if (bytesWritten != tx.size()) {
            if(console_output) cout << "Failed to send complete command to ELL14K on port " << Get_Serial_Port_Name() << "." << endl;
        }
        else if (!serial_port.waitForBytesWritten(5000)) {
            if(console_output) cout << "Command sent to ELL14K on port " << Get_Serial_Port_Name() << " timed out or an error occurred." << endl;
        }

//        if(console_output) cout << "  ELL14K->Tx: " << tx.toStdString() << endl;

        bool got_data = false;
        QByteArray readData = serial_port.readAll();
        while(!got_data){
            while (serial_port.waitForReadyRead(100))
                readData.append(serial_port.readAll());
            if(readData.size()>0) got_data=true;
        }
//        if(console_output) cout << "  ELL14K->Rx: " << readData.toStdString() << endl;

        return readData;
    }
    else{
        cout << "Command not sent: ELL14K device not connected." << endl;
        return "";
    }

}

//Extract substrings from a QByteArray. Useful for decoding ELL14K response commands.
QByteArray blackflyVisibleMonoDoT::getSubstring(const QByteArray qstring, size_t start, size_t end){
    if((start<0) || (start>=qstring.size()) || (start>end)) return "";
    else if((end<0) || (end>=qstring.size()) || (end<start)) return "";
    else{
        QByteArray sub = "";
        for(int k=start; k<=end; k++) sub.append(qstring[k]);
        return sub;
    }
}

//Converts an integer to a hexidecimal string
string blackflyVisibleMonoDoT::Int_To_Hex_String(int num){
    stringstream stream;
    stream << std::setfill ('0') << std::setw(sizeof(num)*2)
           << std::hex << num;
    string hex = stream.str();
    //convert any hex characters from lower to upper case
    for(int k=0; k<hex.length(); k++){
        if(hex[k] == 'a') hex[k] = 'A';
        else if(hex[k] == 'b') hex[k] = 'B';
        else if(hex[k] == 'c') hex[k] = 'C';
        else if(hex[k] == 'd') hex[k] = 'D';
        else if(hex[k] == 'e') hex[k] = 'E';
        else if(hex[k] == 'f') hex[k] = 'F';
    }
    return hex;
}


//Will request device, motor 1, and motor 2 status info from the device
//and output to console.
bool blackflyVisibleMonoDoT::Get_ELL14K_Device_Info(){

    //Get basic define information
    QByteArray rx0 = Send_Receive_Command("0in");

    //Extract info from response
    QString dv = getSubstring(rx0,0,0);
    QString cm = getSubstring(rx0,1,2);
    QString el = getSubstring(rx0,3,4);
    QString sn = getSubstring(rx0,5,12);
    QString yr = getSubstring(rx0,13,16);
    QString fw = getSubstring(rx0,17,18);
    char th = rx0[19];
    bool threads = (th & (1 << 7));
    QString hw = getSubstring(rx0,20,20);
    QString tv = getSubstring(rx0,21,24);
    QString pl = getSubstring(rx0,25,32);

    //Decode response data and output to terminal
    bool ok;
    cout << " ELL14K Device Info:" << endl;
    cout << " ------------------" << endl;
    cout << " ->Device Address: " << dv.toStdString() << endl;
    cout << " ->Command: " << cm.toStdString() << endl;
    cout << " ->ELL Model: " << el.toUInt(&ok,16) << endl;//el.toStdString() << endl;
    cout << " ->Serial Number: " << sn.toStdString() << endl;
    cout << " ->Year Manufactured: " << yr.toStdString() << endl;
    cout << " ->Firmware: " << (float)fw.toUInt(&ok,10)/10.0 << endl;
    cout << " ->Threads: " << (threads ? "Imperial" : "Metric") << endl;
    cout << " ->Hardware Release: " << hw.toUInt(&ok,16) << endl;
    cout << " ->Device Travel: " << tv.toUInt(&ok,16) << endl;
    cout << " ->Encoder Pulses Per Revolution: " << pl.toUInt(&ok,16) << endl;
    encoder_pulses_per_degree = (double)pl.toUInt(&ok,16) / (double)tv.toUInt(&ok,16);
    cout << " ->Encoder Pulses Per Degree: " << encoder_pulses_per_degree << endl << endl;

    //Get motor 1 information
    QByteArray rx1 = Send_Receive_Command("0i1");

    QString loop1 = getSubstring(rx1,3,3);
    QString motor1 = getSubstring(rx1,4,4);
    QString current1 = getSubstring(rx1,5,8);
    QString rampup1 = getSubstring(rx1,9,12);
    QString rampdown1 = getSubstring(rx1,13,16);
    QString fperiod1 = getSubstring(rx1,17,20);
    QString bperiod1 = getSubstring(rx1,21,24);
    bool lp1 = false;
    if(loop1.toUInt(&ok,10)==1) lp1 = true;
    bool mt1 = false;
    if(motor1.toUInt(&ok,10)==1) mt1 = true;

    cout << " ELL14K Motor 1 Info:" << endl;
    cout << " -------------------" << endl;
    cout << " ->Loop: " << (lp1 ? "On" : "Off") << endl;
    cout << " ->Motor: " << (mt1 ? "On" : "Off") << endl;
    cout << " ->Current: " << (float)current1.toUInt(&ok,16)/1866.0 << " A" << endl;
    cout << " ->Ramp Up: " << rampup1.toUInt(&ok,16) << endl;
    cout << " ->Ramp Down: " << rampdown1.toUInt(&ok,16) << endl;
    cout << " ->Forward Period: " << 14740/(float)fperiod1.toUInt(&ok,16) << " kHz" << endl;
    cout << " ->Backward Period: " << 14740/(float)bperiod1.toUInt(&ok,16) << " kHz" << endl << endl;

    //Get motor 2 information
    QByteArray rx2 = Send_Receive_Command("0i2");

    QString loop2 = getSubstring(rx2,3,3);
    QString motor2 = getSubstring(rx2,4,4);
    QString current2 = getSubstring(rx2,5,8);
    QString rampup2 = getSubstring(rx2,9,12);
    QString rampdown2 = getSubstring(rx2,13,16);
    QString fperiod2 = getSubstring(rx2,17,20);
    QString bperiod2 = getSubstring(rx2,21,24);
    bool lp2 = false;
    if(loop2.toUInt(&ok,10)==1) lp2 = true;
    bool mt2 = false;
    if(motor2.toUInt(&ok,10)==1) mt2 = true;

    cout << " ELL14K Motor 2 Info:" << endl;
    cout << " -------------------" << endl;
    cout << " ->Loop: " << (lp2 ? "On" : "Off") << endl;
    cout << " ->Motor: " << (mt2 ? "On" : "Off") << endl;
    cout << " ->Current: " << (float)current2.toUInt(&ok,16)/1866.0 << " A" << endl;
    cout << " ->Ramp Up: " << rampup2.toUInt(&ok,16) << endl;
    cout << " ->Ramp Down: " << rampdown2.toUInt(&ok,16) << endl;
    cout << " ->Forward Period: " << 14740/(float)fperiod2.toUInt(&ok,16) << " kHz" << endl;
    cout << " ->Backward Period: " << 14740/(float)bperiod2.toUInt(&ok,16) << " kHz" << endl;

    return true;
}

//Request status from ELL14K device
unsigned int blackflyVisibleMonoDoT::Get_ELL14K_Status(){
    QByteArray rx = Send_Receive_Command("gs");
    bool ok;
    QString status = getSubstring(rx,3,4);
    return status.toUInt(&ok,16);
}

//Translate status code from ELL14K to text string message.
string Get_ELL14K_Status_String(unsigned int status){
    string message = "";
    if(status==0) message = "OK, no error.";
    else if(status==1) message = "Communication time out.";
    else if(status==2) message = "Mechanical time out.";
    else if(status==3) message = "Command error or not supported.";
    else if(status==4) message = "Value out of range.";
    else if(status==5) message = "Module isolated.";
    else if(status==6) message = "Module out of isolation.";
    else if(status==7) message = "Initializing error.";
    else if(status==8) message = "Thermal error.";
    else if(status==9) message = "Busy.";
    else if(status==10) message = "Sensor Error (May appear during self test. If code persists there is an error).";
    else if(status==11) message = "Motor Error (May appear during self test. If code persists there is an error).";
    else if(status==12) message = "Out of Range (e.g. stage has been instructed to move beyond its travel range).";
    else if(status==13) message = "Over Current error.";
    else message = "14-255 Reserved.";

    return message;
}

//Sends home command to ELL14K device
bool blackflyVisibleMonoDoT::Home_Device(bool dir_CCW){

    //Home device
    QByteArray rx = "";
    if(dir_CCW) rx = Send_Receive_Command("0ho1");
    else rx = Send_Receive_Command("0ho0");

    //Response can either be PO, indicated move is complete
    // or GS, indicating a get status message that the move is
    // still in process. If GS message, keep polling the device
    // until status is not busy.
    QString cm = getSubstring(rx,2,3);
    if(cm == "GS"){
        bool ok;
        QString s = getSubstring(rx,3,4);
        unsigned int status = s.toUInt(&ok,16);
        while(status != 0){
            status = Get_ELL14K_Status();
        }
    }
    return true;
}

//Send command to ELL14K to move to a specific angular position relative to 0 in degrees
bool blackflyVisibleMonoDoT::Set_Angular_Position(double degrees){

    //convert degrees to number of encoder pulses
    string pos = "0ma" + Int_To_Hex_String((int)(degrees*encoder_pulses_per_degree));
    QString ppos = QString::fromStdString(pos);
    QByteArray tx = ""; tx += ppos;

    //Send absolute position command
    QByteArray rx = Send_Receive_Command(tx);

    //Response can either be PO, indicated move is complete
    // or GS, indicating a get status message that the move is
    // still in process. If GS message, keep polling the device
    // until status is not busy.
    QString cm = getSubstring(rx,2,3);
    if(cm == "GS"){
        bool ok;
        QString s = getSubstring(rx,3,4);
        unsigned int status = s.toUInt(&ok,16);
        while(status != 0){
            status = Get_ELL14K_Status();
        }
    }
    return true;
}

//Send command to ELL14K to move to a specific relative angular position in degrees
bool blackflyVisibleMonoDoT::Set_Relative_Angular_Position(double degrees){

    //convert degrees to number of encoder pulses
    string pos = "0mr" + Int_To_Hex_String((int)(degrees*encoder_pulses_per_degree));
    QString ppos = QString::fromStdString(pos);
    QByteArray tx = ""; tx += ppos;

    //Send relative position command
    QByteArray rx = Send_Receive_Command(tx);

    //Response can either be PO, indicated move is complete
    // or GS, indicating a get status message that the move is
    // still in process. If GS message, keep polling the device
    // until status is not busy.
    QString cm = getSubstring(rx,2,3);
    if(cm == "GS"){
        bool ok;
        QString s = getSubstring(rx,3,4);
        unsigned int status = s.toUInt(&ok,16);
        while(status != 0){
            status = Get_ELL14K_Status();
        }
    }
    return true;
}

double blackflyVisibleMonoDoT::Get_Angular_Position(){

    //Get angular position in degrees
    QByteArray rx = Send_Receive_Command("0gp");

    bool ok;
    QString p = getSubstring(rx,3,10);
    double pos = (double)((int)p.toUInt(&ok,16))/encoder_pulses_per_degree;
    cout << " ->ELL14K Position: " << pos << " degrees." << endl;;

    return pos;
}











/***********************************************************************************
 * deviceServer Class Member Functions
 */


//Constructor using default ports
deviceServer::deviceServer(device *dev_ptr, string name)
{
    Set_Server_Name(name);
    dev = dev_ptr;
    Bind();

    cout << "Device server configured:" << endl;
    cout << "------------------------" << endl;
    cout << " ->Server name: " << Get_Server_Name() << endl;
    cout << " ->Response port: " << Get_Response_Port() << endl;
    cout << " ->Monitoring Device: " << dev_ptr->Get_Device_Name() << endl << endl;
}

//Constructor allowing for custom ports
deviceServer::deviceServer(device *dev_ptr, string name, string res_port)//, string img_res_port)
{
    Set_Server_Name(name);
    Set_Response_Port(res_port);
    dev = dev_ptr;
    Bind();

    cout << "Device Server configured:" << endl;
    cout << "------------------------" << endl;
    cout << " ->Server name: " << Get_Server_Name() << endl;
    cout << " ->Response port: " << Get_Response_Port() << endl;
}

//default destructor
deviceServer::~deviceServer(){}

//binds currently specified ports to corresponding sockets
void deviceServer::Bind()
{
    this->response_socket.bind("tcp://*:" + Get_Response_Port());
}


void deviceServer::Start_Task_Manager()
{
    cout << "========================================" << endl;
    cout << "| Starting Device Server Task Manager: |" << endl;
    cout << "----------------------------------------" << endl << endl;
    task_manager = thread(&deviceServer::Task_Manager,this);
    task_manager.join();
    cout << "Ending Device Server Task Manager." << endl;
}


void deviceServer::Task_Manager()
{

    //Cast device into appropriate subclass to ensure that proper Send_Request function is called.
    string model = dev->Get_Device_Model();
    device* dev_subclass = NULL;

    //Zaber Rotary Device
    if(model == "X-RST120AK-E03"){
        dev_subclass = static_cast<zaberRotationStage*>(dev);
    }
    //Blackfly Camera
    else if(model.substr(0,8) == "Blackfly"){
        dev_subclass = static_cast<blackflyCamera*>(dev);
    }
    //Unrecognized Device
    else{
        cout << "Task Manager started with invalid device pointer. Task Manager ending." << endl;
        return;
    }

    cout << "Listening for client requests:" << endl;
    cout << "-----------------------------" << endl << endl;

    for (;;) {

        //Receive Message and Parse into String
        zmq::message_t request;
        response_socket.recv(request,zmq::recv_flags::none);
        string msg_str(static_cast<char*>(request.data()),request.size());
        this->mutex_request.lock();
        this->req.ParseFromString(msg_str);

        string text_str;
        google::protobuf::TextFormat::PrintToString(this->req,&text_str);
        ASLpb::Request device_request = this->req;
        this->mutex_request.unlock();
        cout << text_str << endl;


        //Handle any server specific requests
        //Otherwise, forward request to device, receive Device response, and
        //compose response message to client
        if(req.request() == ASLpb::Request::GET_SERVER_NAME){
            ASLpb::Response reply;
            cout << "Server Request: GET_SERVER_NAME" << endl;
            reply.set_response(ASLpb::Response::SERVER_NAME);
            reply.clear_double_param();
            reply.clear_int_param();
            reply.set_string_param(Get_Server_Name());
            reply.clear_bool_param();
            reply.set_status("OK");
            cout << "Server Response: SERVER_NAME = " << Get_Server_Name() << endl;

            //Generate serialized reply string and send Tx to client
            string reply_str;
            reply.SerializeToString(&reply_str);
            response_socket.send(zmq::buffer(reply_str));
        }
        //If request is to collect an image, we expect an ImageResponse
        else if((device_request.request() == ASLpb::Request::GET_CAMERA_IMAGE) || (device_request.request() == ASLpb::Request::GET_CAMERA_SINGLE_IMAGE)){

            //Pass image response string to approrpiate device Send_Request function
            ASLpb::ImageResponse reply;            

            dev_subclass->Send_Request(dev_subclass,device_request,reply);

            //Generate serialized reply string and send Tx to client
            string reply_str;
            reply.SerializeToString(&reply_str);
            response_socket.send(zmq::buffer(reply_str));
        }
        //Otherwise we expect a Response
        else {

            //Pass reply stirng to approrpiate device Send_Request function
            ASLpb::Response reply;

            dev_subclass->Send_Request(dev_subclass,device_request,reply);

            //Generate serialized reply string and send Tx to client
            string reply_str;
            reply.SerializeToString(&reply_str);
            response_socket.send(zmq::buffer(reply_str));
        }
    }
    this->Set_Return_Flag(true);
}
