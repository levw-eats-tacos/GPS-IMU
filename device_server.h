/* Note that this class requires that libraries be accessible and properly linked for
 *     zeroMQ
 *     Google Protocol Buffers
 *          ->NOTE: Had to revert to version used by Zaber library: 3.17.03
 *     FLIR Spinnaker Software
 *     Zaber Motion Control Libraries
 *     Polaris Pyxis SDK
 */

/* NOTES:
 *
 * 1. Need to ensure that the zmq dll is in the execution directory for Windows.
 *
 * 2. Need to make sure that the server_config.txt file exists and is in the
 *    execution directory.
 *
 * 3. Need to ensure that google protobuf version matches that of the Zaber Motion
 *    Control library (currently 3.17.03). Had to download the corresponding version
 *    of protoc and compile the libraries for that version.
 */


/* THINGS TO DO:
 *
 * 1. Figure out why device detection loops don't work for Blackfly devices
 *    when camera is not connected.
 *
 * 2. Determine if I need threading for image acquisition. If so, need to
 *    figure out any mutex stuff.
 *
 * 3. Need to write the Is_Device_Busy() function for the Blackfly and DoT class.
 *
 * 4. Need to write Set_Pixel_Bit_Depth() function for the Blackfly class.
*/

#ifndef DEVICE_SERVER_H
#define DEVICE_SERVER_H

#pragma once
#include <string>
#include <thread>
#include <mutex>
#include <future>
#include <QVector>
#include <zmq.hpp>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
#include "ASL.pb.h"
#include <google/protobuf/text_format.h>

//Device specific libraries
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <zaber/motion/ascii.h>
//#endif // DEVICE_SERVER_H
using namespace std;
using namespace chrono;
using namespace std::this_thread;     // sleep_for, sleep_until
//using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace zaber::motion;
using namespace zaber::motion::ascii;


/***********************************************************************************
 *
 * Device enumeration declarations
 */

//NOTE: Changes to this enum type should be reflected in the
// device.Get_Device_Type_As_String() function.
enum devicetype {
    no_device_type=-1,
    camera=0,
    motion_control_stage=1,
    calibration_source=2
};

//NOTE: Changes to this enum type should be reflected in the
// device.Get_Device_Subtype_As_String() function.
enum devicesubtype{

    //camera sub types
    no_device_subtype=-1,
    visible_mono=0,
    visible_color=1,
    visible_mono_dot_polarimeter=2,
    visible_mono_microgrid_polarimeter=3,
    visible_color_microgrid_polarimeter=4,

    //motion control stage sub types
    rotary_stage=20,
    translation_stage=21,
    robotic_arm=22,

    //calibration source sub types
    integrating_sphere=40,
    blackbody=41
};






/***********************************************************************************
 * device Class
 *
 * This superclass defines a base class that all specialized device classes will
 * inherit from.
 */
class device {
protected:
    string device_name;
    string device_manufacturer;
    string device_model;
    enum devicetype device_type;
    enum devicesubtype device_subtype;
    bool device_detected=false;
    bool device_initialized=false;
    bool device_busy=false;
    unsigned int number_of_detection_tries = 20;
    double redetection_wait_time = 1; //second

    bool console_output = false;
public:
    device();

    //Set and Get Functions
    void Set_Device_Name(string name) {device_name = name;}
    void Set_Device_Manufacturer(string manu) {device_manufacturer = manu;}
    void Set_Device_Model(string model) {device_model = model;}
    void Set_Device_Type(devicetype type) {device_type = type;}
    void Set_Device_Subtype(devicesubtype subtype) {device_subtype = subtype;}
    void Set_Number_Of_Detection_Tries(unsigned int num_tries) {number_of_detection_tries = num_tries;}
    void Set_Redetection_Wait_Time(double wait_time) {redetection_wait_time = wait_time;}

    string Get_Device_Name() {return device_name;}
    string Get_Device_Manufacturer() {return device_manufacturer;}
    string Get_Device_Model() {return device_model;}
    devicetype Get_Device_Type() {return device_type;}    
    devicesubtype Get_Device_Subtype() {return device_subtype;}
    string Get_Device_Type_As_String();
    string Get_Device_Subtype_As_String();
    unsigned int Get_Number_Of_Detection_Tries() {return number_of_detection_tries;}
    double Get_Redetection_Wait_Time() {return redetection_wait_time;}
    bool Is_Device_Detected(){ return device_detected;}
    bool Is_Device_Initialized(){ return device_initialized;}

    //Other Useful Functions
    string Get_Timestamp(bool filename_safe);

    //==============>Functions that should be overloaded in inherited devices<==
    virtual void Detect_Device();
    virtual void Initialize_Device();
    virtual bool Is_Device_Busy(){return device_busy;}

    //Functions for sending TCP requests to a given device
    virtual int Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::Response &resp);
    virtual int Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::ImageResponse &imgresp);

};

/***********************************************************************************
 * zaberRotationStage Class
 *
 * This is a device class for controlling a Zaber high precision rotation stage. It
 * contains all relevant functionality for controlling the rotation stage and is
 * based upon Zaber's motion controll ASCII library. This class inherits from the
 * base device class.
 *
 * Tasks To Do:
 *      -Figure out how to properly do a time-out within the Detect_Device() function
 *       when attempting to re-detect device
 */
class zaberRotationStage : public device {

private:
    string serial_port_name;
    Connection connection;
    vector<Device> deviceList;
    Device *rotation_stage_device;
    Axis *rotation_stage_axis;

    bool turntable_rotation_enabled = false;

public:
    zaberRotationStage();
    bool Detect_Zaber_Serial_Port();
    void Set_Serial_Port(string serialPort);

    //Rotation Stage Commands
    bool Home_Device();
    bool Stop_Device();
    bool Set_Angular_Position(double degrees);
    bool Set_Relative_Angular_Position(double degrees);
    bool Rotate_At_Constant_Velocity(double velocity);

    //Device Queries
    double Get_Angular_Position();
    double Get_Maximum_Angular_Velocity();
    bool Is_Device_Rotating_At_Constant_Velocity(){return turntable_rotation_enabled;}

    //Overloaded functions
    void Detect_Device() override;
    void Initialize_Device() override;
    bool Is_Device_Busy() override;

    //Functions for sending TCP requests to a given device
    int Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::Response &resp) override;
    int Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::ImageResponse &imgresp) override;
};

/***********************************************************************************
 * blackflyCamera Class
 *
 * This is a general class for a FLIR Blackfly camera. This class contains all basic
 * functionality for communicating with a Blackfly camera. This device inherits from
 * the base device class. It is then intended to be further inherited by specific
 * Blackfly camera model classes that will allow for default camera settings to be
 * configured, such as the mono microgrid, mono DoT camera, and RGB microgrid.
 */
class blackflyCamera : public device {

protected:

    //Spinnaker variables
    SystemPtr system;
    CameraPtr pCam = nullptr;
    CameraList camList;
    ImagePtr acquiredFrame;

    //local camera settings variables
    unsigned int image_width;
    unsigned int image_height;
    double exposure_time;
    double gain;
    double gamma;
    bool gamma_enable;
    unsigned int adc_bit_depth;
    string pixel_bit_depth;
    unsigned int number_of_channels;
    unsigned int manual_stream_buffer_count;
    unsigned int number_of_frames_to_average;
    mutex mutex_camera_busy;

    //Local image variables
    unsigned int image_npixels; //set by image_width, image_height
    QVector<double> dimage;    //of size image_width x image_height x number_of_channels

public:

    blackflyCamera();

    //Latest acquired image averaged and converted to unsigned 16-bit image
    QVector<uint32_t> image;

    //Camera Settings
    void Set_Image_Width(unsigned int w);
    void Set_Image_Height(unsigned int h);
    bool Set_Exposure_Time(double et);
    bool Set_Gain(double g);
    bool Set_Gamma(double g);
    bool Set_Gamma_Enable(bool enable);
    bool Set_ADC_Bit_Depth(unsigned int adc);
    bool Set_Pixel_Bit_Depth(string pbd);
    bool Set_Manual_Stream_Buffer_Count(unsigned int bc);
    void Set_Number_Of_Frames_To_Average(unsigned int num_frames_avg);
    void Set_Number_Of_Channels(unsigned int num_channels);

    unsigned int Get_Image_Width(){ return image_width;}
    unsigned int Get_Image_Height(){ return image_height;}
    double Get_Exposure_Time(){ return exposure_time;}
    double Get_Gain(){ return gain;}
    double Get_Gamma(){ return gamma;}
    bool Get_Gamma_Enable(){ return gamma_enable;}
    unsigned int Get_ADC_Bit_Depth(){ return adc_bit_depth;}
    string Get_Pixel_Bit_Depth(){ return pixel_bit_depth;}
    unsigned int Get_Manual_Stream_Buffer_Count(){ return manual_stream_buffer_count;}
    unsigned int Get_Number_Of_Frames_To_Average(){ return number_of_frames_to_average;}
    unsigned int Get_Number_Of_Channels(){return number_of_channels;}

    //Camera Functionality
    virtual bool Get_Image();
    bool Get_Single_Image();

    //Overloaded Functions
    void Detect_Device() override;
    void Initialize_Device() override;
    bool Is_Device_Busy() override;

    //Functions for sending TCP requests to a given device
    int Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::Response &resp) override;
    int Send_Request(device* dev_ptr, ASLpb::Request req, ASLpb::ImageResponse &imgresp) override;
};


/***********************************************************************************
 * blackflyVisibleMonoMicrogrid Class
 *
 * This is a device class for the FLIR Blackfly Visible Monochromatic Microgrid
 * camera. This device inherits from the base device class and is meant to be a simple
 * class that simply sets the correct device parameters for the FLIR Blackfly Visible
 * Monochromatic Microgrid Camera (Blackfly S BFS-U3-51S5P).
 */
class blackflyVisibleMonoMicrogrid : public blackflyCamera {

public:
    blackflyVisibleMonoMicrogrid();
};


/***********************************************************************************
 * blackflyVisibleRGBMicrogrid Class
 *
 * This is a device class for the FLIR Blackfly Visible RGB (non-polarized)
 * camera. This device inherits from the base device class and is meant to be a simple
 * class that simply sets the correct device parameters for the FLIR Blackfly Visible
 * Monochromatic Microgrid Camera (Blackfly S BFS-U3-51S5M).
 */
class blackflyVisibleRGBMicrogrid : public blackflyCamera {

public:
    blackflyVisibleRGBMicrogrid();
};


/***********************************************************************************
 * blackflyVisibleMono Class
 *
 * This is a device class for the FLIR Blackfly Visible Monochromatic (non-polarized)
 * camera. This device inherits from the base device class and is meant to be a simple
 * class that simply sets the correct device parameters for the FLIR Blackfly Visible
 * Monochromatic Microgrid Camera (Blackfly S BFS-U3-51S5M).
 */
class blackflyVisibleMono : public blackflyCamera {

public:
    blackflyVisibleMono();
};


/***********************************************************************************
 * blackflyVisibleMonoDoT Class
 *
 * This is a device class for the FLIR Blackfly Visible Monochromatic Division of
 * Time (DoT) polarimeter camera that makes use of the Thor Labs ELL14K rotation
 * stage. This device inherits from the base device class and is meant to be a
 * simple class that simply sets the correct device parameters for the FLIR Blackfly
 * Visible Monochromatic Microgrid Camera (Blackfly S BFS-U3-51S5M).
 */
class blackflyVisibleMonoDoT : public blackflyCamera {
private:
    QVector<double> polarizer_angles;
    QSerialPort serial_port;
    string serial_port_name;
    bool serial_port_open=false;
    double encoder_pulses_per_degree = 143360/360.0;

    //Serial Port Commands
    QByteArray Send_Receive_Command(const QByteArray tx);

    //Helper functions
    QByteArray getSubstring(const QByteArray qstring,size_t start, size_t end);
    string Int_To_Hex_String(int num);

public:
    blackflyVisibleMonoDoT();

    //Camera Settings
    void Get_Polarizer_Angles(QVector<double> &pangles);
    void Set_Polarizer_Angles(QVector<double> &pangles);

    //ELL14K Serial Port Settings
    bool Is_Serial_Port_Open(){return serial_port_open;}
    string Get_Serial_Port_Name(){return serial_port_name;}

    //ELL14K Settings
    bool Connect_ELL14K();
    bool Get_ELL14K_Device_Info();
    unsigned int Get_ELL14K_Status();
    string Get_ELL14K_Status_String(unsigned int status);
    bool Home_Device(bool dir_CCW);
    bool Set_Angular_Position(double degrees);
    bool Set_Relative_Angular_Position(double degrees);
    double Get_Angular_Position();

    //Camera Functionality
    void Detect_Device() override;
    void Initialize_Device() override;
    bool Is_Device_Busy() override;

    bool Get_Image() override;
};




/***********************************************************************************
 * Specific Device Classes To Write in the near future...
 *
 * Pyxis LWIR Microgrid
 */







/***********************************************************************************
 * deviceServer Class
 *
 * This class works as a server TCP/IP server wrapper to control a given device.
 * This class monitors port 5555 for any client requests, receives and decodes them
 * and if determined valid will forword on to the given device. Responses are then
 * sent back to the client. Responses can include basic status information or
 * data in the form of collected images. Message passing is performed using ZeroMQ
 * in conjunction with Google protocol buffers.
 */
class deviceServer {
private:

    //Private Member Variables
    string server_name = "";
    thread task_manager;
    bool return_flag = false;

    device *dev;

    //Socket and Messaging Stuff
    string response_port = "5555";
//    string image_response_port = "5556";
    mutex mutex_request;
    mutex mutex_return_flag;
    zmq::context_t context{1};
    zmq::socket_t response_socket{context,zmq::socket_type::rep};
//    zmq::socket_t imgresponse_socket{context,zmq::socket_type::rep};
    ASLpb::Request req;

    //Private Member Functions
    void Bind();
    void Task_Manager();
    void Set_Server_Name(string s) { server_name = s;}
    void Set_Return_Flag(bool b) { return_flag = b;};
    void Set_Response_Port(string s) { response_port = s;}
//    void Set_Image_Response_Port(string s) { image_response_port = s;}
    string Get_Server_Name() {return server_name;}
    bool Get_Return_Flag(){return return_flag;}
    string Get_Response_Port() { return response_port;}
//    string Get_Image_Response_Port() { return image_response_port;}

public:
    //Constructors and Destructors
    deviceServer(device *dev_ptr, const string svr_name);
    deviceServer(device *dev_ptr, const string svr_name, string res_port);//, string img_res_port);
    ~deviceServer();

    //Task Manager Startup
    void Start_Task_Manager();
};

#endif // DEVICE_SERVER_H
