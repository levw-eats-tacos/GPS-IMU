#include "berry.h"
#include <string.h>

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// function for updating the ypr instance variable
void Berry::updateYPR()
{
    char buff[1] = {1};
    DWROD bytesWritten = 0;
    WriteFile( m_serialHandle, buff, (DWORD)strlen(buff), &bytesWritten, NULL);
    std::string output = buffercheck();
    json j = json::parse(output);

    float v1 = j["kalmanx"].get<float>();
    float v2 = j["kalmany"].get<float>();
    float v3 = j["kalmanz"].get<float>();
    arr[0] = &v1;
    arr[1] = &v2;
    arr[2] = &v3;
}

// function for updating the gps instance variable
void Berry::updateGPS()
{
}

// constructor
Berry::Berry() : Device()
{
}

Berry::~Berry()
{
    disconnect();
}

// function for connecting to the device
void Berry::connect()
{
    // Open serial port
    m_serialHandle = CreateFile("\\\\.\\COM8", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

    // Do some basic settings
    DCB serialParams = {0};
    serialParams.DCBlength = sizeof(serialParams);

    GetCommState(serialHandle, &serialParams);
    serialParams.BaudRate = baudrate;
    serialParams.ByteSize = byteSize;
    serialParams.StopBits = stopBits;
    serialParams.Parity = parity;
    SetCommState(m_serialHandle, &serialParams);

    // Set timeouts
    COMMTIMEOUTS timeout = {0};
    timeout.ReadIntervalTimeout = 50;
    timeout.ReadTotalTimeoutConstant = 50;
    timeout.ReadTotalTimeoutMultiplier = 50;
    timeout.WriteTotalTimeoutConstant = 50;
    timeout.WriteTotalTimeoutMultiplier = 10;

    SetCommTimeouts(m_serialHandle, &timeout);
}

// function for disconnecting the device
void Berry::disconnect()
{
    CloseHandle(m_serialHandle);
}

std::string Berry::buffercheck()
{
    std::string stringOfStuff = "";
    while(true)
    {
        char buff[1] = {0};
        DWORD dwRead = 0;
        ReadFile( m_serialHandle, buff, 1, &dwRead, NULL);
        stringOfStuff.append(buff[0]);
        if ( buff[0] == "}" )
        {
            return stringOfStuff;
        }
    }
}
    