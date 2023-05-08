#include <windows.h>
#include <iostream>


int main(int argc, char* argv[])
{
    //Opening Serial Communication Port For Arduino Communication
    HANDLE hComm;
    DWORD dNoOFBytestoWrite;         // No of bytes to write into the port
    DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
    DWORD bytes_read = 0;

    hComm = CreateFileA("\\\\.\\COM3", GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);        // Null for Comm Devices

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hComm, &dcb);
    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    SetCommState(hComm, &dcb);


    if (hComm == INVALID_HANDLE_VALUE)
    {
        std::cerr << "Error in opening serial port\n";
        return -1;
    }
    else
        std::cerr << "Opening serial port successful\n";

    return 0;
}