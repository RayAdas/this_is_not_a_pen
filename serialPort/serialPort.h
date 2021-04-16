#ifndef USARTPORT
#define USARTPORT
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
using namespace std;
#pragma pack(1)//用于指定内存对齐

class SerialPort
{

public:
    SerialPort();
    SerialPort(const char* filename);
    bool Init();//1 = success
    void send_data(const unsigned char* data,const int length);
    bool get_data();//1:success
    unsigned char receice_data_[22];
private:
    char fd;
    const char* file_name_;
    int buadrate_;//0 or 1
};








#endif // USARTPORT

