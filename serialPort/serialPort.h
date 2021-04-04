#ifndef USARTPORT
#define USARTPORT
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
using namespace std;
#pragma pack(1)//用于指定内存对齐

class serialPort
{

public:
    serialPort();
    serialPort(const char* filename);
    bool Init();//1 = success
    void send_data(const unsigned char* data,const int length);
    bool get_data();//1:success
    unsigned char receiceData[22];
private:
    char fd;
    const char* file_name_;
    int buadrate_;//0 or 1
    float last_bullet_speed;
    int last_blood;
};








#endif // USARTPORT

