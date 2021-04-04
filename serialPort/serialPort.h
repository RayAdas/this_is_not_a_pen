#ifndef USARTPORT
#define USARTPORT
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
using namespace std;
#pragma pack(1)//用于指定内存对齐
struct contralData
{
    unsigned char start_data0 = 0xAA;
    unsigned char start_data1 = 0xAA;
    //unsigned char id = 0x07;
    unsigned char label;//0X05 wei zengliang jiao 0X06 wei juedui jiao
    float yawAngle;
    float pitchAngle;
    unsigned char flags;
    unsigned char end_data = 0XBB;

};
class serialPort
{

public:
    serialPort();
    serialPort(const char* filename);
    bool Init();//1 = success
    void send_data(const struct contralData & data);
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

