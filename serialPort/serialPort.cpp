#include <serialPort/serialPort.h>


SerialPort::SerialPort()
{

}


SerialPort::SerialPort(const char * filename)
{
    file_name_ = filename;
    buadrate_ = 0;
    //    serial_mode = NO_INIT;
}
bool SerialPort::Init()
{
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port// No terminal will control the process
    if(fd == -1)
    {
        //        NOTICE("wait serial " << file_name_,1);
        return false;
    }
    else if(fd != -1 )
    {
        fcntl(fd, F_SETFL,0);//这是原来的，阻塞的
        //fcntl(fd,F_SETFL,FNDELAY);//这个是非阻塞
        //        NOTICE("port is open " << file_name_,1);
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate_ == 1)
    {
        cfsetispeed(&port_settings, B230400);       // set baud rates
        cfsetospeed(&port_settings, B230400);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 0;           // read doesn't block
    port_settings.c_cc[VTIME] = 0;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port
    tcflush(fd, TCIOFLUSH);
    return true;
}

void SerialPort::send_data(const unsigned char* data,const int length)
{
    write(fd,data,length);
}
ssize_t SerialPort::get_data(unsigned char* r,const int maxLength)
{
    read(fd,&r,sizeof(r));
    ssize_t readLength = read(fd,r,maxLength);

    if(readLength > 0)
    {
        //cout<<"?"<<readLength<<endl;
    }
/*
    for(int i=0;i<22;i++)
    {
        printf("%x|",r[i]);
    }
    printf("\n");
*/
    if(readLength > 0)
    {
        return readLength;
    }
    else
    {
        return 0;
    }
}
