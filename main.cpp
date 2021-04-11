#include <math.h>
#include <iostream>

#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Media/ImageConvert.h"
#include "Infra/PrintLog.h"
#include "Memory/SharedPtr.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "preferences.h"

#include "targetModel/buff.h"
#include "videoSource/videoSource.h"
#include "serialPort/serialPort.h"
#include "targetModel/robot.h"
#include "coordinateTransform/coordinateTransform.h"
#include "controller.h"
#include "tool/kalman.h"
#include "trajectoryCalculation.h"
#include "videoSource/galaxycamera.h"

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace Dahua::Memory;
using namespace std;
using namespace cv;

int main()
{
    //创建视频源
    videoSource *VideoSource = videoSourceUtility::sourceInit();

    //创建角度解算器
    cv::Mat cameraInternalParam = (cv::Mat_<double>(3,3) <<
                                       644.531787 , 0 , 320 ,
                                       0       , 642.090508 , 256 ,
                                       0       , 0       , 1       );
    /*cv::Mat cameraInternalParam = (cv::Mat_<double>(3,3) <<
                                   1289.063574 , 0 , 320 ,
                                   0       , 1284.181016 , 256 ,
                                   0       , 0       , 1       );*/
    /*cv::Mat cameraInternalParam = (cv::Mat_<double>(3,3) <<
                                   1289.063574 , 0 , 639.5 ,
                                   0       , 1284.181016 , 511.5 ,
                                   0       , 0       , 1       );*/
    cv::Mat distortionParam = (cv::Mat_<double>(1,5) <<
                               -0.071912,0.078401,0.000264,-0.002826,0.113940);
    cv::Size2f smallArmor;smallArmor.height = 141;smallArmor.width = 125;
    cv::Size2f bigArmor;bigArmor.height = 243.2;smallArmor.width = 125;
    coordinatTransform CameraTransformer(cameraInternalParam,
                                         distortionParam,
                                         smallArmor,
                                         bigArmor);

    //创建串口
    char dev[]={"/dev/ttyTHS2"};
    serialPort Uart1(dev);
    UartKeeper *UKeeper;

    switch(THIS_ROBOT_TYPE)
    {
    case Robot_Type::Hero:UKeeper = new UartKeeper_Infantry(&Uart1);break;
    case Robot_Type::Infantry:UKeeper = new UartKeeper_Infantry(&Uart1);break;
    case Robot_Type::Guard:UKeeper = new UartKeeper_Guard(&Uart1);break;
    case Robot_Type::Drone:UKeeper = new UartKeeper_Infantry(&Uart1);break;
    }
    //创建弹道解算器

    //创建控制器
    controller Controller(VideoSource,&CameraTransformer,UKeeper);

    while(true)
    {
        if(Uart1.Init() == false)
        {
            cout<<"serial port offline"<<endl;
            //cout<<"waitting to open"<<dev<<"..."<<endl;
            //while(! Uart1.Init()){}//等待串口开启
            //cout<<"serial port online"<<endl;
        }
        else{cout<<"serial port online"<<endl;}


        if(VideoSource->Init() == false)
        {
            cout<<"video source offline"<<endl;
            cout<<"waitting to open video source..."<<endl;
            while(! VideoSource->Init()){}//等待开启
            cout<<"video source online"<<endl;
        }
        else
        {
            cout<<"video source online"<<endl;
            //开启推流
            VideoSource->startPush();
        }
        //移交控制器
        Controller.boot();
    }
    delete UKeeper;
    return 0;
}
