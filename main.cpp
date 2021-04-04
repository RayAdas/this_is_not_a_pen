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

#include "targetModel/buff.h"
#include "videoSource/videoSource.h"
#include "serialPort/serialPort.h"
#include "targetModel/robot.h"
#include "coordinateTransform/coordinateTransform.h"
#include "controller.h"
#include "tool/kalman.h"
#include "trajectoryCalculation.h"
#include "videoSource/galaxycamera.h"
#define MAIN_PATH (string)"/home/ware_/114514"

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace Dahua::Memory;
using namespace std;
using namespace cv;

int main()
{
    /*
    galaxycamera gx;
    Mat imgs;
    gx.initial_camera();
    while(1){
        gx.getimage(imgs);
        imshow("imgs",imgs);
        waitKey(1);
    }
    */

    //读入主配置文件参数
    /*cv::FileStorage mainArgumentsRead(MAIN_PATH + "/MainArguments.xml",cv::FileStorage::READ);
    if(!mainArgumentsRead.isOpened())
    {
        cout<<"Failed to open settings file at: "<<MAIN_PATH + "/MainArguments.xml"<<endl;
        exit;
    }
    string videoSourceArgumentsRelativePath = (string)mainArgumentsRead["VideoSourceArgumentsPath"];
    */

    //创建视频源
    videoSource *VideoSource = videoSourceUtility::sourceInit();

    //创建角度解算器
    cv::Mat cameraInternalParam = (cv::Mat_<double>(3,3) <<
                         1358.507 , 0 , 337.4941 ,
                          0       , 1351.934 , 257.2953 ,
                          0       , 0       , 1       );
    cv::Mat distortionParam = (cv::Mat_<double>(1,5) <<
                         -0.0804,0.3165,0,0,0);
    cv::Size2f smallArmor;smallArmor.height = 141;smallArmor.width = 125;
    cv::Size2f bigArmor;bigArmor.height = 243.2;smallArmor.width = 125;
    coordinatTransform CameraTransformer(cameraInternalParam,
                                         distortionParam,
                                         smallArmor,
                                         bigArmor);

    //创建串口
    char dev[]={"/dev/ttyTHS2"};
    serialPort Uart1(dev);

    //创建弹道解算器

    //创建控制器
    controller Controller(VideoSource,&CameraTransformer,&Uart1);

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
    return 0;
}
