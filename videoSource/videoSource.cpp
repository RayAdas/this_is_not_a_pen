#include "videoSource.h"
videoSource* videoSourceUtility::sourceInit()
{
    //读入视频配置参数
    //cv::FileStorage videoSourceArgumentsRead(videoSourceArgumentsPath,cv::FileStorage::READ);
    //if(!videoSourceArgumentsRead.isOpened())
    //{
    //    cout<<"Failed to open settings file at: "<<videoSourceArgumentsPath<<endl;
    //    exit;
    //}
    //创建视频源
    videoSource *VideoSource;
    //int Type=videoSourceArgumentsRead["Type"];
    if(0)//Type为1则为视频，为0则为相机
    {//视频

    }
    else
    {//相机
        VideoSource = new cameraVideoSource;
    }
    //videoSourceArgumentsRead["PixelSize"]>>VideoSource->cameraInternalReference.PixelSize;
    //videoSourceArgumentsRead["SensorSize"]>>VideoSource->cameraInternalReference.SensorSize;
    //videoSourceArgumentsRead["F"]>>VideoSource->cameraInternalReference.F;
    //videoSourceArgumentsRead["ImageSize"]>>VideoSource->ImageSize;

    return VideoSource;
}

videoSource::videoSource()
{
}

cameraVideoSource::cameraVideoSource()
{
}

bool cameraVideoSource::Init()
{
    if(! CAMERA.cameraInit())
    {
        return 0;
    }
    //CAMERA.SetExposeTime(5500);//设置曝光时间
    //CAMERA.setFrameRate(210);//设置帧率
    //CAMERA.setBalanceRatio(1.55,1.00,1.51);


    return 1;
}

void cameraVideoSource::startPush()
{
    VideoPushThread = std::thread(&cameraVideoSource::cameraGrabImageCycle,this);
    while(ImageBuffer.read()->SrcImage.size().height == 0){}//等待读入第一张图片
}

void cameraVideoSource::cameraGrabImageCycle()
{
    while(true)
    {
        timeval NowTimmer;
        gettimeofday(&NowTimmer,0);
        CAMERA.IGetFrame(ImageBuffer.Back->SrcImage);
        ImageBuffer.Back->timestamp = NowTimmer;
        ImageBuffer.writeOver();
    }
}

ImageData* cameraVideoSource::getImage()
{
    return(ImageBuffer.read());
}
