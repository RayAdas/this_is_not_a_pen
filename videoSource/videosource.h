#ifndef VIDEOSOURCE_H
#define VIDEOSOURCE_H

#include "videoSource/camera.h"
#include <thread>
#include <sys/time.h>
#include "tool/triplebuffering.h"
#include "videoSource/galaxycamera.h"
struct ImageData//图像信息
{
    cv::Mat SrcImage;//图像Mat
    timeval timestamp;//时间戳
};

class VideoSource
{
public:
    VideoSource();
    virtual bool Init() = 0;
    virtual void startPush() = 0;//开始自动读入视频
    virtual ImageData* getImage() = 0;//返回最新一张图片的引用,并返回数组下标
    cv::Size2i ImageSize;
    TripleBuffering<ImageData> ImageBuffer;
protected:
    std::thread VideoPushThread;//startPush函数的推流线程

};

class CameraVideoSource:public VideoSource//使用摄像头时
{
public:
    CameraVideoSource();
    bool Init() override;
    void startPush() override;
    ImageData* getImage() override;
public:
    //camera CAMERA;//摄像头对象
    GalaxyCamera CAMERA;
private:
    void cameraGrabImageCycle();
private:
};


class VideoSourceUtility
{
public:
    static VideoSource* sourceInit();
};
#endif // VIDEOSOURCE_H
