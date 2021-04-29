#ifndef CAMERA_H
#define CAMERA_H
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/ParameterNode.h"
#include "Media/ImageConvert.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace Dahua::Memory;
using namespace std;

class Camera
{
public:
    Camera();
    bool cameraInit();
    bool IGetFrame(cv::Mat& src);//获取一帧图像
    void setFrameRate(double rate = 210);//设置帧率
    void SetExposeTime(double exp);//设置曝光时间
    void setBalanceRatio(double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio);//设置白平衡
public:
    ICameraPtr cameraSptr;                   // 相机指针对象
private:
    cv::Size2i image_size_ = cv::Size(640,512);//图像大小
    TVector<ICameraPtr> ICameraPtrVector;     // 相机指针列表对象
    IStreamSourcePtr streamPtr;               //流数据对象

};

#endif // CAMERA_H
