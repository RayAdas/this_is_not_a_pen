#ifndef GALAXYCAMERA_H
#define GALAXYCAMERA_H

#include<iostream>
#include"GxIAPI.h"
#include"DxImageProc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
using namespace std;
class galaxycamera
{
public:
    galaxycamera();
    bool cameraInit();
    bool IGetFrame(Mat &src);
private:
    GX_STATUS status=GX_STATUS_SUCCESS;
    uint32_t nDeviceNum=0;
     GX_DEV_HANDLE hDevice = NULL;
     PGX_FRAME_BUFFER pFrameBuffer;
     PGX_FRAME_BUFFER midBuffer;
     GX_INT_RANGE ADCLevelRange;
     uint64_t nWidth=1280;
     uint64_t nHeight=1024;
};

#endif // GALAXYCAMERA_H
