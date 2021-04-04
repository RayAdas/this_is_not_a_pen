#ifndef COORDINATTRANSFORM_H
#define COORDINATTRANSFORM_H
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>


#include "videoSource/videoSource.h"

const double PI = 3.1415926535;
const float GRAVITY = 9.78;

class coordinatTransform
{
public:
    coordinatTransform(cv::Mat cameraInternalParam,
                       cv::Mat distortionParam,
                       cv::Size2f smallArmor,
                       cv::Size2f bigArmor);
    cv::Point2f PCoord2ICoord(cv::Point2f pixelPoint);
    cv::Point2f ICoord2CCoord(cv::Point2f imagePoint);
    cv::Point3f PNP(std::vector<cv::Point2f> &xy, bool SIZE,double &distance);
    void PNPcompensateShot(vector<cv::Point2f> &Point2fs, bool symbol,float shotV, float &yaw, float &pitch);
    void Transform(cv::Point3f &postion, float &yaw, float &pitch,float v);
    void Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k);
    void init_PNP();
    static void pixel2what(cv::Point2f pixelPoint, float &angleX,float &angleY);
    const float length_per_pixel;// = 4.8e-6;
    const float F;// = 6.308e-03;

    float GetPitch(float x, float y, float v);
    float BulletModel(float x, float v, float angle);
private:
    std::vector<cv::Point3f>smallArmorPoint3f;
    std::vector<cv::Point3f>bigArmorPoint3f;
    videoSource* VideoSource;
    cv::Mat camera_matrix;//opencv测得的相机主要内参矩阵
    cv::Mat distortion_coefficients;//opencv测得的相机畸变参数矩阵
    cv::Mat cameraInternalParam;
    cv::Mat distortionParam;
    cv::Point3f offset_;
    float offset_pitch_;
    float offset_yaw_;
    float init_k_;

};

#endif // COORDINATTRANSFORM_H
