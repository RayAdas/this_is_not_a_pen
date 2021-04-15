#ifndef COORDINATTRANSFORM_H
#define COORDINATTRANSFORM_H
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>

#include "videoSource/videosource.h"

const double PI = 3.1415926535;
const float GRAVITY = 9.78;

class CoordinatTransform
{
public:
    CoordinatTransform(cv::Mat camera_internal_param_,
                       cv::Mat distortion_param_,
                       cv::Size2f smallArmor,
                       cv::Size2f bigArmor);
    cv::Point2f PCoord2ICoord(cv::Point2f pixelPoint);
    cv::Point2f ICoord2CCoord(cv::Point2f imagePoint);
    cv::Point3f PNP(std::vector<cv::Point2f>&xy,bool SIZE,double &distance,float &yaw,float &pitch);

    void init_PNP();
    static void pixel2what(cv::Point2f pixelPoint, float &angleX,float &angleY);
    const float length_per_pixel_;// = 4.8e-6;
    const float f_;// = 6.308e-03;


private:
    std::vector<cv::Point3f>small_armor_point3f_;
    std::vector<cv::Point3f>big_armor_point3f_;

    cv::Mat camera_matrix_;//opencv测得的相机主要内参矩阵
    cv::Mat distortion_coefficients_;//opencv测得的相机畸变参数矩阵
    cv::Mat camera_internal_param_;
    cv::Mat distortion_param_;
    cv::Point3f offset_;
    float offset_pitch_;
    float offset_yaw_;
    float init_k_;

};

#endif // COORDINATTRANSFORM_H
