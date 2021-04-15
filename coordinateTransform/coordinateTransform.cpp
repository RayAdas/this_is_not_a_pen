#include "coordinateTransform.h"

CoordinatTransform::CoordinatTransform(cv::Mat cameraInternalParam,
                   cv::Mat distortionParam,
                   cv::Size2f smallArmor,
                   cv::Size2f bigArmor):f_(6.308e-03),length_per_pixel_(4.8e-6)
{
    this->camera_internal_param_ = cameraInternalParam;
    this->distortion_param_ = distortionParam;
/*
    float smallArmorHalfWidth = smallArmor.width / 2;
    float smallArmorHalfHeight = smallArmor.height / 2;

    float smallArmorHalfWidth = 32;
    float smallArmorHalfHeight = 27;
    */

    float smallArmorHalfWidth = 70.5;
    float smallArmorHalfHeight = 62.5;
    cv::Point3f worldPoints;

    small_armor_point3f_.clear();


    worldPoints=cv::Point3f(-smallArmorHalfWidth,-smallArmorHalfHeight,0);//左下角点
    small_armor_point3f_.emplace_back(worldPoints);
    worldPoints=cv::Point3f(-smallArmorHalfWidth,smallArmorHalfHeight,0) ;//左上角点
    small_armor_point3f_.emplace_back(worldPoints);
    worldPoints=cv::Point3f(smallArmorHalfWidth,smallArmorHalfHeight,0)  ;//右上角点
    small_armor_point3f_.emplace_back(worldPoints);
    worldPoints=cv::Point3f(smallArmorHalfWidth,-smallArmorHalfHeight,0) ;//右下角点
    small_armor_point3f_.emplace_back(worldPoints);

//    float bigArmorHalfWidth   =121.6 ;
//    float bigArmorHalfHeight  =62.5;
    float bigArmorHalfWidth   =116;
    float bigArmorHalfHeight  =26;


    big_armor_point3f_.clear();
    worldPoints=cv::Point3f(-bigArmorHalfWidth,-bigArmorHalfHeight,0);//左下角点
    big_armor_point3f_.emplace_back(worldPoints);
    worldPoints=cv::Point3f(-bigArmorHalfWidth,bigArmorHalfHeight,0) ;//左上角点
    big_armor_point3f_.emplace_back(worldPoints);
    worldPoints=cv::Point3f(bigArmorHalfWidth,bigArmorHalfHeight,0)  ;//右上角点
    big_armor_point3f_.emplace_back(worldPoints);
    worldPoints=cv::Point3f(bigArmorHalfWidth,-bigArmorHalfHeight,0) ;//右下角点
    big_armor_point3f_.emplace_back(worldPoints);
}

cv::Point3f CoordinatTransform::PNP(std::vector<cv::Point2f>&xy,bool SIZE,double &distance,float &yaw,float &pitch)
{
    cv::Mat rvec,tvec;
    if(SIZE==false){
        //big armor
        solvePnP(big_armor_point3f_,xy,camera_internal_param_,distortion_param_,rvec,tvec);
    }
    else if (SIZE==true){//small armor

        solvePnP(small_armor_point3f_,xy,camera_internal_param_,distortion_param_,rvec,tvec);

    }
    double X0=tvec.at<double>(0,0);                                             //X轴上世界coordinate相对摄像头coordinate的平移
    double Y0=tvec.at<double>(0,1);                                             //Y轴上世界coordinate相对摄像头coordinate的平移
    double Z0=tvec.at<double>(0,2);                                             //z轴上世界coordinate相对摄像头coordinate的平移
    distance=sqrt(X0*X0+Y0*Y0+Z0*Z0);
//this position need more accuraciation
    //plz ask me for more information and decision,thx


    yaw = atan(X0/distance);
    pitch = atan(Y0/distance);

        //将弧度转化为角度
    yaw = yaw * (180 / PI);
    pitch = pitch * (180 / PI);
    return cv::Point3f(X0,Y0,Z0);
}



cv::Point2f CoordinatTransform::PCoord2ICoord(cv::Point2f pixelPoint)
{
    pixelPoint.x -= 320;
    pixelPoint.y -= 256;
    pixelPoint.x *= length_per_pixel_;
    pixelPoint.y *= length_per_pixel_;
    return pixelPoint;
}
cv::Point2f CoordinatTransform::ICoord2CCoord(cv::Point2f imagePoint)
{
    cv::Point2f angle(0,0);
    angle.x = atan(imagePoint.x / this->f_);
    angle.y = atan(imagePoint.y / this->f_);
    return angle;
}
