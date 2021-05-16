#include "coordinateTransform.h"

CoordinatTransform::CoordinatTransform(cv::Mat cameraInternalParam,
                                       cv::Mat distortionParam,
                                       cv::Size2f smallArmor,
                                       cv::Size2f bigArmor):f_(6.308e-03),length_per_pixel_(9.6e-6)
{
    this->camera_internal_param_ = cameraInternalParam;
    this->distortion_param_ = distortionParam;

    float smallArmorHalfWidth = 70.5;
    float smallArmorHalfHeight = 26;
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

    Init(0,0,0,0,0,15,0.028);
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
    //cout<<"tvec"<<tvec<<endl;
    //cout<<tvec<<endl;
    //this position need more accuraciation
    //plz ask me for more information and decision,thx

    /*
    yaw = atan(X0/distance);
    pitch = atan(Y0/distance);

        //将弧度转化为角度
    yaw = yaw * (180 / PI);
    pitch = pitch * (180 / PI);
*/
    cv::Point3f postion(X0/10,Y0/10,Z0/10);
    Transform(postion,yaw,pitch,10.5);
    Y0 *= -1;
    return cv::Point3f(X0,Y0,Z0);
}

void CoordinatTransform::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k) {
    offset_.x = x;
    offset_.y = y;
    offset_.z = z;
    offset_pitch_ = pitch;
    offset_yaw_ = yaw;
    // init_v_ = init_v;
    init_k_ = init_k;
}

void CoordinatTransform::Transform(cv::Point3f &postion, float &yaw, float &pitch,float v) {
    pitch =
            -GetPitch((postion.z + offset_.z) / 100, -(postion.y + offset_.y) / 100, v)*180.0/PI + (float)(offset_pitch_ );
    //yaw positive direction :anticlockwise
    yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z)) *180.0/PI + (float)(offset_yaw_);

}

float CoordinatTransform::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t, y;
    t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return y;
}

//x:distance , y: height
float CoordinatTransform::GetPitch(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++) {
        a = (float) atan2(y_temp, x);
        y_actual = BulletModel(x, v, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
    }
    return a;

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
    angle.y *= -1;
    return angle;
}
cv::Point3f CoordinatTransform::CCoord2ACoord(cv::Point3f cameraPoint,cv::Point2f axisData)
{
    cv::Point2f rotateAngle;
    cv::Mat _cameraPoint = (cv::Mat_<float>(3,1)<<cameraPoint.x,cameraPoint.y,cameraPoint.z);

    rotateAngle.y = -axisData.x;
    rotateAngle.x = -axisData.y;
    cv::Mat Ry = (cv::Mat_<float>(3,3)<<
                  cos(rotateAngle.y),0,-sin(rotateAngle.y),
                  0,1,0,
                  sin(rotateAngle.y),0,cos(rotateAngle.y));
    cv::Mat Rx = (cv::Mat_<float>(3,3)<<
                  1,0,0,
                  0,cos(rotateAngle.x),-sin(rotateAngle.x),
                  0,sin(rotateAngle.x),cos(rotateAngle.x));
    cv::Mat _axisPoint;
    _axisPoint = Ry * Rx * _cameraPoint;
    cv::Point3f axisPoint(_axisPoint.at<float>(0),_axisPoint.at<float>(1),_axisPoint.at<float>(2));
    return axisPoint;
}
