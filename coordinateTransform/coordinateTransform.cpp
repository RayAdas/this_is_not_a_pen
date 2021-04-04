#include "coordinateTransform.h"

coordinatTransform::coordinatTransform(cv::Mat cameraInternalParam,
                   cv::Mat distortionParam,
                   cv::Size2f smallArmor,
                   cv::Size2f bigArmor):F(6.308e-03),length_per_pixel(4.8e-6)
{
    this->cameraInternalParam = cameraInternalParam;
    this->distortionParam = distortionParam;

    float smallArmorHalfWidth = smallArmor.width / 2;
    float smallArmorHalfHeight = smallArmor.height / 2;
    cv::Point3f worldPoints;

    smallArmorPoint3f.clear();


    worldPoints=cv::Point3f(-smallArmorHalfWidth,-smallArmorHalfHeight,0);//左下角点
    smallArmorPoint3f.emplace_back(worldPoints);
    worldPoints=cv::Point3f(-smallArmorHalfWidth,smallArmorHalfHeight,0) ;//左上角点
    smallArmorPoint3f.emplace_back(worldPoints);
    worldPoints=cv::Point3f(smallArmorHalfWidth,smallArmorHalfHeight,0)  ;//右上角点
    smallArmorPoint3f.emplace_back(worldPoints);
    worldPoints=cv::Point3f(smallArmorHalfWidth,-smallArmorHalfHeight,0) ;//右下角点
    smallArmorPoint3f.emplace_back(worldPoints);

    float bigArmorHalfWidth   =121.6 ;
    float bigArmorHalfHeight  =62.5;


    bigArmorPoint3f.clear();
    worldPoints=cv::Point3f(-bigArmorHalfWidth,-bigArmorHalfHeight,0);//左下角点
    bigArmorPoint3f.emplace_back(worldPoints);
    worldPoints=cv::Point3f(-bigArmorHalfWidth,bigArmorHalfHeight,0) ;//左上角点
    bigArmorPoint3f.emplace_back(worldPoints);
    worldPoints=cv::Point3f(bigArmorHalfWidth,bigArmorHalfHeight,0)  ;//右上角点
    bigArmorPoint3f.emplace_back(worldPoints);
    worldPoints=cv::Point3f(bigArmorHalfWidth,-bigArmorHalfHeight,0) ;//右下角点
    bigArmorPoint3f.emplace_back(worldPoints);
}

cv::Point3f coordinatTransform::PNP(std::vector<cv::Point2f>&xy,bool SIZE,double &distance){
    cv::Mat rvec,tvec;
    if(SIZE==false){
        //big armor
        solvePnP(bigArmorPoint3f,xy,cameraInternalParam,distortionParam,rvec,tvec);
    }
    else if (SIZE==true){//small armor

        solvePnP(smallArmorPoint3f,xy,cameraInternalParam,distortionParam,rvec,tvec);

    }
    double X0=tvec.at<double>(0,0);                                             //X轴上世界coordinate相对摄像头coordinate的平移
    double Y0=tvec.at<double>(0,1);                                             //Y轴上世界coordinate相对摄像头coordinate的平移
    double Z0=tvec.at<double>(0,2);                                             //z轴上世界coordinate相对摄像头coordinate的平移
    distance=sqrt(X0*X0+Y0*Y0+Z0*Z0);
//this position need more accuraciation
    //plz ask me for more information and decision,thx
}


void coordinatTransform::PNPcompensateShot(vector<cv::Point2f> &Point2fs, bool symbol,float shotV, float &yaw, float &pitch){
    cv::Mat rvec;
    cv::Mat tvec;

    if(symbol==true)       //true是校装甲版，false是大装甲版                                                                  //检测到是小装甲
    {
        solvePnP(smallArmorPoint3f,Point2fs,cameraInternalParam,distortionParam,rvec,tvec);  //得到相机外参参数，[r|t]；
    }
    else if(symbol==false)
    {
        solvePnP(bigArmorPoint3f,Point2fs,cameraInternalParam,distortionParam,rvec,tvec);  //得到相机外参参数，[r|t]；
    }

    double X0=tvec.at<double>(0,0);                                             //X轴上世界coordinate相对摄像头coordinate的平移
    double Y0=tvec.at<double>(0,1);                                             //Y轴上世界coordinate相对摄像头coordinate的平移
    double Z0=tvec.at<double>(0,2);

    cv::Point3f postion(X0/10,Y0/10,Z0/10);
    Transform(postion,yaw,pitch,shotV);

}


void coordinatTransform::Transform(cv::Point3f &postion, float &yaw, float &pitch,float v) {
  pitch =
      -GetPitch((postion.z + offset_.z) / 100, -(postion.y + offset_.y) / 100, v)*180.0/PI + (float)(offset_pitch_ );
  //yaw positive direction :anticlockwise
  yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z)) *180.0/PI + (float)(offset_yaw_);
}

void coordinatTransform::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k){
    offset_.x = x;
    offset_.y = y;
    offset_.z = z;
    offset_pitch_ = pitch;
    offset_yaw_ = yaw;
   // init_v_ = init_v;
    init_k_ = init_k;
}

float coordinatTransform::GetPitch(float x, float y, float v){
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

float coordinatTransform::BulletModel(float x, float v, float angle){
    float t, y;
    t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return y;
}

cv::Point2f coordinatTransform::PCoord2ICoord(cv::Point2f pixelPoint)
{
    pixelPoint.x -= 320;
    pixelPoint.y -= 256;
    pixelPoint.x *= length_per_pixel;
    pixelPoint.y *= length_per_pixel;
    return pixelPoint;
}
cv::Point2f coordinatTransform::ICoord2CCoord(cv::Point2f imagePoint)
{
    cv::Point2f angle(0,0);
    angle.x = atan(imagePoint.x / this->F);
    angle.y = atan(imagePoint.y / this->F);
    return angle;
}
