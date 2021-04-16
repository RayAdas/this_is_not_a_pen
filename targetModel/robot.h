#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "targetModel.h"
#include "coordinateTransform/coordinateTransform.h"
#include "preferences.h"
#include "videoSource/videosource.h"
using namespace std;
using namespace cv;


//该结构体存储关于灯条检测的所有参数
struct LightParam
{
    int Light_Point_min;                   //灯条最小点集个数
    int Light_Point_max;
    float DBH_max=5;
    float DBH_min=1.1;
    float Light_Max_Min2Points;            //轮廓点集很小的最大，最小比限幅
    float Light_Normal_Max_Min2Points;     //轮廓点集正常的最大，最小比限幅
    float angle_center_difference;          //灯条自身角度与两灯条夹角的最大差值
    float angle_difference;                 //灯条旋转角度的最大差值
    float height_difference_ratio;                     //灯条最长边相差的最大比例

    //为各项参数赋默认值
    LightParam()
    {
        Light_Point_min=8;                //灯条最小点集个数
        Light_Point_max=2100;
        Light_Max_Min2Points=2.3;         //轮廓点集很小的最大，最小比限幅
        Light_Normal_Max_Min2Points=1.8;  //轮廓点集正常的最大，最小比限幅
        angle_center_difference=13;        //灯条自身角度与两灯条夹角的最大差值
        angle_difference=15;               //灯条旋转角度的最大差值
        height_difference_ratio=4;                    //灯条最长边相差的最大比例
    }
};

/*
struct Message
{
    float pitch;
    float yaw;
    Message()
    {
        float pitch=0;
        float yaw=0;
    }
};
*/

typedef enum _
{
    Lightleft=0,
    Lightright=1
}sense_of_roRect;


typedef enum ___
{
    UNKNOWN_ARMOR = 0,
    SMALL_ARMOR   = 1,
    BIG_ARMOR     = 2,
    SMALL_BUFF    = 3,
    BIG_BUFF      = 4
}ObjectType;


//灯条描述的类
class LightDescriptor
{
public:
    LightDescriptor();//无参构造
    LightDescriptor(const cv::RotatedRect& another);
    LightDescriptor& operator=(const LightDescriptor& another);
    cv::RotatedRect rotatedrect();//返回描述灯条最小的旋转包围矩形

public:

    sense_of_roRect sense;//表示灯条方向
    cv::Point2f center;
    float max;
    float min;
    float angle;
    float area;
};

//装甲板描述类
/*
 * 因为装甲板外形也是一个旋转矩形，只是多了一些它固有的特性，因此我选择直接从用装甲板描述类去继承旋转矩形。
 * */
class ArmorDescriptor : public RotatedRect
{
public:
    ArmorDescriptor();
    ArmorDescriptor(const RotatedRect& another,ObjectType type=UNKNOWN_ARMOR,Robot_Type ro=Infantry);
    ArmorDescriptor& operator =(const ArmorDescriptor& another);
    RotatedRect rotatedrect();
    void setRobotType(Robot_Type set);
    void setArmorrType(ObjectType set);


public:
    float               Longest;
    float              Shortest;
    float              lightLen;
    float                 pitch;
    float                   yaw;
    double             distance;
    Robot_Type            robot;
    ObjectType        armorType;
    sense_of_roRect  armorsense;
    vector<Point2f> armorPoints;



};

class ArmorModel:public TargetModel
{
public:
    armorModel(coordinatTransform*);

    void setImage(cv::Mat& set_src);    //预处理图像

    void judgeArmorrType(ArmorDescriptor &a,float arrmorHBW);
    void getLightLen(vector<Point2f> &lightPoint2fs,float &len);    //获取装甲板最长灯条长度
    void getArmorImagePoint2f(ArmorDescriptor &armor, Point2f Points[]);

public:

    Mat frame,mask;
    cv::Mat cameraInternalParam;
    cv::Mat distortionParam;
    float yaw=0,pitch=0;
    LightParam Light;                                //装甲板描述的结构体
    Point2f offset_point;
    Size ImageSize;                                  //图片大小
    cv::Mat src_roi;
    ArmorDescriptor targetArrmor;                    // 目标装甲板
//    ArmorDescriptor lastArrmor;
    vector<LightDescriptor> lightCountersRoRect;     // 筛选出来的单个灯条vector
    double armorDistance=0;



public:
    double Distance(Point2f, Point2f);
    //explicit armorModel(Mat& input);
    void amend(ImageData* imageData) override;//修正预测模型
    //virtual amend(陀螺仪数据) = 0;//修正预测模型
    cv::Point3f getFuturePosition(const float offset) override;//获得预测点



private:
    CoordinatTransform* pnpsolve;
    Point3f GetArmorCenter();
    void setInputImage(Mat input);
    void Pretreatment();


};
#endif // ROBOT_H
