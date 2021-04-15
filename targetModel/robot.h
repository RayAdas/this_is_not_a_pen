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



typedef enum _
{
    Lightleft=0,
    Lightright=1
}sense_of_roRect;

typedef enum __
{
    ARMOR_NO = 0,		// not found
    ARMOR_LOST = 1,		// lose tracking
    ARMOR_GLOBAL = 2,	// armor found globally
    ARMOR_LOCAL = 3		// armor found locally(in tracking mode)
}ArmorFindFlag;

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
    float              Longest;
    float             Shortest;
    float             lightLen;
    Robot_Type            robot;
    ObjectType       armorType;
    sense_of_roRect armorsense;
    vector<Point2f> armorPoints;


};

class ArmorModel:public TargetModel
{
public:

    ArmorModel(CoordinatTransform*);
    void judgeArrmorState();    //判断图像处理roi区size
    ArmorFindFlag ArrmorDection();    //find装甲板主要函数
    void histMaker(Mat& src_hist);    //画出图像直方图
    void setImage(cv::Mat& set_src);    //预处理图像

    void setDigtisRecognize(bool flag);    //开关装甲板数字识别
    void judgeArmorrType(ArmorDescriptor &a);
    void recrodArmorStatus(bool isFoundArmor);    //记录find装甲板结果
    void getLightLen(vector<Point2f> &lightPoint2fs,float &len);    //获取装甲板最长灯条长度
    void getArmorImagePoint2f(ArmorDescriptor &armor, vector<Point2f> &point2fs);

public:

    Mat frame,mask;
    cv::Mat cameraInternalParam;
    cv::Mat distortionParam;
    float yaw=0,pitch=0;
    LightParam Light;                                //装甲板描述的结构体
    Point2f offset_roi_point;
    Size ImageSize;
    Size roiImageSize;                               //找到的装甲板大小
    cv::Mat src_roi;                                 //寻找装甲板的ROI大小
    ArmorDescriptor targetArrmor;                    // 目标装甲板
    cv::RotatedRect targetArrmor2FindRoi;
    vector<LightDescriptor> lightCountersRoRect;     // 筛选出来的单个灯条vector
    double armorDistance=0;

private:
    ArmorFindFlag _armorFindFlag;
    int _trackCounter;      //记录在追踪模式下处理图片的张数，达到max_track_num后变为全局搜索模式
    bool _isTracking;       //判断是否在追踪模式下
    float widthRatio;
    float heightRatio;
    int losed_counter;                 // 目标装甲板丢失计数器
    int find_counter;                  //目标装甲板找到计数器


    //Ptr<ml::SVM> SVM_Params;           //svm数字识别模型
    //Ptr<ml::SVM> SVM_ArmorTypeParam;   //装甲板类型svm模型
    bool enableDigitsRecognize;

    void mySvmPredict(Mat& src,int& armorNum)             ;
    int mySvmArmorTypePredict(float ratio, float angle)   ;
    void drawMaxConnect(Mat& out,Mat& labels,int maxLabel);



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
