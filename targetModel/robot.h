#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "targetModel.h"
#include "coordinateTransform/coordinateTransform.h"
using namespace std;
using namespace cv;




class armorModel:public targetModel
{

private:

    Mat frame, hsv, mask,test,test2;
    Mat elementDilate=getStructuringElement(MORPH_RECT,Size(1,3),Point(-1,-1));
    Point2f currentCenter;
    Point2f lastCenter;
    vector<RotatedRect> minRects;
    int lost;
    cv::Mat cameraInternalParam;
    cv::Mat distortionParam;
    std::vector<cv::Point3f>smallArmorPoint3f;
    std::vector<cv::Point3f>bigArmorPoint3f;
    vector<Point2f> arrmorpoint2fs;
    double jvli[1000000];
    cv::Point2f arrmorpoints[4];
    bool armorsense=true;//ture是左偏，false是右偏
    bool target = true;  //true是找到目标，false是没有找到目标
    double mindistance;
    int index;
    double Longest;
    double Shortest;
    bool armorType=true;//true小装甲版，false是大装甲版
    Ptr<ml::SVM> SVM_Params;           //svm数字识别模型
    Ptr<ml::SVM> SVM_ArmorTypeParam;   //装甲板类型svm模型
    double maxarmorsize;
    double DBH;
    float yaw=0,pitch=0;
public:
    armorModel(coordinatTransform *);
    //explicit armorModel(Mat& input);
    void amend(const ImageData* imageData) override;//修正预测模型
    //void amend(const axisData* AxisData) override;//修正预测模型
    //virtual amend(陀螺仪数据) = 0;//修正预测模型
    cv::Point2f getFuturePosition(const float offset) override;//获得预测点


private:
    coordinatTransform *pnpsolve;
    Point2f GetArmorCenter();
    void setInputImage(Mat input);
    void Pretreatment();
    void LostTarget();
    double Distance(Point2f, Point2f);
    double max(double, double);
    double min(double, double);
    void DrawArrmor(RotatedRect& light1,RotatedRect& light2);
    void mySvmPredict(Mat& src,int& armorNum)             ;
    int mySvmArmorTypePredict(float ratio, float angle)   ;
    void judgeArmorrType(RotatedRect &a);
    static bool CmpLight(RotatedRect L1,RotatedRect L2);
};
#endif // ROBOT_H
