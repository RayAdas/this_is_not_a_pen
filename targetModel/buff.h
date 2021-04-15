#ifndef BUFF_H
#define BUFF_H

//注意：包括cv::RotatedRect.angle在内的所有单位均为弧度

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include "tool/tool.h"
#include <limits.h>
#include "targetModel/targetModel.h"
#include <stdlib.h>//test
#include <trajectoryCalculation.h>
#include "coordinateTransform/coordinateTransform.h"
#include "targetModel/criticalcore.h"

#define subThreMinRed 70
#define subThreMaxRed 255

#define subThreMinBule 65
#define subThreMaxBule 255

#define MIN_ACCEPTABLE_ARROMOR_AREA 1000//最小的可接受的大装甲板面积

#define RATIO_OF_R_WIDTH 3
#define MIN_ACCEPTABLE_ARROMOR_SUB_AREA 100//最小的可接受的大装甲板子区域面积


#define BUFF_CENTER_DISTANCE 6.4
#define BUFF_CENTER_HEIGHT 0.97
#define BUFF_CENTER_RADIUS 0.7

enum ArromorLightEnum{off,target,complete};
struct ArmorTick
{
    cv::Point2f targetPoint = cv::Point2f(-1,-1);//打击点
    float angle;//这个角度以x轴正半轴为准，范围0～2PI
    ArromorLightEnum aromorLight = off;
    float length;
};


struct BuffTick
{
    cv::Point2f center = cv::Point2f(0,0);//Buff中心点
    float angle = -1;//取与x轴的最小正夹角
    timeval timestamp;//时间戳
    float radius;//半径
    int lightCounter = 0;//亮了几块装甲板
};

class BuffTickUtility//buffTick的工具类
{
public:
    static BuffTick Mat2buffTick(const cv::Mat &src,TeamColor enemyColor);
private:
    static void preprocess(const cv::Mat &src,cv::Mat &dst,TeamColor enemyColor);
    static float getArmorTickAngle(const cv::RotatedRect &tragetRotRect,const cv::RotatedRect &rotRect1,const cv::RotatedRect &rotRect2);
    static float getArmorTickAngle(const cv::RotatedRect &tragetRotRect);
    static double markACenterLikePoint(cv::Point2f canddidate,const ArmorTick &judge);
    static int getTargetNum(const BuffTick& thisBuffTick);//返回一个buffTick的target装甲板的的下标
};
class BuffModel:public TargetModel
{
public:
    BuffModel();
    void amend(ImageData* imageData) override;//修正预测模型
    //void amend(const axisData* AxisData) override;//修正预测模型
    cv::Point3f getFuturePosition(const float offset) override;//获得预测点
    cv::Point2f getCurrentDirection();
private:
    CriticalCore criti_calcore_;
    CoordinatTransform *coordinat_transform_;
    BuffTick last_buff_tick_;
    int confidence_level_;
    timeval what_time_;

};

#endif // BUFF_H
