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
//#include "targetModel/criticalcore.h"

#define subThreMinRed 70
#define subThreMaxRed 255

#define subThreMinBule 65
#define subThreMaxBule 255

#define MIN_ACCEPTABLE_ARROMOR_AREA 1000//最小的可接受的大装甲板面积

#define RATIO_OF_R_WIDTH 3.0
#define MIN_ACCEPTABLE_ARROMOR_SUB_AREA 100//最小的可接受的大装甲板子区域面积

#define BUFF_CENTER_DISTANCE 7
#define BUFF_CENTER_HEIGHT 0.97
#define BUFF_CENTER_RADIUS 0.7

#define CREDIBLE_CONSECUTIVE_TIMES 3
#define VARIANCE_BUFFER_MAX 300

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
    timeval timestamp;//时间戳
    cv::Point2f center = cv::Point2f(0,0);//Buff中心点
    float angle = -1;//取与x轴的最小正夹角
    float radius;//半径
    int lightCounter = 0;//亮了几块装甲板
};

struct AngleTick
{
    double timestamp;
    float angle;
};

struct SpeedTick
{
    double timestamp;
    float speed;
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
    BuffModel(CoordinatTransform* coordinatTransform);
    ~BuffModel();
    void amend(ImageData* imageData) override;//修正预测模型
    void amend(cv::Point2f* axisData) override;//修正预测模型
    cv::Point3f getFuturePosition(const float offset) override;//获得预测点
private:
    cv::Point2f getCameraCurrentDirection();
private:
    //CriticalCore criti_calcore_;
    BuffTick last_buff_tick_;
    CoordinatTransform *coordinat_transform_ = nullptr;
    int confidence_level_;
    timeval what_time_;

    /*=======预测部份=======*/
    //变速能量机关
    //spd = 0.785 * sin(1.884 * t) + 1.305
    //pos = 0.785 * -1 / 1.884 * cos(1.884 * t) + 1.305 * t - (-1 * 0.785 / 1.884) + pos(0)

    //匀速能量机关
    //spd = 1.047
    //pos = 1.047 * t
private:
    void initAngle(const AngleTick initialAngleTick);
    void amendAngle(const AngleTick thisAbgleTick);
    float getFutureAngle(const long double interval);
    void amendPhase(const AngleTick preAbgleTick,const AngleTick afterAbgleTick);
    float getMaxDeltaAngle(double timeInterval);
    void solveInverseTrigonometricFunction(float A,float &t1,float &t2);
private:
    AngleTick last_angle_tick_;
    SpeedTick last_speed_;
    long double t_inter_;//相位差，定义不明但是能跑

    AngleTick jump_angle_tick_buffer_[CREDIBLE_CONSECUTIVE_TIMES];//储存暂时不被信任的点
    std::queue<float> variance_buffer_;//方差储存器
    double variance_sum_;//方差和

    int positive_or_negative_;//正反向指示器,正数为正向，负数为反向
    float variable_or_uniform_;//大小能量机关指示器
    int consecutive_times_;//连续次数


};

#endif // BUFF_H
