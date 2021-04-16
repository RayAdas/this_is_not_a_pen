#ifndef CRITICALCORE_H
#define CRITICALCORE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define CREDIBLE_CONSECUTIVE_TIMES 3
#define VARIANCE_BUFFER_MAX 300
struct AngleTick
{
    float angle;
    double timestamp;
};

struct SpeedTick
{
    float speed;
    double timestamp;
};

class CriticalCore
{
    //spd = 0.785 * sin(1.884 * t) + 1.305
    //pos = 0.785 * -1 / 1.884 * cos(1.884 * t) + 1.305 * t - (-1 * 0.785 / 1.884) + pos(0)

    //spd = 1.047
    //pos = 1.047 * t
public:
    CriticalCore();
    void Init(AngleTick initialAngleTick);
    void amend(AngleTick thisAbgleTick);
    float getFutureAngle(const long double interval);

private:
    void amendPhase(const AngleTick preAbgleTick,const AngleTick afterAbgleTick);
    float getMaxDeltaAngle(double timeInterval);
    void solveInverseTrigonometricFunction(float A,float &t1,float &t2);

private:

    long double t_inter_;//相位差，定义不明但是能跑
    int positive_or_negative_;//正反向指示器,正数为正向，负数为反向
    float variable_or_uniform_;//大小能量机关指示器
    int consecutive_times_;//连续次数

    AngleTick jump_angle_tick_buffer_[CREDIBLE_CONSECUTIVE_TIMES];//储存暂时不被信任的点
    std::queue<float> variance_buffer_;//方差储存器
    double variance_sum_;//方差和

    AngleTick last_angle_tick_;
    SpeedTick last_speed_;
};

#endif // CRITICALCORE_H
