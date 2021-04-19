#include "criticalcore.h"

CriticalCore::CriticalCore()
{
}
void CriticalCore::Init(const AngleTick initialAngleTick)
{
    this->last_angle_tick_ = initialAngleTick;
    this->consecutive_times_ = 0;
    this->positive_or_negative_ = 0;
    this->last_speed_.speed = 0;
    this->last_speed_.timestamp = 0;
    this->t_inter_ = 0;
    while(!variance_buffer_.empty()){variance_buffer_.pop();}//队列清零
    this->variance_sum_ = 0;
}

void CriticalCore::amend(const AngleTick thisAngleTick)
{
    float maxDeltaAngle;//如果大于这个角度，则认为发生跳变
    float DeltaAngle;

    if(consecutive_times_ == 0)
    {
        maxDeltaAngle = getMaxDeltaAngle(thisAngleTick.timestamp - last_angle_tick_.timestamp);
        DeltaAngle = thisAngleTick.angle - last_angle_tick_.angle;

        if(fabs(DeltaAngle) > maxDeltaAngle)
        {//跳变了
            consecutive_times_ = 1;
            jump_angle_tick_buffer_[0].angle = thisAngleTick.angle;
            jump_angle_tick_buffer_[0].timestamp = thisAngleTick.timestamp;
        }
        else
        {//连上了
            amendPhase(last_angle_tick_,thisAngleTick);
            last_angle_tick_ = thisAngleTick;
        }
    }
    else
    {
        maxDeltaAngle = getMaxDeltaAngle(thisAngleTick.timestamp - jump_angle_tick_buffer_[consecutive_times_ - 1].timestamp);
        DeltaAngle = thisAngleTick.angle - jump_angle_tick_buffer_[consecutive_times_ - 1].angle;

        if(fabs(DeltaAngle) > maxDeltaAngle)
        {//跳变了
            consecutive_times_ = 0;
        }
        else
        {//连上了
            consecutive_times_++;
            jump_angle_tick_buffer_[consecutive_times_ - 1] = thisAngleTick;

            if(consecutive_times_ == CREDIBLE_CONSECUTIVE_TIMES)
            {
                //lastAngleTick和jumpAngleTickBuffer[0]之间的速度被忽略
                for(int i = 0;i < CREDIBLE_CONSECUTIVE_TIMES - 1;i++)
                {
                    amendPhase(jump_angle_tick_buffer_[i],jump_angle_tick_buffer_[i + 1]);
                }
                last_angle_tick_.angle = thisAngleTick.angle;
                last_angle_tick_.timestamp = thisAngleTick.timestamp;
                consecutive_times_ = 0;
            }
        }
    }
}
float CriticalCore::getMaxDeltaAngle(double timeInterval)
{
    //pos = 0.785 * -1 / 1.884 * cos(1.884 * t) + 1.305 * t - (-1 * 0.785 / 1.884) + pos(0)
    //delta_pos = 0.785 * -1 / 1.884 * [cos(1.884 * (t + timeInterval)) - cos(1.884 * t)] + 1.305 * timeInterval
    float maxDeltaAngle;
    maxDeltaAngle = 0.785 / 0.942 * sin(0.942 * timeInterval) + 1.305 * timeInterval;
    return maxDeltaAngle + 0.1;
}

void CriticalCore::amendPhase(const AngleTick preAngleTick,const AngleTick afterAngleTick)
{
    SpeedTick thisSpeed;

    //算出本帧和上一帧间的速度
    thisSpeed.speed = (afterAngleTick.angle - preAngleTick.angle) / (afterAngleTick.timestamp - preAngleTick.timestamp);
    thisSpeed.timestamp = (afterAngleTick.timestamp + preAngleTick.timestamp) / 2;

    /*调试用的
    static cv::Mat img = cv::Mat::zeros(800,800,CV_8UC3);
    static int i = 0;
    static long double ss = 0;
    ss += thisSpeed.speed;
    std::cout<<thisSpeed.speed<<std::endl;
    cv::circle(img,cv::Point2i(i,thisSpeed.speed * 100 + 400),1,cv::Scalar(255,255,255));
    cv::imshow("img",img);
    i++;
    std::cout<<ss / i<<std::endl;
    */

    //判断正反转并去除正反特性
    if(thisSpeed.speed > 0)
    {
        positive_or_negative_++;
        if(positive_or_negative_ == INT_MAX){positive_or_negative_ = 1e3;}//防止溢出
    }
    else
    {
        positive_or_negative_--;
        if(positive_or_negative_ == INT_MIN){positive_or_negative_ = -1e3;}//防止溢出
    }
    thisSpeed.speed = fabs(thisSpeed.speed);

    //判断大小能量机关
    this->variance_buffer_.push(pow(thisSpeed.speed - 1.047,2));
    this->variance_sum_ += variance_buffer_.back();
    if(variance_buffer_.size() > VARIANCE_BUFFER_MAX)//防止溢出
    {
        this->variance_sum_ -= variance_buffer_.front();
        variance_buffer_.pop();
    }

    if(thisSpeed.speed < 2.090 + 0.1 && thisSpeed.speed > 0.52 - 0.1)//2.09是最大转速
    {
        float a1,a2,b1,b2;
        float sub,subs[4];
        int minSubNum = 0;
        float minSub = FLT_MAX;
        sub = thisSpeed.timestamp - last_speed_.timestamp;
        solveInverseTrigonometricFunction(thisSpeed.speed,a1,a2);
        solveInverseTrigonometricFunction(last_speed_.speed,b1,b2);
        subs[0] = fabs(a1 - b1 - sub);
        subs[1] = fabs(a2 - b2 - sub);
        subs[2] = fabs(a1 - b2 - sub);
        subs[3] = fabs(a2 - b1 - sub);
        for(int i = 0;i<3;i++)
        {
            if(subs[i] < minSub)
            {
                minSubNum = i;
                minSub = subs[i];
            }
        }
        switch(minSubNum)
        {
        case 0:t_inter_ = last_speed_.timestamp - a1;break;
        case 1:t_inter_ = last_speed_.timestamp - a2;break;
        case 2:t_inter_ = last_speed_.timestamp - a1;break;
        case 3:t_inter_ = last_speed_.timestamp - a2;break;
        }
    }
    else
    {

    }
    last_speed_.speed = thisSpeed.speed;
    last_speed_.timestamp = thisSpeed.timestamp;

}
void CriticalCore::solveInverseTrigonometricFunction(float A,float &t1,float &t2)
{
    t1 = asin((A - 1.305) / 0.785) / 1.884;
    if(t1 >=0)
    {
        t2 = 5.0f/3 - t1;
    }
    else
    {
        t2 = -5.0f/3 - t1;
    }
}
float CriticalCore::getFutureAngle(const long double interval)
{
    //delta_pos = 0.785 * -1 / 1.884 * [cos(1.884 * (t + timeInterval)) - cos(1.884 * t)] + 1.305 * timeInterval
    float pos;
    double t = last_angle_tick_.timestamp;

    if(variance_sum_ > 10)
    {
        pos = last_angle_tick_.angle + 0.785 * -1 / 1.884 * (cos(1.884 * (t + interval)) - cos(1.884 * t));
        if(positive_or_negative_ >= 0)
        {
            pos += 1.305 * interval;
        }
        else
        {
            pos -= 1.305 * interval;
        }
    }
    else
    {
        pos = last_angle_tick_.angle;
        if(positive_or_negative_ >= 0)
        {
            pos += 1.047 * interval;
        }
        else
        {
            pos -= 1.047 * interval;
        }
    }
    while(pos > 2 * M_PI)
    {
        pos -= (2 * M_PI);
    }
    return pos;
}

