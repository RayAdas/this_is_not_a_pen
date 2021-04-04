#include "criticalcore.h"

CriticalCore::CriticalCore()
{
}
void CriticalCore::Init(angleTick initialAngleTick)
{
    this->lastAngleTick = initialAngleTick;
    this->consecutiveTimes = 0;
    this->positiveOrNegative = 0;
    this->lastSpeed.speed = 0;
    this->lastSpeed.timestamp = 0;
    this->t_inter = 0;
    while(!varianceBuffer.empty()){varianceBuffer.pop();}//队列清零
    this->varianceSum = 0;
}

void CriticalCore::amend(angleTick thisAngleTick)
{
    float maxDeltaAngle;//如果大于这个角度，则认为发生跳变
    float DeltaAngle;

    if(consecutiveTimes == 0)
    {
        maxDeltaAngle = getMaxDeltaAngle(thisAngleTick.timestamp - lastAngleTick.timestamp);
        DeltaAngle = thisAngleTick.angle - lastAngleTick.angle;

        if(fabs(DeltaAngle) > maxDeltaAngle)
        {//跳变了
            consecutiveTimes = 1;
            jumpAngleTickBuffer[0].angle = thisAngleTick.angle;
            jumpAngleTickBuffer[0].timestamp = thisAngleTick.timestamp;
        }
        else
        {//连上了
            amendPhase(lastAngleTick,thisAngleTick);
            lastAngleTick = thisAngleTick;
        }
    }
    else
    {
        maxDeltaAngle = getMaxDeltaAngle(thisAngleTick.timestamp - jumpAngleTickBuffer[consecutiveTimes - 1].timestamp);
        DeltaAngle = thisAngleTick.angle - jumpAngleTickBuffer[consecutiveTimes - 1].angle;

        if(fabs(DeltaAngle) > maxDeltaAngle)
        {//跳变了
            consecutiveTimes = 0;
        }
        else
        {//连上了
            consecutiveTimes++;
            jumpAngleTickBuffer[consecutiveTimes - 1] = thisAngleTick;

            if(consecutiveTimes == CREDIBLE_CONSECUTIVE_TIMES)
            {
                //lastAngleTick和jumpAngleTickBuffer[0]之间的速度被忽略
                for(int i = 0;i < CREDIBLE_CONSECUTIVE_TIMES - 1;i++)
                {
                    amendPhase(jumpAngleTickBuffer[i],jumpAngleTickBuffer[i + 1]);
                }
                lastAngleTick.angle = thisAngleTick.angle;
                lastAngleTick.timestamp = thisAngleTick.timestamp;
                consecutiveTimes = 0;
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
    return maxDeltaAngle + 0.01;
}

void CriticalCore::amendPhase(const angleTick preAngleTick,const angleTick afterAngleTick)
{
    speedTick thisSpeed;

    //算出本帧和上一帧间的速度
    thisSpeed.speed = (afterAngleTick.angle - preAngleTick.angle) / (afterAngleTick.timestamp - preAngleTick.timestamp);
    thisSpeed.timestamp = (afterAngleTick.timestamp + preAngleTick.timestamp) / 2;

    //判断正反转并去除正反特性
    if(thisSpeed.speed > 0)
    {
        positiveOrNegative++;
        if(positiveOrNegative == INT_MAX){positiveOrNegative = 1e3;}//防止溢出
    }
    else
    {
        positiveOrNegative--;
        if(positiveOrNegative == INT_MIN){positiveOrNegative = -1e3;}//防止溢出
    }
    thisSpeed.speed = fabs(thisSpeed.speed);

    //判断大小能量机关
    this->varianceBuffer.push(pow(thisSpeed.speed - 1.047,2));
    this->varianceSum += varianceBuffer.back();
    if(varianceBuffer.size() > VARIANCE_BUFFER_MAX)//防止溢出
    {
        this->varianceSum -= varianceBuffer.front();
        varianceBuffer.pop();
    }

    if(thisSpeed.speed < 2.090 + 0.1 && thisSpeed.speed > 0.52 - 0.1)//2.09是最大转速
    {
        float a1,a2,b1,b2;
        float sub,subs[4];
        int minSubNum = 0;
        float minSub = FLT_MAX;
        sub = thisSpeed.timestamp - lastSpeed.timestamp;
        solveInverseTrigonometricFunction(thisSpeed.speed,a1,a2);
        solveInverseTrigonometricFunction(lastSpeed.speed,b1,b2);
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
        case 0:t_inter = lastSpeed.timestamp - a1;break;
        case 1:t_inter = lastSpeed.timestamp - a2;break;
        case 2:t_inter = lastSpeed.timestamp - a1;break;
        case 3:t_inter = lastSpeed.timestamp - a2;break;
        }
    }
    else
    {

    }
    lastSpeed.speed = thisSpeed.speed;
    lastSpeed.timestamp = thisSpeed.timestamp;

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
    double t = lastAngleTick.timestamp;

    if(varianceSum > 10)
    {
        pos = lastAngleTick.angle + 0.785 * -1 / 1.884 * (cos(1.884 * (t + interval)) - cos(1.884 * t));
        if(positiveOrNegative >= 0)
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
        pos = lastAngleTick.angle;
        if(positiveOrNegative >= 0)
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

