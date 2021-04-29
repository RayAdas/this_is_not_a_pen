#include "buff.h"
//#define VISUAL//开启可视化显示
BuffTick BuffTickUtility::Mat2buffTick(const cv::Mat &src,TeamColor enemyColor)
{

    cv::Mat dst;
    vector<ArmorTick> armorTicks;
    BuffTick thisBuffTick;

#ifdef VISUAL
    cv::Mat visual;
    src.copyTo(visual);
    cv::line(visual,cv::Point2i(0,256),cv::Point2f(640,256),cv::Scalar(255,255,255));
    cv::line(visual,cv::Point2i(320,0),cv::Point2f(320,512),cv::Scalar(255,255,255));
    //cv::imshow("src",visual);
#endif
    BuffTickUtility::preprocess(src,dst,enemyColor);//预处理
#ifdef VISUAL
    cv::namedWindow("after preprocess",cv::WINDOW_NORMAL);
    cv::imshow("after preprocess",dst);
#endif
    std::vector<std::vector<cv::Point>> contours;//提取轮廓
    std::vector<cv::Vec4i> hierarchical;
    cv::findContours(dst,contours,hierarchical,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> rotRects;//所有旋转矩形
    std::vector<int> armorRotRect;//被认为是装甲板的顶级轮廓在rotRects中的下标
    std::vector<int> otherRotRect;//其他顶级轮廓在rotRects中的下标

    {//将顶级父轮廓分为装甲板和其他
        int i;
        cv::RotatedRect rotRect;
        for(i = 0;i< hierarchical.size();i++)
        {
            rotRect = cv::minAreaRect(contours[i]);
            rotRect.angle = rotRect.angle / 180 * M_PI;
            rotRects.push_back(rotRect);//不管三七二十一先存起来
            if(hierarchical[i][3] == -1)//父轮廓未被初始化，即是顶级轮廓
            {
                if(rotRect.size.width * rotRect.size.height >= MIN_ACCEPTABLE_ARROMOR_AREA)//装甲板的顶级父轮廓
                {
                    armorRotRect.push_back(i);
                }
                else
                {
                    otherRotRect.push_back(i);
                }
            }
        }
    }
    {//对每个装甲板矩形转化为装甲板
        ArmorTick thisArmor;
        std::vector<int> next_nums;//下一个轮廓的下标
        float minRectCenterDistance;//中心距离最小的两个旋转矩形的距离
        float thisRectCenterDistance;
        int targetPoint_num;//打击点下标
        int i;
        int next_num;
        for(i = 0;i < armorRotRect.size();i++)
        {
            next_nums.clear();
            next_num = hierarchical[armorRotRect[i]][2];//使其指向第一个子轮廓
            if(next_num == -1){continue;}
            do
            {
                if(rotRects[next_num].size.width * rotRects[next_num].size.height >= MIN_ACCEPTABLE_ARROMOR_SUB_AREA)
                {
                    next_nums.push_back(next_num);
                }
                next_num = hierarchical[next_num][0];//使其指向下一个子轮廓
            }while(next_num != -1);//有下一个轮廓
            if(next_nums.size() == 3)//拥有三个内轮廓，认为是已经激活的装甲板
            {
                int j;
                minRectCenterDistance = FLT_MAX;
                targetPoint_num = -1;
                for(j = 0;j < 3;j++)
                {
                    thisRectCenterDistance = getDistance(rotRects[next_nums[j]].center,rotRects[next_nums[(j+1)%3]].center);
                    if(thisRectCenterDistance < minRectCenterDistance)
                    {
                        minRectCenterDistance = thisRectCenterDistance;
                        targetPoint_num = (j + 2) % 3;
                    }
                }
                //求两个狭长内轮廓的角度均值
                thisArmor.angle = BuffTickUtility::getArmorTickAngle(rotRects[next_nums[targetPoint_num]],rotRects[next_nums[(targetPoint_num + 1) % 3]],rotRects[next_nums[(targetPoint_num + 2) % 3]]);
                thisArmor.targetPoint = rotRects[next_nums[targetPoint_num]].center;
                thisArmor.aromorLight = complete;
                thisArmor.length = rotRects[next_nums[targetPoint_num]].size.width > rotRects[next_nums[targetPoint_num]].size.height ? rotRects[next_nums[targetPoint_num]].size.width : rotRects[next_nums[targetPoint_num]].size.height;
                armorTicks.push_back(thisArmor);
                thisBuffTick.lightCounter++;
#ifdef VISUAL
                cv::RotatedRect A;
                A = rotRects[next_nums[targetPoint_num]];
                A.angle = A.angle * 180 / M_PI;
                drawRotatedRect(visual,A,cv::Scalar(0,255,0));
                cv::circle(visual,thisArmor.targetPoint,3,cv::Scalar(255,0,0),-1);
#endif
            }
            else if(next_nums.size() == 1)//拥有1个内轮廓，认为是需要打击的装甲板
            {
                thisArmor.angle = BuffTickUtility::getArmorTickAngle(rotRects[next_nums[0]]);
                thisArmor.targetPoint = rotRects[next_nums[0]].center;
                thisArmor.aromorLight = target;
                thisArmor.length = rotRects[next_nums[0]].size.width > rotRects[next_nums[0]].size.height ? rotRects[next_nums[0]].size.width : rotRects[next_nums[0]].size.height;
                armorTicks.push_back(thisArmor);
                thisBuffTick.lightCounter++;
#ifdef VISUAL
                cv::RotatedRect A;
                A = rotRects[next_nums[0]];
                A.angle = A.angle * 180 / M_PI;

                cv::circle(visual,thisArmor.targetPoint,3,cv::Scalar(255,0,255),-1);
                drawRotatedRect(visual,A,cv::Scalar(255,255,0));
#endif
            }
            else//内轮廓数目怪异，不再被信任的装甲板
            {

            }
        }
    }
    {//在其它外轮廓中筛选圆心
        int i;
        int j;
        double mark;
        double min_mark = DBL_MAX;
        int min_mark_num = -1;
#ifdef VISUAL
        ArmorTick judge;
        cv::Point2f wishPoint,wishPoint1,wishPoint2;
#endif
        for(i = 0;i < otherRotRect.size();i++)
        {
            mark = 0;
            for(j = 0;j < armorTicks.size();j++)
            {
                switch(armorTicks[j].aromorLight)
                {
                case off:
                    continue;
                case target:
#ifdef VISUAL
                    judge = armorTicks[j];
                    wishPoint1.x = RATIO_OF_R_WIDTH * cos(judge.angle) * judge.length * (-1) + judge.targetPoint.x;
                    wishPoint1.y = RATIO_OF_R_WIDTH * sin(judge.angle) * judge.length + judge.targetPoint.y;
                    wishPoint2.x = RATIO_OF_R_WIDTH * cos(judge.angle + M_PI) * judge.length * (-1) + judge.targetPoint.x;
                    wishPoint2.y = RATIO_OF_R_WIDTH * sin(judge.angle + M_PI) * judge.length + judge.targetPoint.y;
                    cv::circle(visual,wishPoint1,3,cv::Scalar(255,0,255),-1);
                    cv::circle(visual,wishPoint2,3,cv::Scalar(255,0,255),-1);
#endif
                    mark += markACenterLikePoint(rotRects[otherRotRect[i]].center,armorTicks[j]);
                    break;
                case complete:
#ifdef VISUAL
                    judge = armorTicks[j];
                    wishPoint.x = RATIO_OF_R_WIDTH * cos(judge.angle) * judge.length * (-1) + judge.targetPoint.x;
                    wishPoint.y = RATIO_OF_R_WIDTH * sin(judge.angle) * judge.length + judge.targetPoint.y;
                    cv::circle(visual,wishPoint,3,cv::Scalar(0,255,255),-1);
#endif
                    mark += 5 * markACenterLikePoint(rotRects[otherRotRect[i]].center,armorTicks[j]);//加了权重
                    break;
                }
            }
            if(mark < min_mark)
            {
                min_mark_num = i;
                min_mark = mark;
            }
        }
        if(otherRotRect.size()>0) thisBuffTick.center = rotRects[otherRotRect[min_mark_num]].center;
    }
    {//求圆的半径
        int i;
        double sum = 0;
        for(i = 0;i < armorTicks.size();i++)
        {
            sum += getDistance(armorTicks[i].targetPoint,thisBuffTick.center);
        }
        thisBuffTick.radius = sum / armorTicks.size();
    }
    {//重新计算夹角
        int i;
        int subx;
        int suby;
        float preAngle;
        for(i=0;i<armorTicks.size();i++)
        {
            subx = armorTicks[i].targetPoint.x - thisBuffTick.center.x;
            suby = armorTicks[i].targetPoint.y - thisBuffTick.center.y;
            suby *= -1;
            preAngle = atan((float)suby / (float)subx);//经验证，atan(INFINITE)结果为PI/2，不会出错
            preAngle = preAngle;
            if(subx >= 0)
            {
                if(suby >= 0)
                {
                    armorTicks[i].angle = preAngle;
                }
                else
                {
                    armorTicks[i].angle = preAngle + 2 * M_PI;
                }
            }
            else
            {
                if(suby >= 0)
                {
                    armorTicks[i].angle = preAngle + M_PI;
                }
                else
                {
                    armorTicks[i].angle = preAngle + M_PI;
                }
            }

            if(armorTicks[i].aromorLight == target)
            {
                thisBuffTick.angle = armorTicks[i].angle;
            }
        }
    }
#ifdef VISUAL
    {
        int i;
        for(i = 0;i < armorTicks.size();i++)
        {
            //if(armorTicks[i].arromorLight == traget)
            cv::putText(visual,std::to_string(armorTicks[i].angle),armorTicks[i].targetPoint,cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(255,255,255));
            //cv::putText(visual,std::to_string(armorTicks[i].aromorLight),armorTicks[i].targetPoint,cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(255,255,255));

            if(armorTicks.size() > 0)
            {
                cv::circle(visual,thisBuffTick.center,3,cv::Scalar(255,255,255),-1);
                cv::circle(visual,thisBuffTick.center,thisBuffTick.radius,cv::Scalar(255,255,255),2);
            }
            cv::namedWindow("visual",cv::WINDOW_NORMAL);
            cv::imshow("visual",visual);
        }
    }
    cv::waitKey(0);
#endif
    return thisBuffTick;
}


void BuffTickUtility::preprocess(const cv::Mat &src,cv::Mat &dst,TeamColor enemyColor)
{
    timeval Timmer1;
    timeval Timmer2;
    int time;
    std::vector<cv::Mat> srcColorChannels;
    cv::split(src,srcColorChannels);//分离src三通道
    cv::TickMeter tm;
    if(enemyColor)
    {
        tm.start();
        cv::subtract(srcColorChannels[0],srcColorChannels[2],dst);//通道相减
        tm.stop();
        cv::threshold(dst,dst,subThreMinRed,subThreMaxRed,cv::THRESH_BINARY);//通道相减的灰度图进行二值化

    }
    else
    {
        tm.start();
        cv::subtract(srcColorChannels[2],srcColorChannels[0],dst);//通道相减
        tm.stop();
        cv::threshold(dst,dst,subThreMinBule,subThreMaxBule,cv::THRESH_BINARY);//通道相减的灰度图进行二值化

    }
    cv::Mat elementDilateBule=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(1,3),cv::Point(-1,-1));
    cv::morphologyEx(dst,dst,cv::MORPH_OPEN ,elementDilateBule);
    //cout<<"time:"<<tm.getCounter()<<"||"<<tm.getTimeMilli()<<endl;

}

float BuffTickUtility::getArmorTickAngle(const cv::RotatedRect &tragetRotRect,const cv::RotatedRect &rotRect1,const cv::RotatedRect &rotRect2)
{
    float angle1 = -1 * rotRect1.angle;
    float angle2 = -1 * rotRect2.angle;
    {//这一段是rotRect1的
        //如果width较大则不处理，否则加90度
        if(rotRect1.size.height > rotRect1.size.width)
        {angle1 += M_PI_2;}
        if(angle1 < M_PI_4)
        {
            if(tragetRotRect.center.x < rotRect1.center.x)
            {angle1 += M_PI;}
        }
        else if(M_PI_4 <= angle1 && angle1 < 3 * M_PI_4)
        {
            if(tragetRotRect.center.y > rotRect1.center.y)
            {angle1 += M_PI;}
        }
        else//3 * M_PI_4 <= angle1 < M_PI
        {
            if(tragetRotRect.center.x > rotRect1.center.x)
            {angle1 += M_PI;}
        }
    }
    {//这一段是rotRect2的
        //如果width较大则不处理，否则加90度
        if(rotRect2.size.height > rotRect2.size.width)
        {angle2 += M_PI_2;}
        if(angle2 < M_PI_4)
        {
            if(tragetRotRect.center.x < rotRect2.center.x)
            {angle2 += M_PI;}
        }
        else if(M_PI_4 <= angle2 && angle2 < 3 * M_PI_4)
        {
            if(tragetRotRect.center.y > rotRect2.center.y)
            {angle2 += M_PI;}
        }
        else//3 * M_PI_4 <= angle2 < M_PI
        {
            if(tragetRotRect.center.x > rotRect2.center.x)
            {angle2 += M_PI;}
        }
    }
    return (angle1 + angle2) / 2;
}

float BuffTickUtility::getArmorTickAngle(const cv::RotatedRect &tragetRotRect)
{
    float angle = -1 * tragetRotRect.angle;
    if(tragetRotRect.size.height < tragetRotRect.size.width)
    {angle += M_PI_2;}
    return angle;
}

double BuffTickUtility::markACenterLikePoint(cv::Point2f canddidate,const ArmorTick &judge)
{
    switch(judge.aromorLight)
    {
    case complete:
    {
        cv::Point2f wishPoint;
        wishPoint.x = RATIO_OF_R_WIDTH * cos(judge.angle) * judge.length * (-1) + judge.targetPoint.x;
        wishPoint.y = RATIO_OF_R_WIDTH * sin(judge.angle) * judge.length + judge.targetPoint.y;
        double thisDistance = getDistance(canddidate,wishPoint);
        return thisDistance;
    }
    case target:
    {
        cv::Point2f wishPoint1,wishPoint2;
        wishPoint1.x = RATIO_OF_R_WIDTH * cos(judge.angle) * judge.length * (-1) + judge.targetPoint.x;
        wishPoint1.y = RATIO_OF_R_WIDTH * sin(judge.angle) * judge.length + judge.targetPoint.y;
        wishPoint2.x = RATIO_OF_R_WIDTH * cos(judge.angle + M_PI) * judge.length * (-1) + judge.targetPoint.x;
        wishPoint2.y = RATIO_OF_R_WIDTH * sin(judge.angle + M_PI) * judge.length + judge.targetPoint.y;
        double thisDistance1 = getDistance(canddidate,wishPoint1);
        double thisDistance2 = getDistance(canddidate,wishPoint2);
        return std::min(thisDistance1,thisDistance2);
    }
    }
}

void BuffModel::amend(ImageData* imageData)//修正预测模型
{

    this->last_buff_tick_ = BuffTickUtility::Mat2buffTick(imageData->SrcImage,this->enemy_color_);
    this->last_buff_tick_.timestamp = imageData->timestamp;
    AngleTick A;
    A.angle = last_buff_tick_.angle;
    A.timestamp = last_buff_tick_.timestamp.tv_sec + last_buff_tick_.timestamp.tv_usec * 1e-6;

    static long double virtual_time = 0;
    virtual_time += 16.667e-03;
    A.timestamp = virtual_time;
    static bool b = true;
    if(b)
    {
        initAngle(A);
        b = false;
    }
    else
    {
        amendAngle(A);
    }

}
BuffModel::BuffModel(CoordinatTransform* coordinatTransform)
{
    this->coordinat_transform_ = coordinatTransform;
}
BuffModel::~BuffModel()
{
}
cv::Point3f BuffModel::getFuturePosition(const float offset)//获得预测点
{
    cv::Point3f futurePosition;
    float angle = getFutureAngle(offset);
    if(angle < 0)
    {
        return cv::Point3f(-1,-1,-1);
    }
    futurePosition.x = BUFF_CENTER_RADIUS * cos(angle);
    futurePosition.y = BUFF_CENTER_RADIUS * sin(angle) + BUFF_CENTER_HEIGHT;
    futurePosition.z = BUFF_CENTER_DISTANCE;
    return futurePosition;
}
cv::Point2f BuffModel::getCameraCurrentDirection()
{
    cv::Point2f d;
    d = coordinat_transform_->ICoord2CCoord(coordinat_transform_->PCoord2ICoord(last_buff_tick_.center));
    d.x *= -1;
    d.y *= -1;
    d.y += atan(BUFF_CENTER_HEIGHT / BUFF_CENTER_DISTANCE);
    return d;
}
void BuffModel::amend(cv::Point2f* axisData)//buff不使用外部陀螺仪数据修正预测模型
{
    cv::Point2f cameraCurrentDirection = getCameraCurrentDirection();
    axisData->x = cameraCurrentDirection.x;
    axisData->y = cameraCurrentDirection.y;
}

/*=======预测部份=======*/
void BuffModel::initAngle(const AngleTick initialAngleTick)
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

void BuffModel::amendAngle(const AngleTick thisAngleTick)
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
float BuffModel::getMaxDeltaAngle(double timeInterval)
{
    //pos = 0.785 * -1 / 1.884 * cos(1.884 * t) + 1.305 * t - (-1 * 0.785 / 1.884) + pos(0)
    //delta_pos = 0.785 * -1 / 1.884 * [cos(1.884 * (t + timeInterval)) - cos(1.884 * t)] + 1.305 * timeInterval
    float maxDeltaAngle;
    maxDeltaAngle = 0.785 / 0.942 * sin(0.942 * timeInterval) + 1.305 * timeInterval;
    return maxDeltaAngle + 0.1;
}

void BuffModel::amendPhase(const AngleTick preAngleTick,const AngleTick afterAngleTick)
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
void BuffModel::solveInverseTrigonometricFunction(float A,float &t1,float &t2)
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
float BuffModel::getFutureAngle(const long double interval)
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

