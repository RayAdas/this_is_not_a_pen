#include "buff.h"
#define VISUAL//开启可视化显示
BuffTick BuffTickUtility::Mat2buffTick(const cv::Mat &src,TeamColor enemyColor)
{

    cv::Mat dst;
    vector<ArmorTick> armorTicks;
    BuffTick thisBuffTick;

#ifdef VISUAL
    cv::Mat visual;
    src.copyTo(visual);
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
            cv::waitKey(1);
        }
    }
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
    cv::waitKey(1);
    cv::Mat elementDilateBule=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(1,3),cv::Point(-1,-1));
    cv::morphologyEx(dst,dst,cv::MORPH_OPEN ,elementDilateBule);
    cout<<"time:"<<tm.getCounter()<<"||"<<tm.getTimeMilli()<<endl;

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
    static bool b = true;
    if(b)
    {
        criti_calcore_.Init(A);
        b = false;
    }
    else
    {
        criti_calcore_.amend(A);
    }
}
BuffModel::BuffModel()
{
}
cv::Point2f BuffModel::getFuturePosition(const float offset)//获得预测点
{
    cv::Point2f futurePosition;
    float angle = criti_calcore_.getFutureAngle(offset);
    if(angle < 0)
    {
        return cv::Point2f(640,512);
    }
    futurePosition.x = this->last_buff_tick_.center.x + this->last_buff_tick_.radius * cos(angle);
    futurePosition.y = this->last_buff_tick_.center.y - this->last_buff_tick_.radius * sin(angle);
    float yangjiao = TrajectoryCalculation::getElevation(BUFF_CENTER_DISTANCE,BUFF_CENTER_HEIGHT + sin(angle) * BUFF_CENTER_RADIUS,25);
    yangjiao -= atan(BUFF_CENTER_HEIGHT / BUFF_CENTER_DISTANCE);

    futurePosition.y = last_buff_tick_.center.y - this->coordinat_transform_->f_ * tan(yangjiao) / this->coordinat_transform_->length_per_pixel_;
    //cout<<futurePosition<<endl;
    return futurePosition;
}
