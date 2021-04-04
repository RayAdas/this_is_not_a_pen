#include "robot.h"
#include"coordinateTransform/coordinateTransform.h"
#include <math.h>
//armorModel::armorModel() = default;//这玩意是干嘛的？

#define VISUAL_ROBOT

armorModel::armorModel(coordinatTransform *a)
{
    pnpsolve = a;
}
void armorModel::setInputImage(Mat input) {
    frame = input.clone();
    test = input.clone();
    test2 = input.clone();
    currentCenter.x = 0;
    currentCenter.y = 0;
}
void armorModel::amend(const ImageData* imageData)//修正预测模型
{

    this->setInputImage(imageData->SrcImage);
    this->Pretreatment();

}
cv::Point2f armorModel::getFuturePosition(const float offset)//获得预测点
{
    //cout <<this->GetArmorCenter() <<endl;
    return(this->GetArmorCenter());
}

//图像预处理
void armorModel::Pretreatment() {

    Mat input;
    Point p, center;
    vector<vector<Point>> contours;
    vector<Vec4i> hireachy;
    vector<Rect> boundRect(contours.size());

    double wh=0;  //宽比高
    double hw=0;  //高比宽
//    Point2f vertex[4];

    std::vector<cv::Mat> srcColorChannels;
    cv::split(frame,srcColorChannels);//分离src三通道
    //B,G,R

    if(!EnemyColor)        //red，！blue
    {
        cv::subtract(srcColorChannels[0],srcColorChannels[2],mask);//通道相减
        cv::threshold(mask,mask,55,255,cv::THRESH_BINARY);//通道相减的灰度图进行二值化
    }
    else
    {
       cv::subtract(srcColorChannels[2],srcColorChannels[0],mask);//通道相减
       cv::threshold(mask,mask,65,255,cv::THRESH_BINARY);//通道相减的灰度图进行二值化
    }

    dilate(mask,mask,elementDilate);



    findContours(mask, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    waitKey(1);

    minRects.clear();

    Point2f  vertex[4];
    //这里是在筛选灯条
    for (int i = 0; i < contours.size(); ++i)
    {
        RotatedRect minRect = minAreaRect(Mat(contours[i]));
        //minAreaRect获得轮廓的最小斜矩形

        minRect.points(vertex); //把轮廓的四个点存进vertex数组里
        for (int l = 0; l < 4; l++)
        {
            line(test2, vertex[l], vertex[(l + 1) % 4], Scalar(0, 0, 255), 2);

        }
        if(minRect.size.width > minRect.size.height)   //灯条向右倾斜的情况
        {
            wh =minRect.size.width*1.0/minRect.size.height;
            if(wh >= 4 && wh <= 9)
            {
                //cout << wh <<endl;
                minRects.push_back(minRect);
            }
            else
            {
                continue;
            }
        }

        else if(minRect.size.width < minRect.size.height)   //灯条向左倾斜的情况
        {
            hw = minRect.size.height*1.0 / minRect.size.width;

            if(hw >= 4 && hw <= 9)
            {

                minRects.push_back(minRect);
            }
            else
            {
                continue;
            }
        }



    }
}


Point2f armorModel::GetArmorCenter() {
    //遍历所有矩形，两两组合/*
    RotatedRect Recttemp;
    RotatedRect leftRect, rightRect;
    std::vector<cv::Point2f>xypt;//to storage 4 points which are use for PNP solving
    //vector<int*> reliability;
    vector<cv::Point2i> reliability;
    double area[2], distance, height;
    double leftheight,leftwidth,rightheight,rightwidth;

    if (minRects.size() < 2) {        //minRects存储每一个轮廓的最小斜矩形，minRects.size()<2即存储的最小斜矩形的数量小于2
        target = false;               //表示失去连目标
        LostTarget();
    }

    //对灯条进行一个排序
    std::sort(minRects.begin(),minRects.end(),CmpLight);
    //这里是从符合条件的灯条中匹配对应到两个灯条
    for (int i = 0; i < minRects.size(); i++)
    {
        for (int j = i + 1; j < minRects.size(); j++)
        {

            cv::Point2i temp;
            leftRect = minRects[i];
            rightRect = minRects[j];        //这里到左右只是遍历到顺序，并不是真正到左右灯条
/*
            //省去两个灯条连偏转方向都不同的情况；
            if( leftRect.size.width > leftRect.size.height && rightRect.size.width < rightRect.size.height  )
            {
                cout << 2 <<endl;
                continue;
            }
            if( leftRect.size.width < leftRect.size.height && rightRect.size.width > rightRect.size.height  )
            {
                cout << 2 <<endl;
                continue;
            }
*/
            //方便后面到计算
            //左灯条的两种判断

            if(leftRect.size.width > leftRect.size.height)
            {
                leftheight = leftRect.size.width;
                leftwidth  = leftRect.size.height;
            }
            else if(leftRect.size.width < leftRect.size.height)
            {
                leftheight = leftRect.size.height;
                leftwidth  = leftRect.size.width;
            }

            //右灯条的两种判断
            if(rightRect.size.width > rightRect.size.height)
            {
                rightheight = rightRect.size.width;
                rightwidth  = rightRect.size.height;
            }
            else if(rightRect.size.width < rightRect.size.height)
            {
                rightheight = rightRect.size.height;
                rightwidth  = rightRect.size.width;
            }


            //综合判断
            area[0] = leftRect.size.width * leftRect.size.height;
            area[1] = rightRect.size.width * rightRect.size.height;

            height = (leftheight + rightheight)/2.0;
            distance = Distance(leftRect.center, rightRect.center);

            DBH = distance*1.0/height;
//            cout << DBH <<endl;
/*            cout << abs(leftRect.angle - rightRect.angle) << endl;
            cout << height / abs(leftRect.center.y - rightRect.center.y) <<endl;
            cout << endl;
*/

#ifdef VISUAL_ROBOT
            circle(test2, leftRect.center, 7.5, Scalar(0, 255, 0), 5);
            circle(test2, rightRect.center, 7.5, Scalar(0 ,255, 0), 5);
#endif

            if(abs(leftRect.angle - rightRect.angle) < 30 || abs(leftRect.angle - rightRect.angle) < 96 || abs(leftRect.angle - rightRect.angle) > 84)  //角度判断
            {
                if(min(area[0], area[1]) * 2 > max(area[0], area[1])) //面积判断
                {
                    if (distance != 0 && height!=0 && ( (DBH >= 2.3 && DBH <= 3.2) /*  ||  (DBH >= 2.4 && DBH <= 3.8) */ ) )      //圆心距离和灯条长度到比值
                    {
                        /*  ||  (DBH >= 2.4 && DBH <= 3.7)*/
                        if(height / abs(leftRect.center.y - rightRect.center.y)  > 2 )
                        {


                            /*
                            if(DBH >= 1.2 && DBH <= 2.1)
                            {
                                cout << "小装甲板" <<endl;
                            }
                            else if(DBH >= 2.4 && DBH <= 3.7)
                            {
                                cout << "大装甲板" <<endl;
                            }
                            else
                            {
                                cout << "未辨别出     DBH=" << DBH <<endl;
                            }

                            */

                            temp = cv::Point2i(i,j);

                            reliability.push_back(temp);
                        }
                    }
                }
            }
        }
    }


    if (reliability.empty())
    {


        target = false;
//        cout << "空" <<endl;
        LostTarget();

    }
    else
    {

        target = true;
        lost = 0;
        maxarmorsize=0;
        for (int k = 0; k < reliability.size(); k++)
        {
            DrawArrmor(minRects[reliability[k].x],minRects[reliability[k].y]);


            /*
            cout << Shortest <<endl;
            cout << Longest <<endl;
            cout <<endl;
*/
#ifdef VISUAL_ROBOT
            circle(frame, currentCenter, 7.5, Scalar(255, 255, 0), 5);
            circle(frame, minRects[reliability[k].x].center, 7.5, Scalar(0, 255, 0), 5);
            circle(frame, minRects[reliability[k].y].center, 7.5, Scalar(0 ,255, 0), 5);
#endif
            Point2f  vertex1[4],vertex2[4];
            minRects[reliability[k].x].points(vertex1);
            minRects[reliability[k].y].points(vertex2);

//            cout <<minRects[reliability[k].x].size.height/ minRects[reliability[k].x].size.width<< endl;
//            cout <<minRects[reliability[k].x].size.width/ minRects[reliability[k].x].size.height<< endl;
//            cout << endl;
#ifdef VISUAL_ROBOT
            for (int l = 0; l < 4; l++)
            {
                line(frame, vertex1[l], vertex1[(l + 1) % 4], Scalar(255, 0, 0), 2);

            }
            for (int l = 0; l < 4; l++)
            {
                line(frame, vertex2[l], vertex2[(l + 1) % 4], Scalar(255, 0, 0), 2);

            }
            for (int l = 0; l < 4; l++)
            {
                line(frame, arrmorpoints[l], arrmorpoints[(l + 1) % 4], Scalar(255, 0, 0), 2);

            }
#endif


            arrmorpoint2fs.clear();
            if(armorsense==true)                   //装甲板左偏
            {
                arrmorpoint2fs.emplace_back(arrmorpoints[1]);
                arrmorpoint2fs.emplace_back(arrmorpoints[2]);
                arrmorpoint2fs.emplace_back(arrmorpoints[3]);
                arrmorpoint2fs.emplace_back(arrmorpoints[0]);

            }
            else                                   //装甲板右偏
            {

                arrmorpoint2fs.emplace_back(arrmorpoints[2]);
                arrmorpoint2fs.emplace_back(arrmorpoints[3]);
                arrmorpoint2fs.emplace_back(arrmorpoints[0]);
                arrmorpoint2fs.emplace_back(arrmorpoints[1]);


            }

            //true是校装甲版，false是大装甲版

            pnpsolve->PNP(arrmorpoint2fs,armorType,jvli[k]);
            pnpsolve->PNPcompensateShot(arrmorpoint2fs,armorType,10.5,yaw,pitch);

/*
            cout << pitch <<endl;
            cout << yaw <<endl;
            cout << endl;
*/

        }

        mindistance = jvli[0];
        index = 0;



        for(int i=1;i<reliability.size();i++)
        {
            if(jvli[i]<mindistance)
            {
                mindistance=jvli[i];
                index = i;
            }
        }

        DrawArrmor(minRects[reliability[index].x],minRects[reliability[index].y]);
#ifdef VISUAL_ROBOT
        circle(test, currentCenter, 7.5, Scalar(255, 255, 0), 5);
        circle(test, minRects[reliability[index].x].center, 7.5, Scalar(0, 255, 0), 5);
        circle(test, minRects[reliability[index].y].center, 7.5, Scalar(0 ,255, 0), 5);

        cout << mindistance/1000 <<endl;

        for (int l = 0; l < 4; l++)
        {
            line(test, arrmorpoints[l], arrmorpoints[(l + 1) % 4], Scalar(0, 0, 255), 2);

        }
#endif
        lastCenter = currentCenter;

    }

#ifdef VISUAL_ROBOT

    imshow("mask", mask);
    imshow("frame", frame);
    imshow("test",test);
    imshow("test2",test2);
    waitKey(1);
#endif


    return currentCenter;
}

void armorModel::DrawArrmor(RotatedRect& light1,RotatedRect& light2)
{
    Point2f lightPointsleft[4];
    Point2f lightPointsright[4];
    currentCenter.x = (light1.center.x + light2.center.x) / 2;
    currentCenter.y = (light1.center.y + light2.center.y) / 2;

    //如果算法中到左装机版x坐标比右装甲版x坐标大
    if(light1.center.x > light2.center.x)
    {
        RotatedRect temp = light1;
        light1 = light2;
        light2 = temp;
    }

    light1.points(lightPointsleft);
    light2.points(lightPointsright);


    //右偏到情况
    if(light1.size.width > light1.size.height && light2.size.width > light2.size.height)
    {
        arrmorpoints[0] = lightPointsright[0];
        arrmorpoints[1] = lightPointsleft[1];
        arrmorpoints[2] = lightPointsleft[2];
        arrmorpoints[3] = lightPointsright[3];
        armorsense = false;
        Shortest = Distance(arrmorpoints[1],arrmorpoints[2]);
        Longest = Distance(arrmorpoints[0],arrmorpoints[1]);
    }

    //左偏的情况包含平行到情况
    else if(light1.size.width < light1.size.height && light2.size.width < light2.size.height)
    {
        arrmorpoints[0] = lightPointsleft[0];
        arrmorpoints[1] = lightPointsleft[1];
        arrmorpoints[2] = lightPointsright[2];
        arrmorpoints[3] = lightPointsright[3];
        armorsense = true;
        Shortest = Distance(arrmorpoints[0],arrmorpoints[1]);
        Longest = Distance(arrmorpoints[0],arrmorpoints[3]);
    }


}

void armorModel::LostTarget()
{
    lost++;
    if (lost < 10)
    {
        currentCenter = lastCenter;
    }
    else
    {
        lost = 0;
    }

}



double armorModel::Distance(Point2f a, Point2f b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) +
                (a.y - b.y) * (a.y - b.y));
}

double armorModel::max(double first, double second)
{
    return first > second ? first : second;
}

double armorModel::min(double first, double second)
{
    return first < second ? first : second;
}

bool armorModel::CmpLight(RotatedRect L1,RotatedRect L2)
{
    return L1.center.x<L2.center.x;
}

