#include "robot.h"
#include"coordinateTransform/coordinateTransform.h"
#include <math.h>

#define VISUAL_ROBOT
#define DRAW_PICTURE
//#define SHOW_PNP_DATA
//#define SHOW_SELECTED_DATA
//#define SHOW_RETURN_DATA

LightDescriptor::LightDescriptor()
{
    this->center.x=0;//初始化0,0点
    this->center.y=0;
    this->max=0;
    this->min=0;
    this->angle=0;
    this->area=0;
    this->sense=Lightleft;//  初始化为左
}
LightDescriptor::LightDescriptor(const cv::RotatedRect& another)
{
    if(another.size.width>another.size.height)// 灯条右向  /* /  */
    {
        this->sense=Lightright;
        this->angle=90+another.angle;                  //这个angle是正常人思维中宽于底边的角度
        this->max=another.size.width;
        this->min=another.size.height;
        this->center.x=another.center.x;
        this->center.y=another.center.y;

    }
    else                                    //  灯条左向  /* \  */
    {
        this->sense=Lightleft;
        this->angle=fabs(another.angle);
        this->max=another.size.height;
        this->min=another.size.width;
        this->center.x=another.center.x;
        this->center.y=another.center.y;
    }
}
LightDescriptor& LightDescriptor::operator =(const LightDescriptor& another)
{
    this->angle=another.angle;
    this->area=another.area;
    this->center.x=another.center.x;
    this->center.y=another.center.y;
    this->sense=another.sense;
    this->max=another.max;
    this->min=another.min;
    return *this;
}

cv::RotatedRect LightDescriptor::rotatedrect()
{
    float angleTemp=0;
    Size2f sizeTemp;
    if(this->sense==Lightright)
    {
        angleTemp=this->angle-90;
        sizeTemp.width=this->max;
        sizeTemp.height=this->min;
    }
    else
    {
        angleTemp=-(this->angle);
        sizeTemp.width=this->min;
        sizeTemp.height=this->max;
    }
    RotatedRect ro(Point2f(this->center),sizeTemp,angleTemp);
    return ro;
}




/**
 * 关于装甲板的函数
*/
ArmorDescriptor::ArmorDescriptor()
{
    this->angle     = 0            ;
    this->center.x  = 0            ;
    this->center.y  = 0            ;
    this->lightLen  = 0            ;
    this->armorType = UNKNOWN_ARMOR;
    this->robot     = Infantry     ;
    this->Longest   = 0            ;
    this->Shortest  = 0            ;
    this->lightLen  = 0            ;
    this->armorsense= Lightleft    ;

    this->size=Size2f(0,0)         ;
    this->armorPoints.resize(4)    ;
    this->pitch=0;;
    this->yaw=0;
    this->distance=0;

}

ArmorDescriptor:: ArmorDescriptor(const RotatedRect& another,ObjectType type,Robot_Type ro)
{
    this->angle     = another.angle   ;
    this->center.x  = another.center.x;
    this->center.y  = another.center.y;
    this->size      = another.size    ;
    this->armorType = type            ;
    this->robot     = ro              ;
    this->lightLen  = 0               ;
    if(another.size.width > another.size.height)
    {
        this->Longest    = another.size.width ;
        this->Shortest   = another.size.height;
        this->armorsense = Lightright         ;
    }
    else
    {
        this->Longest    = another.size.height;
        this->Shortest   = another.size.width ;
        this->armorsense = Lightleft          ;
    }
}
ArmorDescriptor& ArmorDescriptor::operator =(const ArmorDescriptor& another)
{
    this->angle      = another.angle     ;
    this->center.x   = another.center.x  ;
    this->center.y   = another.center.y  ;
    this->size       = another.size      ;
    this->armorType  = another.armorType ;
    this->robot      = another.robot     ;
    this->Longest    = another.Longest   ;
    this->Shortest   = another.Shortest  ;
    this->armorsense = another.armorsense;
    this->lightLen   = another.lightLen  ;
    return *this;
}

RotatedRect ArmorDescriptor::rotatedrect()
{
    RotatedRect temp(this->center,this->size,this->angle);
    return temp;
}

void ArmorDescriptor::setArmorrType(ObjectType set)
{
    this->armorType=set;
}

void ArmorDescriptor::setRobotType(Robot_Type set)
{
    this->robot=set;
}


ArmorModel::ArmorModel(CoordinatTransform *a)
{
    pnpsolve = a;
    ImageSize =Size(640,512);
    offset_point.x = 0;
    offset_point.y = 0;
    EnemyColor=true;
}
void ArmorModel::setInputImage(Mat input) {
    frame = input.clone();
    src_roi = input.clone();
    mask = input.clone();

}
void ArmorModel::amend(ImageData* imageData)//修正预测模型
{

    this->setInputImage(imageData->SrcImage);

    this->Pretreatment();


}
cv::Point3f ArmorModel::getFuturePosition(const float offset)//获得预测点
{
    #ifdef  SHOW_RETURN_DATA
    cout <<this->GetArmorCenter() <<endl;
    #endif
    return(this->GetArmorCenter());

}


void ArmorModel::Pretreatment() {

    Mat input;
    Point p, center;
    vector<vector<Point>> lightCounters;
    vector<Vec4i> hireachy;
    vector<LightDescriptor> RotatedRectTemp;



//图像预处理
    std::vector<cv::Mat> srcColorChannels;
    cv::split(frame,srcColorChannels);//分离src三通道
    //B,G,R
    if(enemy_color_)        //red，！blue
    {
        cv::subtract(srcColorChannels[0],srcColorChannels[2],mask);//通道相减
        cv::threshold(mask,mask,40,255,cv::THRESH_BINARY);//通道相减的灰度图进行二值化
    }
    else
    {
       cv::subtract(srcColorChannels[2],srcColorChannels[0],mask);//通道相减
       cv::threshold(mask,mask,40,255,cv::THRESH_BINARY);//通道相减的灰度图进行二值化
    }
    Mat elementDilate=getStructuringElement(MORPH_RECT,Size(1,3),Point(-1,-1));
    dilate(mask,mask,elementDilate);//膨胀
    //预处理结束


#ifdef VISUAL_ROBOT

    imshow("Orgin_src",frame);
    imshow("mask", mask);
    waitKey(1);
#endif

    RotatedRectTemp.clear();
    lightCountersRoRect.clear();
    //寻找轮廓
    findContours(mask, lightCounters, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for(size_t i=0;i<lightCounters.size();i++)        //遍历找到的轮廓
    {
        float max_min=0;
        float widthTemp=0;
        float heightTemp=0;
        RotatedRect RectTemp=minAreaRect(lightCounters[i]); //得到最小斜矩形
        widthTemp=RectTemp.size.width;
        heightTemp=RectTemp.size.height;

        float PointSize=widthTemp*heightTemp;// 点集的个数小于或者大于某个点数直接舍弃
        if( (PointSize < Light.Light_Point_min || PointSize > Light.Light_Point_max) )  //PointSize小于最小面积阈值大于最大面积阈值就跳出
        {
            continue;
        }
        else if(PointSize < Light.Light_Point_min*2)                //这是灯条离的很远的情况
        {
            if(widthTemp>heightTemp)
            {
                float HW_Temp=widthTemp/heightTemp;
                if(HW_Temp < Light.Light_Max_Min2Points)     //Light_Max_Min2Points=2.3，意思是需要大于2.3，不正常
                {
                    continue;
                }
            }
            else
            {
                float HW_Temp=heightTemp/widthTemp;
                if(HW_Temp < Light.Light_Max_Min2Points)
                {
                    continue;
                }
            }
        }
        // 2.通过灯条长宽比，倾斜角度筛选
        float Max_Side=0;
        float Min_Side=0;
        if(widthTemp >= heightTemp)   //灯条右偏的情况
        {
            Max_Side=widthTemp;
            Min_Side=heightTemp;
            max_min=Max_Side/Min_Side;
            if(fabs(RectTemp.angle) < 35 || max_min < Light.Light_Normal_Max_Min2Points)    //Light_Normal_Max_Min2Points=1.8，意思是需要大于1.8，正常
            {
                continue;
            }
        }
        else                         //灯条左偏的情况
        {
            Max_Side=heightTemp;
            Min_Side=widthTemp;
            max_min=Max_Side/Min_Side;
            if(fabs(RectTemp.angle) > 55 || max_min < Light.Light_Normal_Max_Min2Points)
            {
                continue;
            }
        }
        LightDescriptor lightTemp(RectTemp);
        RotatedRectTemp.emplace_back(lightTemp);

    }
    lightCountersRoRect=RotatedRectTemp;
}


//灯条排序模板
bool CmpLight(LightDescriptor R1,LightDescriptor R2)
{
    return R1.center.x<R2.center.x;
}
//旋转矩形排序模板
bool CmpRect(RotatedRect r1,RotatedRect r2)
{
    return r1.center.x<r2.center.x;
}
//装甲板排序模板
bool CmpArrmor(ArmorDescriptor A1,ArmorDescriptor A2)
{
    return A1.center.x<A2.center.x;
}

//对灯条处理完，开始对灯条进行匹配然后对装甲版做匹配
Point3f ArmorModel::GetArmorCenter()
{
    cv::Point3f tvec(0,0,0);
    //将灯条从左往右排序
    sort(lightCountersRoRect.begin(),lightCountersRoRect.end(),CmpLight);            //lightCountersRoRect存储的是LightDescriptor形的数组

    vector<ArmorDescriptor> arrrmor_vector;             //    待筛选的装甲板存储器

    for(size_t i=0;i<lightCountersRoRect.size();i++)      //遍历排序好了的灯条数组
    {
        float leftlight_angle=lightCountersRoRect[i].angle;//  旋转矩形的angle范围：[0,90)
        float Score=0;
        float ScoreTemp=0;
        int angleMark=-1;
        float leftlight_height=lightCountersRoRect[i].max;
        sense_of_roRect leftlight_sense=lightCountersRoRect[i].sense;
        sense_of_roRect rightlight_senseTemp=Lightleft;         //   初始化为左
        // 自定义0度为灯条最理想的角度——竖直！！！
        for(size_t j=i+1;j<lightCountersRoRect.size();j++)
        {
            float rightlight_angle=lightCountersRoRect[j].angle;
            float rightlight_height=lightCountersRoRect[j].max;
            sense_of_roRect rightlight_sense=lightCountersRoRect[j].sense;
            float doubleLightAngleDifference=0;


            if(leftlight_sense!=rightlight_sense)    //如果两个灯条的偏转方向不同
            {
                if(leftlight_angle > 11)
                {
                    continue;
                }//  不满足直接跳出
                doubleLightAngleDifference=leftlight_angle+rightlight_angle;
            }
            else   //偏转角度相同的情况
            {
                if(leftlight_sense==Lightleft)   //两个都是左偏
                {
                    if((lightCountersRoRect[i].center.y-lightCountersRoRect[j].center.y)<-10)
                    {
                        continue;//因为灯条是从左向右排序   因此可以由旋转矩形的角度确定两灯条的中心坐标y值的范围，减少运算
                    }
                }
                else                            //两个都是右偏
                {
                    if((lightCountersRoRect[i].center.y-lightCountersRoRect[j].center.y)>10)
                    {
                        continue;
                    }
                }
                doubleLightAngleDifference=fabs(leftlight_angle-rightlight_angle);
            }
            //旋转角度最大差
            bool rectAngleFlag=doubleLightAngleDifference < Light.angle_difference ? false:true;
            if(rectAngleFlag)   //如果doubleLightAngleDifference大于angle_difference就continue
            {
                continue;
            }

            //两灯条的中心点距离
            float lightside_max = leftlight_height > rightlight_height ? leftlight_height:rightlight_height;
            float lightCenterDistance = Distance(lightCountersRoRect[i].center,lightCountersRoRect[j].center);
            float DBH = lightCenterDistance/lightside_max;

            bool rectCenterLenFlagShort = DBH > Light.DBH_min ? true:false;
            bool rectCenterLenFlagLong = DBH < Light.DBH_max ? true:false;
            if(rectCenterLenFlagShort==false || rectCenterLenFlagLong==false)
            {
                continue;
            }


            //两灯条的形状差别  *************************************************************************

            bool rectLenFlag=fabs(leftlight_height-rightlight_height) < (lightside_max / Light.height_difference_ratio) ? true:false;
            if(rectLenFlag==false)
            {
                continue;
            }


            //灯条中心点连线与灯条旋转角度的关系  **********************************************************
            float doubleLightCenterAngle=atan((lightCountersRoRect[i].center.y-lightCountersRoRect[j].center.y)/
                                            (lightCountersRoRect[i].center.x-lightCountersRoRect[j].center.x))/CV_PI*180;
            float doubleLightAngleaAve=0;
            if(leftlight_sense!=rightlight_sense)   //偏转角度不同
            {
                doubleLightAngleaAve=fabs(leftlight_angle-rightlight_angle)/2;
            }
            else
            {
                doubleLightAngleaAve=(leftlight_angle+rightlight_angle)/2;
            }
            doubleLightAngleaAve=fabs(doubleLightAngleaAve);
            doubleLightCenterAngle=fabs(doubleLightCenterAngle);
            bool angleijCenterFlag=fabs(doubleLightCenterAngle-doubleLightAngleaAve) < Light.angle_center_difference ? true:false;
            if(angleijCenterFlag==false)
            {
                continue;
            }


            //滤掉两灯条中间存在灯条的装甲板
            //实现方式是：通过寻找装甲板的最高点和中间灯条最低点或者最低点和中间灯条的最高点进行比较。
            if((j-i)!=1)
            {
                bool breakflag=false;
                Point2f PointLeftLight[4];
                Point2f PointRightLight[4];
                lightCountersRoRect[i].rotatedrect().points(PointLeftLight);
                lightCountersRoRect[j].rotatedrect().points(PointRightLight);
                int maxy=PointLeftLight[0].y>PointRightLight[0].y ? PointLeftLight[0].y:PointRightLight[0].y;//y越下越
                int miny=PointLeftLight[2].y<PointRightLight[2].y ? PointLeftLight[2].y:PointRightLight[2].y;//
                int ave_center_y= (lightCountersRoRect[i].rotatedrect().center.y+lightCountersRoRect[j].rotatedrect().center.y)/2;
                //k表示两灯条的中间灯条
                for(size_t k=i+1;k<j;k++)
                {
                    Point2f PointMiddleLight[4];

                    lightCountersRoRect[k].rotatedrect().points(PointMiddleLight);
                    if(lightCountersRoRect[k].rotatedrect().center.y>ave_center_y)
                    {
                        if(maxy>PointMiddleLight[2].y)
                        {
                            breakflag=true;// 中间灯条的最低点和装甲板的最高点进行比较
                            break;
                        }
                    }
                    else
                    {
                         if(miny<PointMiddleLight[0].y)
                         {
                             breakflag=true;// 中间灯条的最高点和装甲板的最低点进行比较
                             break;
                         }
                    }
                }
                if(breakflag)
                {
                    continue;
                }
            }
            //打分
            float angleDifferenceScore=(Light.angle_difference-doubleLightAngleDifference)/Light.angle_difference*10;  // 16/10
            float lightSizeDifferenceScore=((lightside_max/Light.height_difference_ratio)-
                                            fabs(leftlight_height-rightlight_height))/(lightside_max/Light.height_difference_ratio)*8;
            float centerAngleDifferenceScore=(Light.angle_center_difference-fabs(doubleLightCenterAngle-doubleLightAngleaAve))/Light.angle_center_difference*6;
            //三个分数求和
            ScoreTemp=lightSizeDifferenceScore+centerAngleDifferenceScore+angleDifferenceScore;
            if(Score > ScoreTemp)
            {
                continue;
            }
            angleMark=j;
            Score=ScoreTemp;
            rightlight_senseTemp=rightlight_sense;
        }

        //某一个灯条匹配完成
        if(angleMark!=(-1))//不等于-1表示有符合条件的装甲板
        {
            RotatedRect arrmorTemp;
            Point2f Point4_i[4];
            Point2f Point4_j[4];
            //Point PointArrmorTemp[4];
            vector<Point2f> PointArrmorTemp;
            RotatedRect lightRectTempi=lightCountersRoRect[i].rotatedrect();
            RotatedRect lightRectTempj=lightCountersRoRect[angleMark].rotatedrect();
            lightRectTempi.points(Point4_i);
            lightRectTempj.points(Point4_j);

            if(leftlight_sense==Lightleft)      //左灯条左偏
            {
                PointArrmorTemp.emplace_back(Point4_i[0]);
                PointArrmorTemp.emplace_back(Point4_i[1]);
            }
            else                                //左灯条右偏
            {
                PointArrmorTemp.emplace_back(Point4_i[1]);
                PointArrmorTemp.emplace_back(Point4_i[2]);
            }
            if(rightlight_senseTemp==Lightleft) //右灯条左偏
            {
                PointArrmorTemp.emplace_back(Point4_j[2]);
                PointArrmorTemp.emplace_back(Point4_j[3]);
            }
            else                                //右灯条右偏
            {
                PointArrmorTemp.emplace_back(Point4_j[3]);
                PointArrmorTemp.emplace_back(Point4_j[0]);
            }
            //根据两灯条自身的长度与两灯条的距离关系筛查灯条
            arrmorTemp=minAreaRect(PointArrmorTemp);

            float armorHeight=0;
            float armorWidth=0;
            ArmorDescriptor targetArrmorTemp(arrmorTemp);        //用ArmorDescriptor类的构造函数实例化
            if(targetArrmorTemp.armorsense==Lightright)    //如果装甲版向右偏
            {
                armorHeight=arrmorTemp.size.width;
                armorWidth=arrmorTemp.size.height;
                targetArrmorTemp.size.height=targetArrmorTemp.size.height*2;
            }
            else            //如果装甲版向左偏
            {
                armorWidth=arrmorTemp.size.width;
                armorHeight=arrmorTemp.size.height;
                targetArrmorTemp.size.width=targetArrmorTemp.size.width*2;
            }
            float arrmorHBW=armorHeight/armorWidth                 ;
            bool arrmor_LenFlag1=armorWidth < 2 ? true:false       ;
            bool arrmor_LenFlag2=arrmorHBW > 5.0 ? true:false      ;
            bool arrmor_LenFlag3=arrmorHBW < 1.8 ? true:false      ;
            if((arrmor_LenFlag1 == true || arrmor_LenFlag2 == true)||arrmor_LenFlag3 == true)
            {
                continue;
            }
#ifdef SHOW_SELECTED_DATA
            cout << arrmorHBW <<endl;

#endif
            judgeArmorrType(targetArrmorTemp,arrmorHBW);
            cout << targetArrmorTemp.armorType<<endl;

            float length=0;
            getLightLen(PointArrmorTemp,length);
            targetArrmorTemp.lightLen=length;
            //cout << targetArrmorTemp.center <<endl;
            targetArrmorTemp.armorPoints = PointArrmorTemp ;
            arrrmor_vector.emplace_back(targetArrmorTemp) ;

        }

    }
    //装甲版寻找结束

    //开始进行装甲版到筛选
//    ArmorDescriptor targetArrmorTemp;
    //
    if(!arrrmor_vector.empty())   //如果存储装甲板的容器不是空的
    {


        sort(arrrmor_vector.begin(),arrrmor_vector.end(),CmpArrmor);


        //滤掉反光装甲板的影响
        if(arrrmor_vector.size()>1)
        {

            for(size_t i=1;i<arrrmor_vector.size();i++)
            {

                if((arrrmor_vector[i].center.x-arrrmor_vector[i-1].center.x)<(arrrmor_vector[i-1].Longest/2))
                {
                    if(arrrmor_vector[i].center.y < arrrmor_vector[i-1].center.y)// y小的是真的y大的是反光的
                    {
                        arrrmor_vector[i-1]=arrrmor_vector[i];   //i是真
                        arrrmor_vector[i-1].armorPoints = arrrmor_vector[i].armorPoints;

                    }
                    else
                    {
                        arrrmor_vector[i]=arrrmor_vector[i-1];   //i-1是真
                        arrrmor_vector[i].armorPoints = arrrmor_vector[i-1].armorPoints;


                    }
                }
            }

        }

        //只是单纯的从装甲板与图像中心距离的关系找出最合适的装甲板
        //后面筛选出目标装甲板的代码还需要优化
        int mask=0;
        float targetArrmorDistanceTemp=0;


        Point2f srcCenter(ImageSize.width/2,ImageSize.height/2);
        Point2f targetArrmorCenterTemp(arrrmor_vector[mask].center.x+offset_point.x,
                                       arrrmor_vector[mask].center.y+offset_point.y);
        targetArrmorDistanceTemp=Distance(targetArrmorCenterTemp,srcCenter);



        for(size_t i=1;i<arrrmor_vector.size();i++)
        {
            Point2f arrmorCenterTemp(arrrmor_vector[i].center.x+offset_point.x,arrrmor_vector[i].center.y+offset_point.y);
            float arrmorDistanceTemp=Distance(arrmorCenterTemp,srcCenter);


            //与摄像头中心距离
            if((targetArrmorDistanceTemp-arrmorDistanceTemp)>=0)
            {
                mask=i;
                targetArrmorDistanceTemp=arrmorDistanceTemp;
            }

        }

        targetArrmor=arrrmor_vector[mask];



        targetArrmor.armorPoints = arrrmor_vector[mask].armorPoints;






        Point2f targetArrmorPoints[4];

        Point2f targetArrmorPoints_Rectangle[4];
        targetArrmor.points(targetArrmorPoints_Rectangle);

        getArmorImagePoint2f(targetArrmor,targetArrmorPoints);




#ifdef DRAW_PICTURE


        for (int l = 0; l < 4; l++)
        {
            //画四个点图形
            line(frame, targetArrmorPoints[l], targetArrmorPoints[(l + 1) % 4], Scalar(0, 0, 255), 2);

            //画旋转矩形
            line(src_roi, targetArrmorPoints_Rectangle[l], targetArrmorPoints_Rectangle[(l + 1) % 4], Scalar(0, 0, 255), 2);



        }

        //画旋转矩形

#endif

        armorDistance=0;
        yaw=0,pitch=0;

        if(targetArrmor.armorType!=UNKNOWN_ARMOR)
        {
            if(1)   //pnp开关
            {
                if(targetArrmor.armorType==SMALL_ARMOR)
                {
                    pnpsolve->PNP(targetArrmor.armorPoints,true,armorDistance,yaw,pitch);

                }
                else if(targetArrmor.armorType==BIG_ARMOR)

                {
                    pnpsolve->PNP(targetArrmor.armorPoints,false,armorDistance,yaw,pitch);

                }
                targetArrmor.pitch=pitch;
                targetArrmor.yaw  =yaw;
                targetArrmor.distance = armorDistance;

                #ifdef SHOW_PNP_DATA
                cout << "距离：" <<armorDistance/1000 <<endl;
                cout << "yaw：" << yaw <<endl;
                cout << "pitch：" << pitch <<endl;
                cout << endl;
                #endif

            }
        }

    }
    else  //没有找到装甲板
    {

        targetArrmor.armorType=UNKNOWN_ARMOR;

    }




    #ifdef VISUAL_ROBOT
    imshow("旋转矩形目标装甲版",src_roi);
    imshow("原始目标装甲版",frame);
    waitKey(1);
    #endif

    if(targetArrmor.armorType != UNKNOWN_ARMOR)
    {
        return Point2f(targetArrmor.yaw,targetArrmor.pitch);
    }
    else
    {
        return Point2f(-1,-1);
    }


}

void armorModel::getArmorImagePoint2f(ArmorDescriptor &armor, Point2f Points[])
{
    if(armor.armorsense  == Lightright)
    {
        Points[0] = armor.armorPoints[0];
        Points[1] = armor.armorPoints[1];
        Points[2] = armor.armorPoints[2];
        Points[3] = armor.armorPoints[3];
    }
    else
    {
        Points[0] = armor.armorPoints[3];
        Points[1] = armor.armorPoints[0];
        Points[2] = armor.armorPoints[1];
        Points[3] = armor.armorPoints[2];
    }
}



void armorModel::judgeArmorrType(ArmorDescriptor &a,float arrmorHBW)
{

    if(arrmorHBW >=1.6 && arrmorHBW <= 3)
    {
        a.setArmorrType(SMALL_ARMOR);
    }
    else if(arrmorHBW >=3.5 && arrmorHBW <= 5)
    {
        a.setArmorrType(BIG_ARMOR)  ;
    }
    else
    {
        a.setArmorrType(UNKNOWN_ARMOR)  ;
    }
}

double armorModel::Distance(Point2f a, Point2f b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) +
                (a.y - b.y) * (a.y - b.y));
}

/**
 ** 得到装甲板的灯条长度，用于计算距离
 **/

 void  ArmorModel::getLightLen(vector<Point2f> &lightPoint2fs, float &len)
 {
     float leftLength =Distance(lightPoint2fs[0],lightPoint2fs[1]);

     float rightlength=Distance(lightPoint2fs[2],lightPoint2fs[3]);

     len=leftLength > rightlength ? leftLength : rightlength;
 }




