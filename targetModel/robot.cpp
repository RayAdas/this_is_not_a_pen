#include "robot.h"
#include"coordinateTransform/coordinateTransform.h"
#include <math.h>

//armorModel::armorModel() = default;//这玩意是干嘛的？
#define VISUAL_ROBOT
//#define DRAW_PICTURE
//#define SHOW_DATA
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
  *用line画框
*/
void drawRotatedangle(Mat& src,Point2f p[],Scalar Color[],int thickness)
{
    line(src,p[3],p[0],Color[0],thickness);//蓝
    line(src,p[0],p[1],Color[1],thickness);//绿
    line(src,p[1],p[2],Color[2],thickness);//红
    line(src,p[2],p[3],Color[3],thickness);
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
    this->armorPoints.resize(4);
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


armorModel::armorModel(coordinatTransform *a)
{
    pnpsolve = a;

    losed_counter=0;
    find_counter=0;
    _armorFindFlag=ARMOR_NO;
    roiImageSize=Size(0,0);

    _trackCounter=0;
    _isTracking=false;
    widthRatio=4;
    heightRatio=4;
    enableDigitsRecognize=false;

    EnemyColor=false;     //false是红色，true是蓝色
}
void armorModel::setInputImage(Mat input) {
    frame = input.clone();
    src_roi = input.clone();
    mask = input.clone();

}
void armorModel::amend(ImageData* imageData)//修正预测模型
{

    this->setInputImage(imageData->SrcImage);

    this->Pretreatment();


}
cv::Point2f armorModel::getFuturePosition(const float offset)//获得预测点
{
//    cout <<this->GetArmorCenter() <<endl;
    return(this->GetArmorCenter());
}


void armorModel::Pretreatment() {

    Mat input;
    Point p, center;
    vector<vector<Point>> lightCounters;
    vector<Vec4i> hireachy;
    vector<LightDescriptor> RotatedRectTemp;



//图像预处理
    std::vector<cv::Mat> srcColorChannels;
    cv::split(frame,srcColorChannels);//分离src三通道
    //B,G,R

    if(!EnemyColor)        //red，！blue
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
Point2f armorModel::GetArmorCenter()
{
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
            /*
            bool rectCenterLenFlagShort = DBH > Light.DBH_min ? false:true;//**************************************************************
            bool rectCenterLenFlagLong = DBH < Light.DBH_max ? false:true;
            if(rectCenterLenFlagShort || rectCenterLenFlagLong)
            {
                continue;
            }
            */
            bool rectCenterLenFlagShort = DBH > Light.DBH_min ? true:false;
            bool rectCenterLenFlagLong = DBH < Light.DBH_max ? true:false;
            if(rectCenterLenFlagShort==false || rectCenterLenFlagLong==false)
            {
                continue;
            }


            //两灯条的形状差别  *************************************************************************
/*
            bool rectLenFlag=fabs(leftlight_height-rightlight_height)<(lightside_max/Light.height_difference_ratio) ? false:true;
            if(rectLenFlag)
            {
                continue;
            }
*/
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
            float arrmorHBW=armorHeight/armorWidth    ;
            bool arrmor_LenFlag1=armorWidth < 2 ? true:false           ;
            bool arrmor_LenFlag2=arrmorHBW > 5.3 ? true:false      ;
            bool arrmor_LenFlag3=arrmorHBW < 1.5 ? true:false      ;
            if((arrmor_LenFlag1 == true || arrmor_LenFlag2 == true)||arrmor_LenFlag3 == true)
            {
                continue;
            }
            judgeArmorrType(targetArrmorTemp);

            float length=0;
            getLightLen(PointArrmorTemp,length);
            targetArrmorTemp.lightLen=length;
            //cout << targetArrmorTemp.center <<endl;
            targetArrmorTemp.armorPoints = PointArrmorTemp;
            arrrmor_vector.emplace_back(targetArrmorTemp);

        }

    }
    //装甲版寻找结束

    //开始进行装甲版到筛选
    ArmorDescriptor targetArrmorTemp;
    //
    vector<ArmorDescriptor> digtisArmorTemp;

    digtisArmorTemp.clear();
    if(!arrrmor_vector.empty())   //如果存储装甲板的容器不是空的
    {
        digtisArmorTemp=arrrmor_vector;

        if(!digtisArmorTemp.empty())       //如果digtisArmorTemp这个数组里面不为空
        {
            //将装甲板按照中心坐标x 从小到大排序
            sort(digtisArmorTemp.begin(),digtisArmorTemp.end(),CmpArrmor);



            //滤掉反光装甲板的影响
            if(digtisArmorTemp.size()>1)
            {

                for(size_t i=1;i<digtisArmorTemp.size();i++)
                {

                    if((digtisArmorTemp[i].center.x-digtisArmorTemp[i-1].center.x)<(digtisArmorTemp[i-1].Longest/2))
                    {
                        if(digtisArmorTemp[i].center.y<digtisArmorTemp[i-1].center.y)// y小的是真的y大的是反光的
                        {
                            digtisArmorTemp[i-1]=digtisArmorTemp[i];


                        }
                        else
                        {
                            digtisArmorTemp[i]=digtisArmorTemp[i-1];


                        }
                    }
                }

            }

            //只是单纯的从装甲板与图像中心距离的关系找出最合适的装甲板
            //后面筛选出目标装甲板的代码还需要优化
            int mask=0;
//            targetArrmorTemp=digtisArmorTemp[mask];
//            targetArrmorTemp.judgeArmorrType();
//            int digtis=ArmorNum[0];


            float targetArrmorDistanceTemp=0;
            Point2f srcCenter(ImageSize.width/2,ImageSize.height/2);
            Point2f targetArrmorCenterTemp(digtisArmorTemp[mask].center.x+offset_roi_point.x,
                                           digtisArmorTemp[mask].center.y+offset_roi_point.y);
            targetArrmorDistanceTemp=Distance(targetArrmorCenterTemp,srcCenter);



            for(size_t i=1;i<digtisArmorTemp.size();i++)
            {
                Point2f arrmorCenterTemp(digtisArmorTemp[i].center.x+offset_roi_point.x,
                                         digtisArmorTemp[i].center.y+offset_roi_point.y);
                float arrmorDistanceTemp=Distance(arrmorCenterTemp,srcCenter);


                //与摄像头中心距离
                if((targetArrmorDistanceTemp-arrmorDistanceTemp)>=0)
                {
                    mask=i;
                    targetArrmorDistanceTemp=arrmorDistanceTemp;
                }

                //画出找到的装甲板
                Point2f allArmorPoints[4];
#ifdef DRAW_PICTURE
                Scalar allArmorPointsColor[4]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,0,255)};
                digtisArmorTemp[i].points(allArmorPoints);
                //drawRotatedangle(src_roi,allArmorPoints,allArmorPointsColor,2);
#endif
            }

            targetArrmorTemp=digtisArmorTemp[mask];
            targetArrmorTemp.armorPoints = digtisArmorTemp[mask].armorPoints;






            Point2f targetArrmorPoint[4];

            targetArrmorTemp.points(targetArrmorPoint);

#ifdef DRAW_PICTURE
            //画出目标装甲板

           // Scalar targetArmorColor[4]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,0,255)};//彩色
            Scalar targetArmorColor[4]={Scalar(255,0,0),Scalar(255,0,0),Scalar(255,0,0),Scalar(255,0,0)};
            drawRotatedangle(src_roi,targetArrmorPoint,targetArmorColor,2);

#endif
            targetArrmor=targetArrmorTemp;// 存储目标装甲板


            recrodArmorStatus(true);      //记录装甲板状态


            Point2f targetArrmorPointTest[4];
            if(targetArrmorTemp.armorsense  == Lightright)
            {
                targetArrmorPointTest[0] = targetArrmorTemp.armorPoints[0];
                targetArrmorPointTest[1] = targetArrmorTemp.armorPoints[1];
                targetArrmorPointTest[2] = targetArrmorTemp.armorPoints[2];
                targetArrmorPointTest[3] = targetArrmorTemp.armorPoints[3];
            }
            else
            {
                targetArrmorPointTest[0] = targetArrmorTemp.armorPoints[3];
                targetArrmorPointTest[1] = targetArrmorTemp.armorPoints[0];
                targetArrmorPointTest[2] = targetArrmorTemp.armorPoints[1];
                targetArrmorPointTest[3] = targetArrmorTemp.armorPoints[2];

            }
            for (int l = 0; l < 4; l++)
            {
                line(src_roi, targetArrmorPointTest[l], targetArrmorPointTest[(l + 1) % 4], Scalar(255, 0, 0), 2);

            }

            armorDistance=0;
            yaw=0,pitch=0;
            if(targetArrmor.armorType!=UNKNOWN_ARMOR)
            {
                if(1)   //pnp开关
                {
                    if(targetArrmor.armorType==SMALL_ARMOR)
                    {
                        pnpsolve->PNP(targetArrmorTemp.armorPoints,true,armorDistance,yaw,pitch);
                        cout << "距离：" <<armorDistance/1000 <<endl;
                        cout << "yaw：" << yaw <<endl;
                        cout << "pitch：" << pitch <<endl;
                        cout << endl;
                    }
                    else if(targetArrmor.armorType==BIG_ARMOR)

                    {
                        pnpsolve->PNP(targetArrmorTemp.armorPoints,false,armorDistance,yaw,pitch);
                        cout << "距离：" <<armorDistance/1000 <<endl;
                        cout << "yaw：" << yaw <<endl;
                        cout << "pitch：" << pitch <<endl;
                        cout << endl;
                    }
#ifdef SHOW_DATA
                    cout << armorDistance/1000.0 <<endl;
                    cout << "yaw:" << yaw <<endl;
                    cout << "pitch:"<< pitch <<endl;
                    cout << endl;
#endif
                }
            }



        }
        else  //没有找到装甲板
        {
            recrodArmorStatus(false);     //记录当前装甲板状态
            targetArrmor.armorType=UNKNOWN_ARMOR;
        }
    }
    else  //没有找到装甲板
    {
        recrodArmorStatus(false);
        targetArrmor.armorType=UNKNOWN_ARMOR;
    }


#ifdef DRAW_PICTURE
    for(int i =0;i<arrrmor_vector.size();i++)
    {
        Point2f  vertex[4];
        arrrmor_vector[i].points(vertex);

        for (int l = 0; l < 4; l++)
        {
            line(frame, vertex[l], vertex[(l + 1) % 4], Scalar(255, 0, 0), 2);

        }


    }
#endif


#ifdef VISUAL_ROBOT

    imshow("src_roi",src_roi);
    imshow("dst",frame);
    waitKey(1);
#endif
    if(targetArrmor.armorType != UNKNOWN_ARMOR)
    {
        return targetArrmor.center;
    }
    else
    {
        return Point(-1,-1);
    }

}


void armorModel::judgeArmorrType(ArmorDescriptor &a)
{

    if(0)
    {
        a.setArmorrType(SMALL_ARMOR);
    }
    else
    {
        a.setArmorrType(BIG_ARMOR)  ;
    }
}


void armorModel::getArmorImagePoint2f(ArmorDescriptor &armor, vector<Point2f> &point2fs)
{
    Point2f armorPoints[4];
    armor.points(armorPoints);

    if(armor.armorsense==Lightleft)        //装甲板左偏
    {
        point2fs.emplace_back(armorPoints[1]);
        point2fs.emplace_back(armorPoints[2]);
        point2fs.emplace_back(armorPoints[3]);
        point2fs.emplace_back(armorPoints[0]);

    }
    else                                   //装甲板右偏
    {
        point2fs.emplace_back(armorPoints[0]);
        point2fs.emplace_back(armorPoints[1]);
        point2fs.emplace_back(armorPoints[2]);
        point2fs.emplace_back(armorPoints[3]);
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

 void  armorModel::getLightLen(vector<Point2f> &lightPoint2fs, float &len)
 {
     float leftLength =Distance(lightPoint2fs[0],lightPoint2fs[1]);

     float rightlength=Distance(lightPoint2fs[2],lightPoint2fs[3]);

     len=leftLength > rightlength ? leftLength : rightlength;
 }


 void armorModel::recrodArmorStatus(bool isFoundArmor)
 {
     if(isFoundArmor)
     {
         _trackCounter++;
         if(_trackCounter==3000)//每3000帧执行一次全局扫描
         {
             _trackCounter=0;
             _armorFindFlag=ARMOR_GLOBAL;
         }

         switch(_armorFindFlag)//通过装甲板的状态确定用何种方式寻找装甲板
         {
             case ARMOR_LOST://上一帧是丢失状态下继续在最近找到装甲板的位置找装甲板
             {
                 _armorFindFlag=ARMOR_LOCAL;
                 break;
             }
             case ARMOR_NO://上一帧是没有找到装甲板的状态下，在全图中寻找装甲板
             {
                 _armorFindFlag=ARMOR_GLOBAL;

                 break;
             }
             default :
             {

                 break;
             }
         }
     }
     else
     {
         if(_armorFindFlag==ARMOR_LOCAL)//上一帧是局部寻找，则标记为丢失状态
         {
             _armorFindFlag=ARMOR_LOST;
         }
         else if(_armorFindFlag==ARMOR_GLOBAL)//上一帧是全局寻找，则标记为没有找到装甲板的状态状态
         {

             _armorFindFlag=ARMOR_NO;
         }
     }
 }


