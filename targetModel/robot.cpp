#include "robot.h"
#include"coordinateTransform/coordinateTransform.h"
#include <math.h>
//armorModel::armorModel() = default;//这玩意是干嘛的？
#define VISUAL_ROBOT

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
        this->angle=90+another.angle;
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
    //SVM_Params=ml::SVM::load("svmNum.xml");
    //SVM_ArmorTypeParam=ml::SVM::load("armorType.xml");
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
    cout <<this->GetArmorCenter() <<endl;
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
    for(size_t i=0;i<lightCounters.size();i++)
    {
        float max_min=0;
        float widthTemp=0;
        float heightTemp=0;
        RotatedRect RectTemp=minAreaRect(lightCounters[i]);
        widthTemp=(RectTemp.size.width);
        heightTemp=(RectTemp.size.height);

        float PointSize=widthTemp*heightTemp;// 点集的个数小于或者大于某个点数直接舍弃
        if((PointSize<Param.Light_Point_min||PointSize>Param.Light_Point_max))continue;
        else if(PointSize<Param.Light_Point_min*2)
        {
            if(widthTemp>heightTemp)
            {
                float max_minTemp=widthTemp/heightTemp;
                if(max_minTemp<Param.Light_Max_Min2Points)continue;
            }
            else
            {
                float max_minTemp=heightTemp/widthTemp;
                if(max_minTemp<Param.Light_Max_Min2Points)continue;
            }
        }
        // 2.通过灯条长宽比，倾斜角度筛选
        float Max_Light=0;
        float Min_Light=0;
        if(widthTemp>=heightTemp)
        {
            Max_Light=widthTemp;
            Min_Light=heightTemp;
            max_min=Max_Light/Min_Light;
            if(fabs(RectTemp.angle)<35||max_min<Param.Light_Normal_Max_Min2Points)continue;
        }
        else
        {
            Max_Light=heightTemp;
            Min_Light=widthTemp;
            max_min=Max_Light/Min_Light;
            if(fabs(RectTemp.angle)>55||max_min<Param.Light_Normal_Max_Min2Points)continue;
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
        float angleLeftLight=lightCountersRoRect[i].angle;//  旋转矩形的angle范围：[0,90)
        float Score=0;
        float ScoreTemp=0;
        int angleMark=-1;
        float lightRectMaxleftLight=lightCountersRoRect[i].max;
        sense_of_roRect rectTurnLeftLight=lightCountersRoRect[i].sense;
        sense_of_roRect rectTurnRightLightTemp=Lightleft;//   初始化为左
        // 自定义0度为灯条最理想的角度——竖直！！！
        for(size_t j=i+1;j<lightCountersRoRect.size();j++)
        {
            float angleRightLight=lightCountersRoRect[j].angle;
            float lightRectMaxRightLight=lightCountersRoRect[j].max;
            sense_of_roRect rectTurnRightLight=lightCountersRoRect[j].sense;
            float doubleLightAngleDifference=0;


            if(rectTurnLeftLight!=rectTurnRightLight)
            {
                if(angleLeftLight>11)
                {
                    continue;
                }//  不满足直接跳出
                doubleLightAngleDifference=angleLeftLight+angleRightLight;
            }
            else
            {
                if(rectTurnLeftLight==Lightleft)
                {
                    if((lightCountersRoRect[i].center.y-lightCountersRoRect[j].center.y)<-10)
                    {
                        continue;//因为灯条是从左向右排序   因此可以由旋转矩形的角度确定两灯条的中心坐标y值的范围，减少运算
                    }
                }
                else
                {
                    if((lightCountersRoRect[i].center.y-lightCountersRoRect[j].center.y)>10)
                    {
                        continue;
                    }
                }
                doubleLightAngleDifference=fabs(angleLeftLight-angleRightLight);
            }
            //旋转角度最大差
            bool rectAngleFlag=doubleLightAngleDifference<Param.angleij_diffMax ? false:true;
            if(rectAngleFlag)
            {
                continue;
            }

            //两灯条的中心点距离
            float lightRectMax= lightRectMaxleftLight>lightRectMaxRightLight ? lightRectMaxleftLight:lightRectMaxRightLight;
            float lightCenterDistance=Distance(lightCountersRoRect[i].center,lightCountersRoRect[j].center);
            float disRatio=lightCenterDistance/lightRectMax;
            bool rectCenterLenFlagShort=disRatio>Param.disRatio_min ? false:true;//**************************************************************
            bool rectCenterLenFlagLong=disRatio<Param.disRatio_max ? false:true;
            if(rectCenterLenFlagShort||rectCenterLenFlagLong)
            {
                continue;
            }


            //两灯条的形状差别  *************************************************************************

            bool rectLenFlag=fabs(lightRectMaxleftLight-lightRectMaxRightLight)<(lightRectMax/Param.len_diff_kp) ? false:true;
            if(rectLenFlag)
            {
                continue;
            }

            //灯条中心点连线与灯条旋转角度的关系  **********************************************************
            float doubleLightCenterAngle=atan((lightCountersRoRect[i].center.y-lightCountersRoRect[j].center.y)/
                                            (lightCountersRoRect[i].center.x-lightCountersRoRect[j].center.x))/CV_PI*180;
            float doubleLightAngleaAve=0;
            if(rectTurnLeftLight!=rectTurnRightLight)
            {
                doubleLightAngleaAve=fabs(angleLeftLight-angleRightLight)/2;
            }
            else doubleLightAngleaAve=(angleLeftLight+angleRightLight)/2;
            doubleLightAngleaAve=fabs(doubleLightAngleaAve);
            doubleLightCenterAngle=fabs(doubleLightCenterAngle);
            bool angleijCenterFlag=fabs(doubleLightCenterAngle-doubleLightAngleaAve)<Param.angleij_center_diffMax ? false:true;
            if(angleijCenterFlag)
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
            float angleDifferenceScore=(Param.angleij_diffMax-doubleLightAngleDifference)/Param.angleij_diffMax*10;  // 16/10
            float lightSizeDifferenceScore=((lightRectMax/Param.len_diff_kp)-
                                            fabs(lightRectMaxleftLight-lightRectMaxRightLight))/(lightRectMax/Param.len_diff_kp)*8;
            float centerAngleDifferenceScore=(Param.angleij_center_diffMax-fabs(doubleLightCenterAngle-doubleLightAngleaAve))/Param.angleij_center_diffMax*6;
            //三个分数求和
            ScoreTemp=lightSizeDifferenceScore+centerAngleDifferenceScore+angleDifferenceScore;
            if(Score>ScoreTemp)
            {
                continue;
            }
            angleMark=j;
            Score=ScoreTemp;
            rectTurnRightLightTemp=rectTurnRightLight;
        }

        //某一个灯条匹配完成
        if(angleMark!=(-1))//不等于-1表示有符合条件的装甲板
        {
            RotatedRect arrmorTemp;
            Point2f Point4_i[4];
            Point2f Point4_j[4];
            //Point PointArrmor[4];
            vector<Point2f> PointArrmor;
            RotatedRect lightRectTempi=lightCountersRoRect[i].rotatedrect();
            RotatedRect lightRectTempj=lightCountersRoRect[angleMark].rotatedrect();
            lightRectTempi.points(Point4_i);
            lightRectTempj.points(Point4_j);

            if(rectTurnLeftLight==Lightleft)
            {
                PointArrmor.emplace_back(Point4_i[0]);
                PointArrmor.emplace_back(Point4_i[1]);
            }
            else
            {
                PointArrmor.emplace_back(Point4_i[1]);
                PointArrmor.emplace_back(Point4_i[2]);
            }
            if(rectTurnRightLightTemp==Lightleft)
            {
                PointArrmor.emplace_back(Point4_j[2]);
                PointArrmor.emplace_back(Point4_j[3]);
            }
            else
            {
                PointArrmor.emplace_back(Point4_j[3]);
                PointArrmor.emplace_back(Point4_j[0]);
            }
            //根据两灯条自身的长度与两灯条的距离关系筛查灯条
            arrmorTemp=minAreaRect(PointArrmor);

            float arrmorLenMaxTemp=0;
            float arrmorLenMinTemp=0;
            ArmorDescriptor targetArrmorTemp(arrmorTemp);        //用ArmorDescriptor类的构造函数实例化
            if(targetArrmorTemp.armorsense==Lightright)    //如果装甲版向右偏
            {
                arrmorLenMaxTemp=arrmorTemp.size.width;
                arrmorLenMinTemp=arrmorTemp.size.height;
                targetArrmorTemp.size.height=targetArrmorTemp.size.height*2;
            }
            else            //如果装甲版向左偏
            {
                arrmorLenMinTemp=arrmorTemp.size.width;
                arrmorLenMaxTemp=arrmorTemp.size.height;
                targetArrmorTemp.size.width=targetArrmorTemp.size.width*2;
            }
            float arrmorLenMaxMinTemp=arrmorLenMaxTemp/arrmorLenMinTemp    ;
            bool arrmor_LenFlag1=arrmorLenMinTemp<2 ? true:false           ;
            bool arrmor_LenFlag2=arrmorLenMaxMinTemp>5.3 ? true:false      ;
            bool arrmor_LenFlag3=arrmorLenMaxMinTemp<1.5 ? true:false      ;
            if((arrmor_LenFlag1||arrmor_LenFlag2)||arrmor_LenFlag3)
            {
                continue;
            }
            judgeArmorrType(targetArrmorTemp)            ;

            float len=0                                  ;
            getLightLen(PointArrmor,len)                 ;
            targetArrmorTemp.lightLen=len                ;
            //cout << targetArrmorTemp.center <<endl;
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
        Mat dstArmor     ;
        Mat dstArmorGray ;
        Mat dstBinArmor  ;
        Mat dstArmorRed  ;
        Mat dstArmorBule ;


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
                Scalar allArmorPointsColor[4]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,0,255)};
                digtisArmorTemp[i].points(allArmorPoints);
                //drawRotatedangle(src_roi,allArmorPoints,allArmorPointsColor,2);
            }

            targetArrmorTemp=digtisArmorTemp[mask];





            Point2f targetArrmorPoint[4];

            targetArrmorTemp.points(targetArrmorPoint);


            //画出目标装甲板

          //  Scalar targetArmorColor[4]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,0,255)};//彩色
            Scalar targetArmorColor[4]={Scalar(255,0,0),Scalar(255,0,0),Scalar(255,0,0),Scalar(255,0,0)};
            drawRotatedangle(src_roi,targetArrmorPoint,targetArmorColor,2);






            targetArrmor=targetArrmorTemp;// 存储目标装甲板


            recrodArmorStatus(true);      //记录装甲板状态




        }
        else  //没有找到装甲板
        {
            recrodArmorStatus(false);     //记录当前装甲板状态
            targetArrmor.armorType=UNKNOWN_ARMOR;
        }

\

    }
    else  //没有找到装甲板
    {
        recrodArmorStatus(false);
        targetArrmor.armorType=UNKNOWN_ARMOR;
    }


    for(int i =0;i<arrrmor_vector.size();i++)
    {
        Point2f  vertex[4];
        arrrmor_vector[i].points(vertex);

        for (int l = 0; l < 4; l++)
        {
            line(frame, vertex[l], vertex[(l + 1) % 4], Scalar(255, 0, 0), 2);

        }


    }

    imshow("src_roi",src_roi);
    imshow("dst",frame);
    waitKey(1);
    if(targetArrmor.armorType != UNKNOWN_ARMOR)
    {
        return targetArrmor.center;
    }
    else
    {
        return Point(320,256);
    }

}


void armorModel::judgeArmorrType(ArmorDescriptor &a)
{

    if(1)
    {
        a.setArmorrType(SMALL_ARMOR);
    }
    else
    {
        a.setArmorrType(BIG_ARMOR)  ;
    }
}

/*
int arrmor::mySvmArmorTypePredict(float ratio, float angle)
{
    if(angle<-45)
    {
        angle+=90;
    }
    else if(angle<0)
    {
        angle=-angle;
    }
    Mat predictData=(Mat_<float>(1,2) << ratio,angle);
    int result=SVM_ArmorTypeParam->predict(predictData);
    return result;
}
*/
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


