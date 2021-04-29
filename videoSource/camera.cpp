#include "camera.h"

Camera::Camera()
{

}
bool Camera::cameraInit()
{
    bool isSuccess;
    //0

    CSystem &ptrCSystem=CSystem::getInstance();//返回引用ptrCSystem
    //1发现设备
    isSuccess = ptrCSystem.discovery(ICameraPtrVector);
    if(isSuccess==false||ICameraPtrVector.size()==0)
    {
        cout <<"未发现摄像头"<<endl;
        return 0;
    }
    cameraSptr=ICameraPtrVector[0];
    //2连接设备
    isSuccess = cameraSptr->connect();
    if(isSuccess==false)
    {
        cout << "摄像头连接失败"<<endl;
        return 0;
    }
    //2.5设置相机参数
    CEnumNode nodeTriggerMode(cameraSptr, "TriggerMode");
    if (false == nodeTriggerMode.isValid())
    {
        printf("get TriggerMode node fail.\n");//属性不可用
        return 0;
    }
    if (false == nodeTriggerMode.setValueBySymbol("Off"))
    {
        printf("set TriggerMode value = Off fail.\n");//设置属性失败
        return 0;
    }
    //3创建流对象
    //setGrabMode(IICameraPtr,1);//设置了相机的抓取模式
    streamPtr=ptrCSystem.createStreamSource(cameraSptr);
    if(streamPtr==NULL)
    {
        cout<< "创建数据流失败"<<endl;
        return 0;
    }
    //4开始采集图像
    isSuccess=streamPtr->startGrabbing();
    if(isSuccess==false)
    {
        cout << "配置从数据流抓取方式失败"<<endl;
        return 0;
    }
    setFrameRate();
    return 1;
}


bool Camera::IGetFrame(cv::Mat &src)
{
    CFrame frame;
    bool getFrameSuccess=streamPtr->getFrame(frame,300);
    if(getFrameSuccess==false)
    {
        cout << "相机读取图像数据失败"<<endl;
        streamPtr->stopGrabbing();
        cameraSptr->disConnect();
        //exit(0);
        return false;
    }
    //判断帧的有效性
    bool isValid = frame.valid();
    if (!isValid)
    {
        printf("frame 是无效的!\n");
        return false;
    }

    int frameBGRSizes=frame.getImageHeight()*frame.getImageWidth()*3;
    uint8_t *frameBGRPtr=new(std::nothrow) uint8_t[frameBGRSizes];

    //设置转换配置参数
    IMGCNV_SOpenParam openParam;
    openParam.width= frame.getImageWidth();
    openParam.height= frame.getImageHeight();
    openParam.paddingX = frame.getImagePadddingX();
    openParam.paddingY = frame.getImagePadddingY();
    openParam.dataSize = frame.getImageSize();
    openParam.pixelForamt = frame.getImagePixelFormat();

    //数据转换
    IMGCNV_EErr status=IMGCNV_ConvertToBGR24((unsigned char *)frame.getImage(),
                                             &openParam,
                                             frameBGRPtr,
                                             &frameBGRSizes);
    if (IMGCNV_SUCCESS != status)
    {
        delete[] frameBGRPtr;
        cout << "转换失败" << endl;
        return false;
    }

    int framewidth=frame.getImageWidth();
    int frameheight=frame.getImageHeight();
    cv::Mat image=cv::Mat(frameheight,framewidth,CV_8UC3,frameBGRPtr);
    resize(image,src,image_size_);
    delete[] frameBGRPtr;
    return true;
}
void Camera::setFrameRate(double rate)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return;
    }

    CBoolNode booleanNode = sptAcquisitionControl->acquisitionFrameRateEnable();
    bRet = booleanNode.setValue(true);
    if (false == bRet)
    {
        printf("set acquisitionFrameRateEnable fail.\n");
        return;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.setValue(rate);
    if (false == bRet)
    {
        printf("set acquisitionFrameRate fail.\n");
        return;
    }
}
void Camera::SetExposeTime(double exp)
{
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        printf("create a IAcquisitionControlPtr failed!\n");
        return;
    }
    CEnumNode eNode = sptrAcquisitionControl->exposureAuto();
    uint64 getValue;
    if (!eNode.getValue(getValue))
    {
        printf("get value of type is failed!\n");
        return;
    }
    if (getValue)//如果开启了自动曝光模式，则关闭
    {
        bRet = eNode.setValueBySymbol("Off");
        if (!bRet)
        {
            printf("close autoExposure failed!\n");
            return;
        }
    }

    CDoubleNode dNode = sptrAcquisitionControl->exposureTime();
    bRet = dNode.setValue(exp);
    if (!bRet)
    {
        printf("set exposure failed!\n");
        return;
    }
    cout << "exposure success"<<endl;
}
void Camera::setBalanceRatio(double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return ;
    }

    /* 关闭自动白平衡 */
    CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return ;
    }

    bRet = enumNode.setValueBySymbol("Off");
    if (false == bRet)
    {
        printf("set balanceWhiteAuto Off fail.\n");
        return ;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return ;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dRedBalanceRatio);
    if (false == bRet)
    {
        printf("set red balanceRatio fail.\n");
        return ;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return ;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dGreenBalanceRatio);
    if (false == bRet)
    {
        printf("set green balanceRatio fail.\n");
        return ;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return ;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dBlueBalanceRatio);
    if (false == bRet)
    {
        printf("set blue balanceRatio fail.\n");
        return ;
    }
}
