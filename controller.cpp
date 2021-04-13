#include "controller.h"

controller::controller(videoSource *VideoSource,coordinatTransform *CameraTransformer,UartKeeper *uart1)
{
    this->VideoSource = VideoSource;
    this->CameraTransformer = CameraTransformer;
    this->Uart1Keeper = uart1;
    this->BuffModel = new buffModel;
    this->ArmorModel = new armorModel(CameraTransformer);
}

void controller::boot(void)
{
    this->Uart1Keeper->boot();
    mainCycle();
}

void controller::mainCycle()
{
    cv::TickMeter tm;
    while(true)
    {
        tm.start();

        Uart1Keeper->read(AxisData,EnemyColor,AimMod);
        BuffModel->setEnemyColor(*EnemyColor);
        ArmorModel->setEnemyColor(*EnemyColor);
        if(*AimMod == buffMode)
        {
            ActiveModel = BuffModel;
            //cout<<"buff mod active"<<endl;
        }
        else if(*AimMod == robotMode)
        {
            ActiveModel = ArmorModel;
            //cout<<"robot mod active"<<endl;
        }
        else if(*AimMod == manualMode)
        {
            ActiveModel = nullptr;
            //cout<<"manual mod active"<<endl;
        }
        else{}//运行到这就是出错了

        ActiveModel = ArmorModel;
        //ArmorModel->setEnemyColor(teamColor_red);
        if(*AimMod != manualMode)
        {
            const unsigned char lingwu = 0x05;
            const unsigned char lingyi = 0x01;
            const unsigned char lingling = 0x00;

            cv::Point2f target;
            Uart1Keeper->set(&lingyi,FirePermit);
            Uart1Keeper->set(&lingwu,AOrR);
            ActiveModel->amend(VideoSource->getImage());
            target = ActiveModel->getFuturePosition(0);;
            //cout<<"time:"<<tm.getTimeMilli()<<endl;
            if(target.x == -1 ||target.y == -1)//没找到
            {
                Uart1Keeper->set(&lingling,YunTaiMode);
            }
            else
            {
                Uart1Keeper->set(&lingyi,YunTaiMode);
            }
            if(fabs(target.x) < 3 && fabs(target.y) < 3)
            {
                Uart1Keeper->set(&lingyi,FirePermit);
            }
            else
            {
                Uart1Keeper->set(&lingling,FirePermit);
            }
            target = CameraTransformer->PCoord2ICoord(target);//注意图像大小
            target = CameraTransformer->ICoord2CCoord(target);
            target.x = target.x * 180 / M_PI;
            target.y = target.y * 180 / M_PI;
            target.x = (-1) * target.x;
            Uart1Keeper->set(&target.x,YawAngle);
            Uart1Keeper->set(&target.y,PitchAngle);
            Uart1Keeper->write();

        }
        tm.stop();
        //cout<<"time:"<<tm.getTimeMilli()<<endl;
        tm.reset();
    }
}
