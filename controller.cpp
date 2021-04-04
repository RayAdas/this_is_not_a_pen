#include "controller.h"

controller::controller(videoSource *VideoSource,coordinatTransform *CameraTransformer,serialPort *uart1)
{
    this->VideoSource = VideoSource;
    this->CameraTransformer = CameraTransformer;
    this->Uart1 = uart1;
    this->Uart1Keeper = new PMBCBSPKeeper(uart1);
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
        /*
        Uart1Keeper->read(AxisData,EnemyColor,AimMod);
        BuffModel->setEnemyColor(*EnemyColor);
        ArmorModel->setEnemyColor(*EnemyColor);
        if(*AimMod == buffMod)
        {
            ActiveModel = BuffModel;
            //cout<<"buff mod active"<<endl;
        }
        else if(*AimMod == robotMod)
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
        */

        //if(*AimMod != manualMode)
        //ActiveModel = BuffModel;
        //BuffModel->setEnemyColor(teamColor_red);
        ActiveModel = ArmorModel;
        ArmorModel->setEnemyColor(teamColor_red);
        {

            cv::Point2f target;
            contralData dataA;
            dataA.flags = 1;
            dataA.label = 0X05;

            ActiveModel->amend(VideoSource->getImage());
            target = ActiveModel->getFuturePosition(0);;
            //cout<<"time:"<<tm.getTimeMilli()<<endl;
            target = CameraTransformer->PCoord2ICoord(target);
            target = CameraTransformer->ICoord2CCoord(target);
            target.x = target.x * 180 / M_PI;
            target.y = target.y * 180 / M_PI;
            dataA.yawAngle = (-1) * target.x;
            dataA.pitchAngle = target.y;
            std::cout<<dataA.yawAngle<<"|"<<dataA.pitchAngle<<std::endl;
            //Uart1->send_data(dataA);

        }
        tm.stop();
        //cout<<"time:"<<tm.getTimeMilli()<<endl;
        tm.reset();
    }
}
