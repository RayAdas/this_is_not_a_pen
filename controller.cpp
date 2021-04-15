#include "controller.h"

Controller::Controller(VideoSource *videoSource,CoordinatTransform *CameraTransformer,UartKeeper *uart1)
{
    this->video_source_ = videoSource;
    this->camera_transformer_ = CameraTransformer;
    this->uart1_keeper_ = uart1;
    this->buff_model_ = new BuffModel;
    this->armor_model_ = new ArmorModel(CameraTransformer);
}

void Controller::boot(void)
{
    this->uart1_keeper_->boot();
    mainCycle();
}

void Controller::mainCycle()
{
    cv::TickMeter tm;
    while(true)
    {
        tm.start();

        uart1_keeper_->read(axis_data_,enemy_color_,aim_mode_);
        buff_model_->setEnemyColor(*enemy_color_);
        armor_model_->setEnemyColor(*enemy_color_);
        if(*aim_mode_ == buffMode)
        {
            active_model_ = buff_model_;
            //cout<<"buff mod active"<<endl;
        }
        else if(*aim_mode_ == robotMode)
        {
            active_model_ = armor_model_;
            //cout<<"robot mod active"<<endl;
        }
        else if(*aim_mode_ == manualMode)
        {
            active_model_ = nullptr;
            //cout<<"manual mod active"<<endl;
        }
        else{}//运行到这就是出错了

        if(*aim_mode_ != manualMode)
        {
            const unsigned char lingwu = 0x05;
            const unsigned char lingyi = 0x01;
            const unsigned char lingling = 0x00;

            cv::Point2f target;
            uart1_keeper_->set(&lingyi,FirePermit);
            uart1_keeper_->set(&lingwu,AOrR);
            active_model_->amend(video_source_->getImage());
            target = active_model_->getFuturePosition(0);;
            if(target.x == -1 ||target.y == -1)//没找到
            {
                uart1_keeper_->set(&lingling,YunTaiMode);
                uart1_keeper_->set(&lingling,FirePermit);
            }
            else
            {
                uart1_keeper_->set(&lingyi,YunTaiMode);
                if(fabs(target.x) < 3 && fabs(target.y) < 3)
                {
                    uart1_keeper_->set(&lingyi,FirePermit);
                }
                else
                {
                    uart1_keeper_->set(&lingling,FirePermit);
                }
                target = camera_transformer_->PCoord2ICoord(target);//注意图像大小
                target = camera_transformer_->ICoord2CCoord(target);
                target.x = target.x * 180 / M_PI;
                target.y = target.y * 180 / M_PI;
                target.x = (-1) * target.x;
                cout<<target.x<<"||"<<target.y<<endl;
                uart1_keeper_->set(&target.x,YawAngle);
                uart1_keeper_->set(&target.y,PitchAngle);
                uart1_keeper_->write();
            }

        }
        tm.stop();
        //cout<<"time:"<<tm.getTimeMilli()<<endl;
        tm.reset();
    }
}
