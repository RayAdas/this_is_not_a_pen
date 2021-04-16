#include "controller.h"

float GetPitch(float x, float y, float v);
float BulletModel(float x, float v, float angle);
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

            cv::Point3f target;
            cv::Point2f gimbal;
            uart1_keeper_->set(&lingyi,FirePermit);
            uart1_keeper_->set(&lingwu,AOrR);
            active_model_->amend(video_source_->getImage());
            target = active_model_->getFuturePosition(0);;
            if(target.x == -1 || target.y == -1 || target.z == -1)//没找到
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
                //target = camera_transformer_->PCoord2ICoord(target);//注意图像大小
                //target = camera_transformer_->ICoord2CCoord(target);
                gimbal.x = atan(target.x / target.z);
                gimbal.y = atan(target.y / target.z) + GetPitch(target.z,target.y,15);

                gimbal.x = gimbal.x * 180 / M_PI;
                gimbal.y = gimbal.y * 180 / M_PI;
                gimbal.x = (-1) * gimbal.x;
                cout<<gimbal.x<<"||"<<gimbal.y<<endl;
                uart1_keeper_->set(&gimbal.x,YawAngle);
                uart1_keeper_->set(&gimbal.y,PitchAngle);
                uart1_keeper_->write();
            }

        }
        tm.stop();
        //cout<<"time:"<<tm.getTimeMilli()<<endl;
        tm.reset();
    }
}

float GetPitch(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++) {
        a = (float) atan2(y_temp, x);
        y_actual = BulletModel(x, v, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
    }
    return a;

}
float BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float init_k_ = 1;
    float t, y;
    t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return y;
}
