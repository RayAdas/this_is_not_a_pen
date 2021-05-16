#include "controller.h"

float GetPitch(float x, float y, float v);
float BulletModel(float x, float v, float angle);
Controller::Controller(VideoSource *videoSource,CoordinatTransform *CameraTransformer,UartKeeper *uart1)
{
    this->video_source_ = videoSource;
    this->camera_transformer_ = CameraTransformer;
    this->uart1_keeper_ = uart1;
    this->buff_model_ = new BuffModel(CameraTransformer);
    this->armor_model_ = new ArmorModel(CameraTransformer);
}

void Controller::boot(void)
{
    this->uart1_keeper_->boot();
    mainCycle();
}

void Controller::mainCycle()
{
    static ImageData* lastImageData_p;//上一个ImageData指针，用于在摄像头慢于图像处理时，同步摄像头与图像处理
    cv::TickMeter tm;

    ImageData imgd;
    //cv::VideoCapture vc("/home/awaki/Videos/red.avi");
    //cv::VideoCapture vc("/home/awaki/Videos/Video_2021_01_18_133731_1.avi");
    cv::VideoCapture vc("/home/awaki/Videos/Video_2021_05_02_005441_1.avi");
    while(true)
    {
        tm.start();
        uart1_keeper_->read(&axis_data_,enemy_color_,aim_mode_,&projectile_vel_);


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

        uart1_keeper_->set(&ZERO_FIVE,AOrR);

        *aim_mode_ = buffMode;
        active_model_ = buff_model_;
        if(*aim_mode_ != manualMode)
        {
            //cout<<"vel:"<<axis_data_->ProjectileVel<<endl;
            //cout<<"RA:"<<axis_data_.x<<"||"<<axis_data_.y<<endl;
            //axis_data_.x = 0;
            cv::Point3f target;
            cv::Point2f gimbal(0,0);

            if(*aim_mode_ == buffMode)//步兵英雄限定,我们至今不知道为什么电控要分机械角模式和陀螺仪模式
            {
                uart1_keeper_->set(&ZERO_FIVE,AOrR);//有可能设置反了，需要确认一下
            }
            else
            {
                uart1_keeper_->set(&ZERO_SIX,AOrR);//有可能设置反了，需要确认一下
            }

            uart1_keeper_->set(&ZERO_FIVE,AOrR);//有可能设置反了，需要确认一下

            ImageData* ImageData_p;
            do
            {
                ImageData_p= video_source_->getImage();
            }
            while(ImageData_p == lastImageData_p);//等待直到相机读入新数据
            lastImageData_p = ImageData_p;
            active_model_->amend(ImageData_p);


//            vc >> imgd.SrcImage;
//            cv::resize(imgd.SrcImage,imgd.SrcImage,cv::Size2i(640,512));
//            active_model_->amend(&imgd);

            active_model_->amend(&axis_data_);
            target = active_model_->getFuturePosition(0);
            //cout<<target<<endl;
            if(target.x == -1 || target.y == -1 || target.z == -1)
            {//没找到
                float fz = 0;
                uart1_keeper_->set(&ZERO_ZERO,YunTaiMode);//哨兵限定：进入巡逻模式
                uart1_keeper_->set(&ZERO_ZERO,FirePermit);
                uart1_keeper_->set(&fz,YawAngle);//此时gimbal为0
                uart1_keeper_->set(&fz,PitchAngle);//此时gimbal为0
                uart1_keeper_->write();
            }
            else
            {//找到目标
                //进行云台控制计算
                gimbal.x = atan(target.x / target.z)
                        - axis_data_.x;
                if(projectile_vel_ == 0)
                {
                    projectile_vel_ = 12;
                }
                //target.y = -0.12;
                gimbal.y = TrajectoryCalculation::getElevation(target.z,target.y,projectile_vel_)//*projectile_vel_
                        - axis_data_.y;

                gimbal.x = gimbal.x * 180 / M_PI;
                gimbal.y = gimbal.y * 180 / M_PI;
                gimbal.x -= 1.2;
                gimbal.y += 1.0;

                cout<<gimbal.x<<"||"<<gimbal.y<<endl;

                gimbal.x *= -1;
                gimbal.y *= -1;
                if(fabs(gimbal.x) < 3 && fabs(gimbal.y) < 3)
                {
                    uart1_keeper_->set(&ZERO_ONE,FirePermit);
                }
                else
                {
                    uart1_keeper_->set(&ZERO_ZERO,FirePermit);
                }
                uart1_keeper_->set(&ZERO_ONE,YunTaiMode);//哨兵限定：进入自瞄模式
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
