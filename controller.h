#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "videoSource/videosource.h"
#include "serialPort/pmbcbspkeeper.h"
#include "coordinateTransform/coordinateTransform.h"
#include "targetModel/buff.h"
#include "targetModel/robot.h"

class Controller
{
public:
    Controller(VideoSource *,CoordinatTransform *,UartKeeper *);
    void mainCycle();
    void boot();
public:

private:
    VideoSource *video_source_ = nullptr;
    CoordinatTransform *camera_transformer_ = nullptr;
    UartKeeper *uart1_keeper_ = nullptr;
    TargetModel *buff_model_ = nullptr;
    TargetModel *armor_model_ = nullptr;
    TargetModel *active_model_ = nullptr;

    axisData* axis_data_ = nullptr;
    TeamColor* enemy_color_ = nullptr;
    aimMod* aim_mode_ = nullptr;
private:
    void explain(void);

};

#endif // CONTROLLER_H
