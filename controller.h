#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "videoSource/videoSource.h"
#include "serialPort/pmbcbspkeeper.h"
#include "coordinateTransform/coordinateTransform.h"
#include "targetModel/buff.h"
#include "targetModel/robot.h"

class controller
{
public:
    controller(videoSource *,coordinatTransform *,UartKeeper *);
    void mainCycle();
    void boot();
public:

private:
    videoSource *VideoSource = nullptr;
    coordinatTransform *CameraTransformer = nullptr;
    UartKeeper *Uart1Keeper = nullptr;
    targetModel *BuffModel = nullptr;
    targetModel *ArmorModel = nullptr;
    targetModel *ActiveModel = nullptr;

    axisData* AxisData = nullptr;
    teamColor* EnemyColor = nullptr;
    aimMod* AimMod = nullptr;
private:
    void explain(void);

};

#endif // CONTROLLER_H
