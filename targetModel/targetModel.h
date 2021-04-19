#ifndef TARGETMODEL_H
#define TARGETMODEL_H

#include "videoSource/videosource.h"
#include "serialPort/pmbcbspkeeper.h"
#include "preferences.h"

class TargetModel
{
public:
    TargetModel();
    virtual void amend(ImageData* imageData) = 0;//修正预测模型
    virtual void amend(AxisData* axisData) = 0;//修正预测模型
    void setEnemyColor(TeamColor enemy_color_);
    virtual cv::Point3f getFuturePosition(const float offset) = 0;//获得预测点
protected:
    TeamColor enemy_color_;
};

#endif // TARGETMODEL_H
