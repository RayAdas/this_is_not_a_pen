#ifndef TRAJECTORYCALCULATION_H
#define TRAJECTORYCALCULATION_H
#include <math.h>
#include <iostream>

class TrajectoryCalculation
{
public:
    TrajectoryCalculation();
    static float getElevation(float distance,float height,short vel);//distance恒为正;height比自身高的为正;自身基准点为两轴交点;失败返回-90
};

#endif // TRAJECTORYCALCULATION_H
