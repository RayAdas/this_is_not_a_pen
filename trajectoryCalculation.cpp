#include "trajectoryCalculation.h"

TrajectoryCalculation::TrajectoryCalculation()
{

}

float TrajectoryCalculation::getElevation(float distance,float height,short vel)
{
    //distance = cos(angle) * vel * t
    //height = (sin(angle) * vel + sin(angle) * vel - g * t) * t / 2
    const float g = 9.794;
    float angle1,angle2;
    float a = -1 * g * 0.5 * distance * distance / vel / vel,
            b = distance,
            c = -1 * g * 0.5 * distance * distance / vel / vel - height;
    float melt = b * b - 4 * a * c;
    if(melt > 0)
    {
    angle1 = atan((-b + sqrt(melt)) / (2 * a));
    angle2 = atan((-b - sqrt(melt)) / (2 * a));
    }
    else if(melt == 0)
    {
        angle1 = atan((-b)/(2*a));
        angle2 = angle1;
    }
    else if(melt < 0)
    {
        angle1 = sqrt(melt);//将会返回nan
        angle2 = angle1;
    }

    //std::cout<<angle1<<"||"<<angle2<<"||||"<<atan(height/distance)<<std::endl;
    return angle1<angle2?angle1:angle2;
}
