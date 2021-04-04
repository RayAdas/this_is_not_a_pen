#include "trajectoryCalculation.h"

TrajectoryCalculation::TrajectoryCalculation()
{

}

float TrajectoryCalculation::getElevation(float distance,float height,short vel)
{
    //distance = cos(angle) * vel * t
    //height = (sin(angle) * vel + sin(angle) * vel - g * t) * t / 2
    float g = 9.794;
    float b2_4ac;
    float angle1,angle2;


    float c = g*distance*distance/vel/vel/2;

    b2_4ac = sqrt(distance*distance - 4 * c * (c + height));
    angle1 = atan(sqrt((- distance + b2_4ac)/(-2*c)));
    angle2 = atan(sqrt((- distance - b2_4ac)/(-2*c)));
    std::cout<<angle1<<"||"<<angle2<<"||||"<<atan(height/distance)<<std::endl;
    return angle1<angle2?angle1:angle2;
}
