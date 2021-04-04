#include "tool.h"

void drawRotatedRect(cv::Mat &dst,cv::RotatedRect rotRect,const cv::Scalar& color)
{
    cv::Point2f* vertices = new cv::Point2f[4];
    rotRect.points(vertices);


    for (size_t i = 0; i < 4; i++)
    {
        cv::line(dst, vertices[i], vertices[(i + 1) % 4], color, 2, 8, 0);
    }
}
double getDistance (cv::Point2f pointO,cv::Point2f pointA )

{

    double distance;

    distance = powf((pointO.x - pointA.x),2) + powf((pointO.y - pointA.y),2);

    distance = sqrtf(distance);



    return distance;
}
long int timeval2usec(timeval thisTimeval)
{
    return (thisTimeval.tv_sec * 1e06 + thisTimeval.tv_usec);
}
