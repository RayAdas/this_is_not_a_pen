#ifndef TOOL_H
#define TOOL_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sys/time.h>
void drawRotatedRect(cv::Mat &src,cv::RotatedRect rotRect,const cv::Scalar& color);
double getDistance (cv::Point2f pointO,cv::Point2f pointA );
long int timeval2usec(timeval thisTimeval);

#endif // TOOL_H
