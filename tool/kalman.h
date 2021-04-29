#ifndef KALMAN_H
#define KALMAN_H
#include<cstdio>
#include<cmath>
#include<queue>
#include<deque>
#include<vector>
#include<opencv2/core.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;
class KALMAN{
private:
    RNG rng;
    int stateNum,measureNum;
    KalmanFilter KF;
    Mat measurement,prediction;
    const int h=512;
    const int w=640;
    Point predict_p;
public:
    Point2f KALMAN_FIL(Point2f P);
    void INI_KALMAN();
};

Point2f clean_wave(Point2f p,int n);
#endif // KALMAN_H
