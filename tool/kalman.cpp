#include "kalman.h"
void KALMAN::INI_KALMAN(){
       stateNum=4;
       measureNum=2;
       KF.init(4,2,0);
       KF.transitionMatrix = (Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
       setIdentity(KF.measurementMatrix);                                             //测量矩阵H
       setIdentity(KF.processNoiseCov, Scalar::all(1e-1));                            //系统噪声方差矩阵Q
       setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
       setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
       rng.fill(KF.statePost,RNG::UNIFORM,0,h>w?w:h);   //初始状态值x(0)
       measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
}

Point2f  KALMAN::KALMAN_FIL(Point2f P){
    if(isnan(P.x)||isnan(P.y))return cv::Point2f(0,0);
    prediction=KF.predict();
    measurement.at<float>(0) = (float)P.x;
    measurement.at<float>(1) = (float)P.y;
    KF.correct(measurement);
    predict_p=Point(KF.statePost.at<float>(0),KF.statePost.at<float>(1));
    return predict_p;
}
