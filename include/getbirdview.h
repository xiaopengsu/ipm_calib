//
// Created by ubuntu on 19-11-11.
//

#ifndef IPM_AUTO_CALIB_GETBIRDVIEW_H
#define IPM_AUTO_CALIB_GETBIRDVIEW_H
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;
void getbirdview(const cv::Mat srcImg, cv::Mat &dstImg, cv::Mat H);
#endif //IPM_AUTO_CALIB_GETBIRDVIEW_H
