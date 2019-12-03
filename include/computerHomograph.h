//
// Created by ubuntu on 19-11-11.
//

#ifndef IPM_AUTO_CALIB_COMPUTERHOMOGRAPH_H
#define IPM_AUTO_CALIB_COMPUTERHOMOGRAPH_H
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
using namespace std;
using namespace cv;
bool ComputBridViewxy(vector<Point2f> &bridviewxy, float localy =0,float localx = 0);  // localx y m --->cm  320---220 +++
bool computerHomograph(vector<int> ids, vector<Point2f> corner, cv::Mat &Homography,float localy =0,float localx = 0);
#endif //IPM_AUTO_CALIB_COMPUTERHOMOGRAPH_H
