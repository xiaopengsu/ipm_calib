//
// Created by ubuntu on 19-11-11.
//

#ifndef IPM_AUTO_CALIB_FINDCARLOACALPOSE_H
#define IPM_AUTO_CALIB_FINDCARLOACALPOSE_H
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;
void getAngleWithXYZ(double& thetax,double& thetay, double& thetaz, cv::Mat R_matrix);
bool findcarLoacalPose(vector<int> ids, vector<Point2f> corner, vector<double> &Loacal, vector<double> &Pose, double &err_mean);
#endif //IPM_AUTO_CALIB_FINDCARLOACALPOSE_H
