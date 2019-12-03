//
// Created by ubuntu on 19-11-11.
//

#ifndef IPM_AUTO_CALIB_DETECTIPMPOINT_ONE_H
#define IPM_AUTO_CALIB_DETECTIPMPOINT_ONE_H
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params);
bool detectIpmPoint_one(Mat &srcImage, vector<int> &ids, vector<Point2f> &corner);
#endif //IPM_AUTO_CALIB_DETECTIPMPOINT_ONE_H
