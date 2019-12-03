//
// Created by ubuntu on 19-11-11.
//
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;


static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs)
//static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs,int &image_width,int&image_height)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    //    fs["image_width"] >> image_width;
    //    fs["image_height"] >> image_height;
    return true;
}


/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}



bool detectIpmPoint_one(Mat &srcImage, vector<int> &ids, vector<Point2f> &corner) {
    //    int dictionaryId = parser.get<int>("d");
    int dictionaryId = 0;
    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    bool readOk = readDetectorParameters("../conf/detector_params.yml", detectorParams);
    if (!readOk) {
        cerr << "Invalid detector parameters file" << endl;
        return 0;
    }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    vector<vector<Point2f> > rejected;
    // detect markers and estimate pose
    vector<vector<Point2f> > corners;
    aruco::detectMarkers(srcImage, dictionary, corners, ids, detectorParams, rejected);
    //    srcImage.copyTo(srcImage);
    if (ids.size() > 0) //if(ids.size() > 6)
    {
        aruco::drawDetectedMarkers(srcImage, corners, ids);
        for (unsigned int i = 0; i < ids.size(); i++) {
            corner.push_back(corners[i][0]);
            //cout<<" id= "<<ids[i]<<"  corner= "<<corner[i]<<endl;
        }
    }
    //    imshow("out", srcImage);
}