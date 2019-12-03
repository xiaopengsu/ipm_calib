#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;
void getAngleWithXYZ(double& thetax,double& thetay, double& thetaz, cv::Mat R_matrix)
{
    double r11 = R_matrix.at<double>(0,0);
    double r12 = R_matrix.at<double>(0,1);
    double r13 = R_matrix.at<double>(0,2);
    double r21 = R_matrix.at<double>(1,0);
    double r22 = R_matrix.at<double>(1,1);
    double r23 = R_matrix.at<double>(1,2);
    double r31 = R_matrix.at<double>(2,0);
    double r32 = R_matrix.at<double>(2,1);
    double r33 = R_matrix.at<double>(2,2);
    thetax = atan2(r32, r33) / CV_PI * 180;
    thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) /CV_PI * 180;
    thetaz = atan2(r21, r11) / CV_PI * 180;
}

// The position of the vehicle is relative to the midpoint position of the two calibration br in the front row.
bool findcarLoacalPose(vector<int> ids, vector<Point2f> corner, vector<double> &Loacal, vector<double> &Pose, double &err_mean)
{
    Mat camera_matrix = Mat(3, 3, CV_32FC1);
    Mat distortion_coefficients = Mat(5, 1, CV_32FC1);
    FileStorage file_storage_camera("../conf/camera.yml", FileStorage::READ);
    file_storage_camera["camera_matrix"] >> camera_matrix;
    file_storage_camera["distortion_coefficients"] >> distortion_coefficients;
    file_storage_camera.release();

    float width_band;
    float dist_board_1, dist_board_2, dist_board_3, dist_board_4;
    FileStorage file_storage_scene("../conf/scene_information.yml", FileStorage::READ);
    file_storage_scene["width_band"] >> width_band;
    file_storage_scene["dist_board_1"] >> dist_board_1;
    file_storage_scene["dist_board_2"] >> dist_board_2;
    file_storage_scene["dist_board_3"] >> dist_board_3;
    file_storage_scene["dist_board_4"] >> dist_board_4;
    file_storage_scene.release();
    float width_band_l = width_band / 2, width_band_r = -width_band / 2;
    float dist_board[4] = {-dist_board_1, -dist_board_2, -dist_board_3, -dist_board_4};

    vector<Point3f> objP;
    vector<Point2f> imgP;
    vector<double> rv(3), tv(3);
    Mat rvec(rv), tvec(tv);
    //vector<Point2f> points;

    for (unsigned int i = 0; i < ids.size(); i++) {
        if (3 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_l, dist_board[0], 0));
        }
        if (8 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_l, dist_board[1], 0));
        }
        if (7 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_l, dist_board[2], 0));
        }
        if (34 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_l, dist_board[3], 0));
        }
        if (0 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_r, dist_board[0], 0));
        }
        if (12 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_r, dist_board[1], 0));
        }
        if (2 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_r, dist_board[2], 0));
        }
        if (41 == ids[i]) {
            imgP.push_back(corner[i]);
            objP.push_back(Point3f(width_band_r, dist_board[3], 0));
        }
    }
//    cout<<"num_3p2p= "<<" "<< objP.size()<<" "<< imgP.size()<<endl;
    solvePnPRansac(objP, imgP, camera_matrix, distortion_coefficients, rvec, tvec, false, SOLVEPNP_EPNP);
    //solvePnP(objP, imgP, camera_matrix, distortion_coefficients, rvec, tvec,false, SOLVEPNP_EPNP);// SOLVEPNP_EPNP
    //computer reproject error to choose good RT
    vector<float> reprojErrs;
//    totalAvgErr = computeReprojectionErrors(objP,imgP,rvec,tvec, camera_matrix, distortion_coefficients,reprojErrs);
    vector<Point2f> imagePoints2;
    double err, mean_err;

    projectPoints(objP, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints2);
    err = norm(Mat(imgP), Mat(imagePoints2), NORM_L2);
    mean_err = err / ids.size();
    err_mean=mean_err;
    Mat rotM, rotMT, TT;
    //旋转向量变旋转矩阵
    Rodrigues(rvec, rotM);
    rotMT = rotM.t();




//    Pose.push_back(rvec.ptr<double>(0)[0]/CV_PI*180); // change
//    Pose.push_back(rvec.ptr<double>(0)[1]/CV_PI*180);
//    Pose.push_back(rvec.ptr<double>(0)[2]/CV_PI*180);


    double pitch, heading, roll;//    double thetax,thetay,thetaz;


    /*ok*/
    getAngleWithXYZ(pitch, heading, roll, rotM);
    Pose.push_back(pitch);
    Pose.push_back(heading);// change
    Pose.push_back(roll);


//    getAngleWithXYZ(pitch, heading, roll, rotMT);
//    Pose.push_back(heading);
//    Pose.push_back(pitch);
//    Pose.push_back(roll);// change

    //tvec 表示世界坐标系的原点在摄像机坐标系的坐标；
    Mat Ttvec = tvec.reshape(0, 3);
    // -R的转置 * t 表示摄像机坐标系的原点在世界坐标系的坐标。
    TT = -rotMT * Ttvec;
    double dx = TT.ptr<double>(0)[0], dy = TT.ptr<double>(0)[1], dz = TT.ptr<double>(0)[2];
    Loacal.push_back(dx);
    Loacal.push_back(dy);
    Loacal.push_back(dz);
    return true;
}
