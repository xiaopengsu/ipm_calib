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


//Computing Pixel Coordinates of brid View,if  from board localy=0,from camera localy=local.y
bool ComputBridViewxy(vector<Point2f> &bridviewxy, float localy =0,float localx = 0)  // localx y m --->cm  320---220 +++
{
    float width_band;
    float dist_board_1, dist_board_2, dist_board_3, dist_board_4;
    int bridview_width,bridview_height;
    float each_pixel_size;
    FileStorage file_storage_scene("../conf/scene_information.yml", FileStorage::READ);
    file_storage_scene["width_band"] >> width_band;
    file_storage_scene["dist_board_1"] >> dist_board_1;
    file_storage_scene["dist_board_2"] >> dist_board_2;
    file_storage_scene["dist_board_3"] >> dist_board_3;
    file_storage_scene["dist_board_4"] >> dist_board_4;
    file_storage_scene["bridview_width"] >> bridview_width; //220
    file_storage_scene["bridview_height"] >> bridview_height; //320
    //file_storage_scene["each_pixel_size"] >> each_pixel_size;
    file_storage_scene.release();
    float width_band_l = -width_band / 2, width_band_r = width_band / 2;
    float dist_board[4] = {dist_board_1, dist_board_2, dist_board_3, dist_board_4};

    //    vector< Point2f >bridviewxy;
    for (int i = 0; i < 4; i++) {
        Point2f bvxy;
        bvxy.x = width_band_l * 10 + (bridview_width/2)-abs(localx*10);//localx*100/10;
        bvxy.y = bridview_height - dist_board[i] * 10-localy*10.0;//localy*100/10;
        bridviewxy.push_back(bvxy);
    }
    for (int i = 0; i < 4; i++) {
        Point2f bvxy;
        bvxy.x = width_band_r * 10 + (bridview_width/2)-abs(localx*10);
        bvxy.y = bridview_height - dist_board[i] * 10 -localy*10.0;//localy*100/10;
        bridviewxy.push_back(bvxy);
    }
    return true;
}

bool computerHomograph(vector<int> ids, vector<Point2f> corner, cv::Mat &Homography,float localy =0,float localx = 0) {


    vector<Point2f> bridviewxy;
    ComputBridViewxy(bridviewxy,localy,localx); // localx  +++  320---220 +++;
    vector<Point2f> usedSrcImgP;
    vector<Point2f> usedBridImgP;
    usedSrcImgP.clear();
    usedBridImgP.clear();

    for (unsigned int i = 0; i < ids.size(); i++) {
        if (3 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[0]);
        }
        if (8 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[1]);
        }
        if (7 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[2]);
        }
        if (34 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[3]);
        }
        if (0 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[4]);
        }
        if (12 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[5]);
        }
        if (2 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[6]);
        }
        if (41 == ids[i]) {
            usedSrcImgP.push_back(corner[i]);
            usedBridImgP.push_back(bridviewxy[7]);
        }
    }
    cout<<"num_src_dst= "<<" "<< usedSrcImgP.size()<<" "<< usedBridImgP.size()<<endl;

    // FIND THE HOMOGRAPHY 4 points   cv::Mat H = cv::getPerspectiveTransform(objPts, imgPts);
    Homography = findHomography(usedSrcImgP, usedBridImgP);
//    H_inv = findHomography(dst, src);
    cout << "Homography  " << endl;
    cout << Homography << endl;
//if( ids.size()<beforenum) //++++++++++
    cv::FileStorage fs;
    if (fs.open("../result/Hbird.xml", FileStorage::WRITE)) {
        fs << "H" << Homography;
        cout << "Write H to ../Hbird.xml" << endl;
    }
    cout << "id_num "<< "="<< ids.size() << endl;
    cout << "usedSrcImgPxy pixel" << endl;
    for (int i = 0; i < usedSrcImgP.size(); i++) {
        cout << usedSrcImgP[i] << endl;
    }
    cout << "usedBridImgPxy pixel" << endl;
    for(int i=0;i<usedBridImgP.size();i++)
    {
        cout << usedBridImgP[i] << endl;
    }
}
