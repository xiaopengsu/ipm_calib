/*
相机内参文件：conf/camera.yml
地面坐标系配置文件：conf/scene_information.yml
运行示例：./ipm_calib_video_un -v=../data/5.avi -dy=8 -dx=0
*/


#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <math.h>

#include "computerHomograph.h"
#include "detectIpmPoint_one.h"
#include "findcarLoacalPose.h"
#include "getbirdview.h"

using namespace std;
using namespace cv;

namespace {
    const char *about = "-- ipm auto calib used camera parameter; \n"
                        "-- run example: \n"
                        "./ipm_calib_video -v=../data/5.avi";
    const char *keys =  "{v|../data/5.avi|input from video file or image}";
}


int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return 0;
    }

    String video;
    if(parser.has("v")) {
            video = parser.get<String>("v");
        }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }


    VideoCapture inputVideo;
    int waitTime;
    if (!video.empty()) {
        inputVideo.open(video);

    }
    else {
        cout<<"read video fail,you should check video or path"<<endl;
    }
    Mat OKHomography;Mat H = Mat(3, 3, CV_64FC1);
    int num_ids_temp=4;bool calib_ipm_ok = false;double error_temp=100.0;//big null
    while (inputVideo.grab())
    {
        Mat image, srcImage;
        inputVideo.retrieve(image);
        image.copyTo(srcImage);
        if(false==calib_ipm_ok) {
            waitTime = 0;
            vector<int> ids;
            vector<Point2f> corner;
            detectIpmPoint_one(srcImage, ids, corner);
//        bool calib_ipm_ok = false; int calib_ipm_num=0;
//        Mat OKHomography;Mat H = Mat(3, 3, CV_64FC1);
//        int num_ids_temp=4;
            if (ids.size() > 5) {
                vector<double> Loacal;
                vector<double> Pose;
                double error_mean;
                findcarLoacalPose(ids, corner, Loacal, Pose, error_mean);
                //将文本框居中绘制
                cv::Point origin, origin2;
                origin.x = image.cols / 2;
                origin.y = image.rows / 2;
                origin2.x = origin.x - image.cols / 3;
                origin2.y = origin.y + image.rows / 6;
//            std::string text = "ok!";
//            putText(srcImage, text, origin, 2, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
                //int、float类型都可以塞到stringstream中
                //            float dx= -TT.ptr<double>(0)[0], dy= -TT.ptr<double>(0)[1], dz= -TT.ptr<double>(0)[2];
                float dx = Loacal[0], dy = Loacal[1], dz = Loacal[2];
                double threshold_error_mean = srcImage.rows / 120;//srcImage.rows=720
                if (error_mean < threshold_error_mean) {
                    stringstream strStream;
                    strStream << dx << " " << dy << " " << dz << " " << Pose[0] << " " << Pose[1] << " " << Pose[2];
                    string s = strStream.str();
                    putText(srcImage, s, origin2, 2, 1, cv::Scalar(0, 0, 255), 1, 1, 0);
                    std::string text = "ok!";
                    putText(srcImage, text, origin, 2, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
                    error_temp = error_mean;
                    cout << "ERR ok,err is=" << error_mean << endl;
                    cv::FileStorage fs;
                    std::string tpfileName = "../result/tp_camera_to_floor.xml";//Coordinate transformation between automobile and floor
                    if (fs.open(tpfileName, cv::FileStorage::WRITE)) {
                        fs << "tx" << Loacal[0];
                        fs << "ty" << Loacal[1];
                        fs << "tz" << Loacal[2];
                        fs << "pitch" << Pose[0];
                        fs << "heading" << Pose[1];
                        fs << "roll" << Pose[3];
                        fs << "error_mean" << error_mean;
                        std::cout << "摄像机yu地面坐标关系的标定结果已写入" << tpfileName << std::endl;
                        fs.release();
                    }
//                cout<<"save t r"<<endl;
                }
                if (num_ids_temp < ids.size())//To update  reasult according num of id
                {
                    Mat Homography;
                    // Homography type 0 CV_8UC1
                    computerHomograph(ids, corner, Homography, dy, dx);//computerHomograph(ids, corner, Homography);
                    num_ids_temp = ids.size();
                    OKHomography = Homography;
                    cout << "num_ids_temp= " << num_ids_temp << endl;
                }
                cv::Point origin3;
                origin3.x = image.cols / 2;
                origin3.y = image.rows / 4;
                stringstream strStream;
                strStream << ids.size();//num_ids_temp;
                string s = strStream.str();
                putText(srcImage, s, origin3, 2, 1, cv::Scalar(0, 0, 255), 1, 1, 0);
            }



            Mat bridImage;
            if (!OKHomography.empty())//if (true==calib_ipm_ok)
            {
//
//            Mat H = Mat(3, 3, CV_64FC1);//CV_8UC1 -->CV_64FC1  double  convertTo的用法
                OKHomography.convertTo(H, CV_64FC1);

//            Mat bridImage;
                getbirdview(srcImage, bridImage, H);//  getbirdview(srcImage,bridImage,Homography);
//            imshow("bridImage", bridImage);


                cv::FileStorage fs;
                std::cout << "H : " << OKHomography << std::endl;
                std::string HfileName = "../result/Hbird.xml";
                if (fs.open(HfileName, cv::FileStorage::WRITE)) {
                    fs << "H" << H;
                    std::cout << "H矩阵的标定结果已写入" << HfileName << std::endl;
                    fs.release();
                } else {
                    std::cout << "Could not write the H file" << std::endl;
                    return -1;
                }
                imwrite("../result/srcImage.png", srcImage);
                imwrite("../result/bridImage.png", bridImage);
                cv::waitKey(5);
                cout << "----------ipm calib success !-----------" << endl;
                char key = (char) waitKey(waitTime);
                if ('s' == key) {
                    calib_ipm_ok = true;
                }
            }
        }
        imshow("srcImage", srcImage);
        char key = (char) waitKey(waitTime);
        if(true==calib_ipm_ok)
        {
            waitTime = 1;
            Mat okbridImage;
            getbirdview(srcImage, okbridImage, OKHomography);//  getbirdview(srcImage,bridImage,Homography);
//            imshow("bridImage", bridImage);
        }
        if (key == 27) break;
    }
    return 0;
}
