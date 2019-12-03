//
// Created by ubuntu on 19-11-11.
//

#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;

void getbirdview(const cv::Mat srcImg, cv::Mat &dstImg, cv::Mat H) {

    int bridview_width,bridview_height;
    FileStorage file_storage_scene("../conf/scene_information.yml", FileStorage::READ);
    file_storage_scene["bridview_width"] >> bridview_width; //220
    file_storage_scene["bridview_height"] >> bridview_height; //320
    file_storage_scene.release();
    // set size of display image in screem
    cv::Size birdviewsize(bridview_width,bridview_height);//cv::Size birdviewsize(1280,720); //220-320
    //read camera instrisic

    Mat cameraMatrix = Mat(3, 3, CV_32FC1);
    Mat distCoeffs = Mat(5, 1, CV_32FC1);
    FileStorage file_storage_camera("../conf/camera.yml", FileStorage::READ);
    file_storage_camera["camera_matrix"] >> cameraMatrix;
    file_storage_camera["distortion_coefficients"] >> distCoeffs;
    file_storage_camera.release();


    Size imageSize;
    Mat tempImg,map1, map2;
    imageSize = srcImg.size();

    double alpha=1;//0 kuo, 1 suo
    Size rectificationSize = imageSize;
    Rect validPixROI;
    cv::Mat P;
    P= getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,
                                 imageSize,
                                 alpha,//0 huo, 1 suo
                                 rectificationSize,//undistorted image size
                                 &validPixROI//undistorted image rectangle in source image
    );//new camera matirx

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            P,//undistort image camera instrinsic matirx
                            rectificationSize,//undistorted image size
                            CV_16SC2, map1, map2);

    remap(srcImg, tempImg, map1, map2, INTER_LINEAR);
    cv::Mat imgBirdView= cv::Mat::zeros(birdviewsize, CV_8UC1);
    warpPerspective(tempImg, imgBirdView, H, birdviewsize);//srcImg

//   /*ok*/
//    cv::Mat imgBirdView= cv::Mat::zeros(birdviewsize, CV_8UC1);
//    warpPerspective(srcImg, imgBirdView, H, birdviewsize);//srcImg

    /*have question  parameter  -->  cv::BORDER_CONSTANT  */
//    cv::warpPerspective(srcImg,            // Source image
//                        imgBirdView,    // Output image
//                        H,              // Transformation matrix
//                        birdviewsize,   // Size for output image
//                        cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
//                        cv::BORDER_CONSTANT, cv::Scalar::all(0) // Fill border with black
//    );

//    cv::imshow("Birds_Eye", imgBirdView);
    dstImg = imgBirdView.clone();

    //add car logo
    Mat carlogo;
    carlogo=imread("../conf/car.jpg",1);
    if(!carlogo.empty())
    {
        int carlogo_w=int(bridview_width/8);int carlogo_h=int(bridview_height/8);
        cv::Size carlogosize(carlogo_w,carlogo_h);//cv::Size birdviewsize(1280,720); //220-320
        resize(carlogo,carlogo,carlogosize);//
        // 0.5 0.5的比例缩放； cv::resize(img, dst,cv::Size(0,0),(0.5),(0.5),1);//将图像尺寸变为512,512     cv::resize(img, dst,cv::Size(512,512));
        Mat img_roi;//Range(int(bridview_width/2)-int(carlogo_w/2),int(bridview_width/2)+int(carlogo_w/2))
        img_roi=imgBirdView(Range((bridview_height-carlogo_h),bridview_height),Range(int(bridview_width/2)-int(carlogo_w/2),int(bridview_width/2)+int(carlogo_w/2)+1));
        addWeighted(img_roi,0.5,carlogo,0.3,0.,img_roi);
        cv::imshow("bird_view_car_logo",imgBirdView );
    }

}
