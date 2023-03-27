//
// Created by sky on 23-3-20.
//
#include<iostream>
#include <ctime>
#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture inputVideo(0);
    if(!inputVideo.isOpened()){
        std::cout << "video is not opened\n\n"<<endl;
    }
    else{
        std::cout << "video is opened \n\n"<<endl;
    }
//  Matlab 标定的相机参数
    Mat frame, frameCalibration;
    inputVideo >> frame;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = 1982.56844306278;
    cameraMatrix.at<double>(0,1) = 1.79099355543064;
    cameraMatrix.at<double>(0,2) = 1042.90384922068;
    cameraMatrix.at<double>(1,1) = 1983.84445594899;
    cameraMatrix.at<double>(1,2) = 480.442502729538;

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0,0) = -0.515906663211726;
    distCoeffs.at<double>(1,0) =  0.201811855093355;
    distCoeffs.at<double>(2,0) =  0.00228453839673728;
    distCoeffs.at<double>(3,0) = -0.00134697993045861;
    distCoeffs.at<double>(4,0) = -0.0572379026696125;

/*  C++程序标定的相机参数
	Mat frame, frameCalibration;
	inputVideo >> frame;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0,0) = 1978.304376178962;
	cameraMatrix.at<double>(0,1) =				   0;
	cameraMatrix.at<double>(0,2) = 1044.639043480329;
	cameraMatrix.at<double>(1,1) = 1979.71454820083;
	cameraMatrix.at<double>(1,2) = 482.6287237060178;
	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0,0) = -0.5277684150872038;
	distCoeffs.at<double>(1,0) =  0.2663992436241138;
	distCoeffs.at<double>(2,0) = -0.001857829391420174;
	distCoeffs.at<double>(3,0) = -0.002175774665050042;
	distCoeffs.at<double>(4,0) = -0.1007311729522544;
*/

    Mat view, rview, map1, map2;
    Size image_Size;
    image_Size = frame.size();

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, image_Size, CV_16SC2, map1, map2);
    // initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_Size, 0.5, image_Size, 0),image_Size, CV_16SC2, map1, map2);

    while(1){
        inputVideo >> frame;
        if(frame.empty()) break;
        remap(frame, frameCalibration, map1, map2, INTER_LINEAR);
        imshow("Original_image",frame);
        imshow("Calibrated_image", frameCalibration);
        char key =waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q') break;
    }


    return 0;
}