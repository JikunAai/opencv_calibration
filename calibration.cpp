//
// Created by sky on 23-3-20.
//
#include <opencv2/imgproc/types_c.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <fstream>

using namespace cv;
using namespace std;

#define SRC_WIDTH  1280
#define SRC_HEIGHT 960
Mat image, img_gray;
int BOARDSIZE[2]{ 5,7 };//棋盘格每行每列角点个数

void capPicture(int picNumber)
{
    VideoCapture capture;
    capture.open(0);
    if (!capture.isOpened())
    {
        printf("video fault");
    }

    capture.set(CAP_PROP_FRAME_WIDTH, SRC_WIDTH);        //设置宽度
    capture.set(CAP_PROP_FRAME_HEIGHT, SRC_HEIGHT);  //设置长度
    Mat frame;
    int picNum = 0;
    char* cstr = new char[120];
    while (picNum<picNumber)
    {
        capture >> frame;
        if (frame.data == NULL)
        {
            printf("Image is empty\n");
            //writer.write(frame);
            break;
            //continue;
        }
        char kk=waitKey(2);
        if (kk == 'S' || kk == 's')
        {

            sprintf(cstr, "%s%d%s", "/home/sky/opencv_calibration/calibration/", picNum++, ".jpg");
            imwrite(cstr, frame);
            printf("保存了图片\n");

        }

        namedWindow("picture", 0);//参数为零，则可以自由拖动
        imshow("picture", frame);
        waitKey(2);
    }
}

pair<Mat, Mat> calParameter(){
    pair<Mat,Mat> result;
    vector<vector<Point3f>> objpoints_img;//保存棋盘格上角点的三维坐标
    vector<Point3f> obj_world_pts;//三维世界坐标
    vector<vector<Point2f>> images_points;//保存所有角点
    vector<Point2f> img_corner_points;//保存每张图检测到的角点
    vector<String> images_path;//创建容器存放读取图像路径

    string image_path = "/home/sky/opencv_calibration/calibration/*.jpg";//待处理图路径
    glob(image_path, images_path);//读取指定文件夹下图像

    //转世界坐标系
    for (int i = 0; i < BOARDSIZE[1]; i++)
    {
        for (int j = 0; j < BOARDSIZE[0]; j++)
        {
            obj_world_pts.push_back(Point3f(j, i, 0));
        }
    }

    for (int i = 0; i < images_path.size(); i++)
    {
        image = imread(images_path[i]);
        cvtColor(image, img_gray, COLOR_BGR2GRAY);
        //检测角点
        bool found_success = findChessboardCorners(img_gray, Size(BOARDSIZE[0], BOARDSIZE[1]),
                                                   img_corner_points,
                                                   CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
        cout<<found_success<<endl;
        //显示角点
        if (found_success)
        {
            //迭代终止条件
            TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            //进一步提取亚像素角点
            cornerSubPix(img_gray, img_corner_points, Size(11, 11),
                         Size(-1, -1), criteria);

            //绘制角点
            drawChessboardCorners(image, Size(BOARDSIZE[0], BOARDSIZE[1]), img_corner_points,
                                  found_success);

            objpoints_img.push_back(obj_world_pts);//从世界坐标系到相机坐标系
            images_points.push_back(img_corner_points);
        }
        //char *output = "image";
        char text[] = "image";
        char *output = text;
        imshow(output, image);
        waitKey(0);

    }

    /*
    计算内参和畸变系数等
    */

    Mat cameraMatrix, distCoeffs, R, T;//内参矩阵，畸变系数，旋转量，偏移量
    calibrateCamera(objpoints_img, images_points, img_gray.size(),
                    cameraMatrix, distCoeffs, R, T);

    cout << "cameraMatrix:" << endl;
    cout << cameraMatrix << endl;

    cout << "*****************************" << endl;
    cout << "distCoeffs:" << endl;
    cout << distCoeffs << endl;
    cout << "*****************************" << endl;

    cout << "Rotation vector:" << endl;
    cout << R << endl;

    cout << "*****************************" << endl;
    cout << "Translation vector:" << endl;
    cout << T << endl;

    result = make_pair(cameraMatrix, distCoeffs);
    ///*
    //畸变图像校准
    //*/
    Mat src, dst;
    src = imread("/home/sky/opencv_calibration/calibration/2.jpg");  //读取校正前图像
    undistort(src, dst, cameraMatrix, distCoeffs);

    return result;
}

void correction(Mat cam, Mat dist)
{
    VideoCapture inputVideo(0);
    if(!inputVideo.isOpened()){
        std::cout << "video is not opened\n\n"<<endl;
    }
    else{
        std::cout << "video is opened \n\n"<<endl;
    }
//  标定的相机参数
    Mat frame, frameCalibration;
    inputVideo >> frame;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = cam.at<double>(0,0);
    cameraMatrix.at<double>(0,1) = cam.at<double>(0,1);
    cameraMatrix.at<double>(0,2) = cam.at<double>(0,2);
    cameraMatrix.at<double>(1,0) = cam.at<double>(1,0);
    cameraMatrix.at<double>(1,1) = cam.at<double>(1,1);
    cameraMatrix.at<double>(1,2) = cam.at<double>(1,2);
    cameraMatrix.at<double>(2,0) = cam.at<double>(2,0);
    cameraMatrix.at<double>(2,1) = cam.at<double>(2,1);
    cameraMatrix.at<double>(2,2) = cam.at<double>(2,2);

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0,0) = dist.at<double>(0,0);
    distCoeffs.at<double>(1,0) = dist.at<double>(1,0);
    distCoeffs.at<double>(2,0) = dist.at<double>(2,0);
    distCoeffs.at<double>(3,0) = dist.at<double>(3,0);
    distCoeffs.at<double>(4,0) = dist.at<double>(4,0);

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

}
int main()
{
/*    capPicture(25);*/
    pair<Mat, Mat> pair_paf=calParameter();

    Mat cameraMatrix = pair_paf.first;
    Mat distCoeffs = pair_paf.second;
    correction(cameraMatrix, distCoeffs);
    return 0;
}