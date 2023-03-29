//
// Created by sky on 23-3-27.
//

#ifndef OPENCV_CALIBRATION_QR_H
#define OPENCV_CALIBRATION_QR_H

#include "AlgoQrCode.h"
#include "BiCubic.h"
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

Mat cut(Mat image, int roi_x, int roi_y, int roi_width, int roi_length){
    cv::Rect m_select;
    cv::Mat gray;
    cv::cvtColor(image, gray, COLOR_BGR2GRAY);
    m_select = Rect(roi_x,roi_y,roi_width,roi_length);
    Mat ROI = image(m_select);

/*    imshow("CUT", ROI);
    waitKey(5);*/
    return ROI;
}

Mat enhance(Mat image){
    dnn_superres::DnnSuperResImpl sr;
    string model_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/TF-ESPCN/export/ESPCN_x4.pb";
    sr.readModel(model_path);
    sr.setModel("espcn", 4); // set the model by passing the value and the upsampling ratio
    Mat result; // creating blank mat for result
    sr.upsample(image, result); // upscale the input image
    return result;
}

Point2f calObjPos(Mat inputImage){
    ///only one target..
    float x,y;
    Point2f pos;

    const std::string detector_prototxt_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/detect.prototxt";
    const std::string detector_caffe_model_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/detect"
                                                  ".caffemodel";
    const std::string super_resolution_prototxt_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/sr"
                                                       ".prototxt";
    const std::string super_resolution_caffe_model_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/sr"
                                                          ".caffemodel";
    wechat_qrcode::WeChatQRCode weChatQR;
    weChatQR = wechat_qrcode::WeChatQRCode(detector_prototxt_path,detector_caffe_model_path,
                                           super_resolution_prototxt_path,super_resolution_caffe_model_path);
    vector<Mat> qrcode_box;
    vector<std::string> recon_res = weChatQR.detectAndDecode(inputImage, qrcode_box);

    for (int i = 0; i < recon_res.size(); i++) {
        int min_x = (int)qrcode_box[i].at<float>(0);
        int min_y = (int)qrcode_box[i].at<float>(1);
        int max_x = (int)qrcode_box[i].at<float>(4);
        int max_y = (int)qrcode_box[i].at<float>(5);
        cv::Rect box = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);
        cv::rectangle(inputImage, box, cv::Scalar(0, 255, 0), 8);
        cv::putText(inputImage,recon_res[i],cv::Point(min_x,min_y-10),
                    cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0,0,255),2,2);
        x = (qrcode_box[i].at<float>(0) + qrcode_box[i].at<float>(2) + qrcode_box[i].at<float>(4) + qrcode_box[i].at<float>(6))/4;
        y = (qrcode_box[i].at<float>(1) + qrcode_box[i].at<float>(3) + qrcode_box[i].at<float>(5) + qrcode_box[i].at<float>(7))/4;
        circle(inputImage, Point(x,y), (int)20, Scalar(255, 255, 0), 2);
        pos = Point2f(x,y);
    }
    return pos;
}

vector<Point2f> calObjMultPos(Mat inputImage){
    ///only one target..
    float x,y;
    vector<Point2f> pos;

    const std::string detector_prototxt_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/detect.prototxt";
    const std::string detector_caffe_model_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/detect"
                                                  ".caffemodel";
    const std::string super_resolution_prototxt_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/sr"
                                                       ".prototxt";
    const std::string super_resolution_caffe_model_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/opencv_3rdparty/sr"
                                                          ".caffemodel";
    wechat_qrcode::WeChatQRCode weChatQR;
    weChatQR = wechat_qrcode::WeChatQRCode(detector_prototxt_path,detector_caffe_model_path,
                                           super_resolution_prototxt_path,super_resolution_caffe_model_path);
    vector<Mat> qrcode_box;
    vector<std::string> recon_res = weChatQR.detectAndDecode(inputImage, qrcode_box);

    for (int i = 0; i < recon_res.size(); i++) {
        int min_x = (int)qrcode_box[i].at<float>(0);
        int min_y = (int)qrcode_box[i].at<float>(1);
        int max_x = (int)qrcode_box[i].at<float>(4);
        int max_y = (int)qrcode_box[i].at<float>(5);
        cv::Rect box = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);
        cv::rectangle(inputImage, box, cv::Scalar(0, 255, 0), 8);
        cv::putText(inputImage,recon_res[i],cv::Point(min_x,min_y-10),
                    cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0,0,255),2,2);
        x = (qrcode_box[i].at<float>(0) + qrcode_box[i].at<float>(2) + qrcode_box[i].at<float>(4) + qrcode_box[i].at<float>(6))/4;
        y = (qrcode_box[i].at<float>(1) + qrcode_box[i].at<float>(3) + qrcode_box[i].at<float>(5) + qrcode_box[i].at<float>(7))/4;
        circle(inputImage, Point(x,y), (int)20, Scalar(255, 255, 0), 2);
        pos.push_back(Point2f(x,y));
    }
    return pos;
}

Point3f getWorldPoints(Point2f inPoints, Mat rotationMatrix, Mat cameraMatrix,Mat tvec)
{
    ///根据公式求Zc，即s
    cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
    cv::Mat tempMat, tempMat2;
    //输入一个2D坐标点，便可以求出相应的s
    imagePoint.at<double>(0,0)=inPoints.x;
    imagePoint.at<double>(1,0)=inPoints.y;
    double zConst = 0;//实际坐标系的距离
    //计算参数s
    double s;
    tempMat = rotationMatrix.inv() * cameraMatrix.inv() * imagePoint;
    tempMat2 = rotationMatrix.inv() * tvec;
    s = zConst + tempMat2.at<double>(2, 0);
    s /= tempMat.at<double>(2, 0);

    Mat wcPoint = rotationMatrix.inv() * (s * cameraMatrix.inv() * imagePoint - tvec);
    Point3f worldPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));
    return worldPoint;
}


#endif //OPENCV_CALIBRATION_QR_H
