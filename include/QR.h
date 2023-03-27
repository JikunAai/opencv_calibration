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

Mat cut(Mat image){
    cv::Rect m_select;
    cv::Mat gray;
    cv::cvtColor(image, gray, COLOR_BGR2GRAY);
    m_select = Rect(400,300,500,380);
    Mat ROI = image(m_select);

/*    imshow("CUT", ROI);
    waitKey(5);*/
    return ROI;
}

Mat enhance(Mat image){
    dnn_superres::DnnSuperResImpl sr;
    string model_path = "/home/sky/open-cv_-open-gl/OpenCV_GL/TF-ESPCN/export/ESPCN_x2.pb";
    sr.readModel(model_path);
    sr.setModel("espcn", 2); // set the model by passing the value and the upsampling ratio
    Mat result; // creating blank mat for result
    sr.upsample(image, result); // upscale the input image
    return result;
}

vector<cv::Point2f> calObjPos(Mat inputImage){
    float x,y;
    vector<Point2f> pos;
    Point2f pos_row;

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
        pos_row = Point2f(x,y);
        pos.push_back(pos_row);
    }
    return pos;
}

#endif //OPENCV_CALIBRATION_QR_H
