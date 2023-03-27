//
// Created by sky on 23-3-2.
//

#ifndef OPENCV_ALGOQRCODE_H
#define OPENCV_ALGOQRCODE_H

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/wechat_qrcode.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/wechat_qrcode.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/dnn_superres.hpp>


using namespace cv;
using namespace std;

class AlgoQRCode
{
private:
    Ptr<wechat_qrcode::WeChatQRCode> detector;

public:
    bool initModel(string modelPath);

    string detectQRCode(Mat cap);

    vector<Mat> detectQRPosition(Mat cap);

    bool compression(string inputFileName, string outputFileName, int quality);

    void release();
};

bool AlgoQRCode::initModel(string modelPath) {
    string detect_prototxt = modelPath + "detect.prototxt";
    string detect_caffe_model = modelPath + "detect.caffemodel";
    string sr_prototxt = modelPath + "sr.prototxt";
    string sr_caffe_model = modelPath + "sr.caffemodel";
    try
    {
        detector = makePtr<wechat_qrcode::WeChatQRCode>(detect_prototxt, detect_caffe_model, sr_prototxt, sr_caffe_model);
    }
    catch (const std::exception& e)
    {
        cout << e.what() << endl;
        return false;
    }

    return true;
}

string AlgoQRCode::detectQRCode(Mat cap)
{
    if (detector == NULL) {
        return "-1";
    }

    vector<Mat> vPoints;
    vector<cv::String> vStrDecoded;
    Mat imgInput = cap;

	detector->detectAndDecode(imgInput, vPoints);
}


bool AlgoQRCode::compression(string inputFileName, string outputFileName, int quality) {
    Mat srcImage = imread(inputFileName);

    if (srcImage.data != NULL)
    {
        vector<int>compression_params;
        compression_params.push_back(IMWRITE_JPEG_QUALITY);
        compression_params.push_back(quality);     //图像压缩参数，该参数取值范围为0-100，数值越高，图像质量越高

        bool bRet = imwrite(outputFileName, srcImage, compression_params);

        return bRet;
    }

    return false;
}

void AlgoQRCode::release() {
    detector = NULL;
}

#endif //OPENCV_ALGOQRCODE_H
