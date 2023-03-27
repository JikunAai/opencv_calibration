#include "include/calibration.h"
#include "include/QR.h"

#define SRC_WIDTH  1280
#define SRC_HEIGHT 960

int main()
{
    int calQRTime = 0;
    Mat Image;
    VideoCapture capture;
    capture.open(0);
    if (!capture.isOpened())
    {
        printf("video fault");
    }

    capture.set(CAP_PROP_FRAME_WIDTH, SRC_WIDTH);        //设置宽度
    capture.set(CAP_PROP_FRAME_HEIGHT, SRC_HEIGHT);  //设置长度
    /// capture calibration picture
/*    capPicture(25,capture);*/
    /// calculate inner parameter
    pair<Mat, Mat> pair_paf=calParameter();
    Mat cameraMatrix = pair_paf.first;
    Mat distCoeffs = pair_paf.second;

    /// detect QR & calculate outer parameter &calculate inner parameter
    vector<cv::Point3f> objectRealPoints ;
    vector<cv::Point2f> objectPoints;
    while(1)
    {
        capture>>Image;

        imshow("raw image", Image);
        Image = correction(cameraMatrix, distCoeffs, Image);
        imshow("correction image", Image);
        Image = cut(Image);
        Image = enhance(Image);
        objectPoints = calObjPos(Image);
        imshow("QRCode", Image);

        char kk = waitKey(2);
        if(kk == 's' || kk == 'S')
            break;
        else
            waitKey(0);
    }

    objectRealPoints.at(0) = Point3f(20,20,10);
    objectRealPoints.at(1) = Point3f(20,50,10);
    objectRealPoints.at(2) = Point3f(50,50,10);
    objectRealPoints.at(3) = Point3f(50,20,10);

    pair<Mat, Mat> inner = calinnerParameter(objectPoints,objectRealPoints,cameraMatrix,distCoeffs);
    Mat tran = inner.first;
    Mat rota = inner.second;

    return 0;
}
