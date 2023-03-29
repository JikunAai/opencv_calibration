#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "include/calibration.h"
#include "include/QR.h"
#include "include/OpenGlSSVEP.h"


#define SRC_WIDTH  1280
#define SRC_HEIGHT 960



Mat cameraMatrix;
Mat distCoeffs;



int main(int argc, char* argv[])
{
    int calQRTime = 0;
    Mat Image;
    Mat Image_cut;
    Mat Image_enhance;
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
    cameraMatrix = pair_paf.first;
    distCoeffs = pair_paf.second;

    /// detect QR & calculate outer parameter &calculate inner parameter
    vector<cv::Point3f> objectRealPoints ;
    vector<cv::Point2f> objectPoints;
    vector<cv::Point2f> tobjectPoints;
    Point2f obj;
    Point3f OBJ;
    vector<cv::Point2f> objs;
    vector<cv::Point3f> objsF;
    while(1)
    {
        capture>>Image;
        objectPoints.clear();
        Image = correction(cameraMatrix, distCoeffs, Image);

        Image_cut = cut(Image,375,225,175,175);//Rect(400,300,500,380);
        Image_enhance = enhance(Image_cut);
        obj = calObjPos(Image_enhance);
        obj.x /= 4;
        obj.y /= 4;
        obj.x += 375;
        obj.y += 225;
        cout<<obj<<endl;
        objectPoints.push_back(obj);
        imshow("QRCode1", Image_enhance);

        Image_cut = cut(Image,805,235,175,175);//Rect(400,300,500,380);
        Image_enhance = enhance(Image_cut);
        obj = calObjPos(Image_enhance);
        obj.x /= 4;
        obj.y /= 4;
        obj.x += 805;
        obj.y += 235;
        cout<<obj<<endl;
        objectPoints.push_back(obj);
        imshow("QRCode2", Image_enhance);

        Image_cut = cut(Image,365,525,175,175);//Rect(400,300,500,380);
        Image_enhance = enhance(Image_cut);
        obj = calObjPos(Image_enhance);
        obj.x /= 4;
        obj.y /= 4;
        obj.x += 365;
        obj.y += 525;
        cout<<obj<<endl;
        objectPoints.push_back(obj);
        imshow("QRCode3", Image_enhance);

        Image_cut = cut(Image,815,535,175,175);//Rect(400,300,500,380);
        Image_enhance = enhance(Image_cut);
        obj = calObjPos(Image_enhance);
        obj.x /= 4;
        obj.y /= 4;
        obj.x += 815;
        obj.y += 535;
        cout<<obj<<endl;
        objectPoints.push_back(obj);
        imshow("QRCode4", Image_enhance);

        char kk = waitKey(0);
        if(kk == 's' || kk == 'S')

            destroyAllWindows();
            break;
    }

    cout<<objectPoints<<endl;

    objectRealPoints.push_back(Point3f(-0.25,-0.25,0));
    objectRealPoints.push_back(Point3f(0.25,-0.25,0));
    objectRealPoints.push_back(Point3f(-0.25,-0.6,0));
    objectRealPoints.push_back(Point3f(0.25,-0.6,0));

    pair<Mat, Mat> inner = calinnerParameter(objectPoints,objectRealPoints,cameraMatrix,distCoeffs);
    Mat rota = inner.first;
    Mat tran = inner.second;

    while(1)
    {
        capture>>Image;
        tobjectPoints.clear();
        Image = correction(cameraMatrix, distCoeffs, Image);
        imshow("correction image", Image);

        Image_cut = cut(Image,530,370,300,230);//Rect(400,300,500,380);
        Image_enhance = enhance(Image_cut);
        objs = calObjMultPos(Image_enhance);

        for (int i=0; i<objs.size(); i++)
        {
            obj = objs.at(i);
            obj.x /= 4;
            obj.y /= 4;
            obj.x += 530;
            obj.y += 370;
            OBJ = getWorldPoints(obj, rota, cameraMatrix, tran);
            objsF.push_back(OBJ);
        }

        imshow("QRCode00", Image_enhance);
        cout<<objsF<<endl;
        char kk = waitKey(0);
        if(kk == 's' || kk == 'S')
            destroyAllWindows();
            break;
    }

    cout<<"Finished calibration process, waiting for SSVEP"<<endl;

    struct timeval start_tv;
    gettimeofday(&start_tv,NULL);
    //Set time to count;
    start_t = start_tv.tv_sec*1000 + start_tv.tv_usec/1000;

    //set parameters;
    parameter_set();

    //Set gl;
    glInit(argc,argv);

    thread t1(glut_go);
    thread t2(detect,capture,cameraMatrix,distCoeffs);
    thread t3(TCP);

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
