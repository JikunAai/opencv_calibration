#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "include/calibration.h"
#include "include/QR.h"
/*#include "include/OpenGlSSVEP.h"*/
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <GL/glut.h>    // OpenGL toolkit
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <sys/socket.h>
#include <sys/time.h>
#include <stdio.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sstream>

#define SRC_WIDTH  1280
#define SRC_HEIGHT 960

Mat cameraMatrix;
Mat distCoeffs;
VideoCapture capture(0);
//TCP通信服务端
int targetList[5][2];

using namespace cv;
using namespace std;

GLfloat x_0 = 0.0f, y_0 = 0.0f;
GLfloat x = 0.0f, y = 0.0f;
GLfloat rsize = 250, inter_size = 420;
GLfloat rsizex, rsizey, inter_sizex, inter_sizey;
float R[5], fre[5];
float pi = 3.1415926;
float gLightPos[4] = { 0, 5, 0, 1 };
long start_t;
long timet;

//Set some needed parameters;
GLuint gCameraTextureId;
Mat gCameraImage, gResultImage, imgHSV, imgBGR, imgThresholded;
int timer = 0, k = 0, miss = 0, gFinished;
float tt;
int pos = 0;
double prob;
//static mutex cam_mutex;
vector<cv::Point2f> objsC; // object pos in rgb
int target = 1; // target from ros
int targetT = 1; //targetT
int inArea = 0; // in area?
// 跟踪窗口宽度和高度的变化
GLfloat windowWidth;
GLfloat windowHeight;

void parameter_set()
{
    fre[0] = 8.0f;fre[1] = 9.0f;fre[2] = 10.0f;fre[3] = 11.0f;fre[4] = 12.0f; //frequency;
    R[0] = 1.0f;R[1] = 1.0f;R[2] = 1.0f;R[3] = 1.0f;R[4] = 1.0f;//color;
}

void glInit(int argc, char* argv[]) {
    //-------------------------------------------------------------------------------
    //OpenGL init;

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &gCameraTextureId);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glClearColor(1, 1, 1, 1.0f); //Draw background color

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(1600,1200);//#define WIDTH       640 #define HEIGHT      480
    glutCreateWindow("Bounce");
}

void updateTexture() {
    //-------------------------------------------------------------------------------
    //Update texture from camera;
    glBindTexture(GL_TEXTURE_2D, gCameraTextureId);

    // set texture filter to linear - we do not build mipmaps for speed
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // create the texture from OpenCV image data
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1280, 960, 0, GL_BGR,
                 GL_UNSIGNED_BYTE, gResultImage.data);
}

void SSVEP_BLOCK()
{
    rsizex = rsize / 1920;
    rsizey = rsize / 1080;
    inter_sizex = inter_size / 1920;
    inter_sizey = inter_size / 1080;
    miss++;
    //-------------------------------------------------------------------------------
    //Draw SSVEP_BLOCK;
    updateTexture();
    glClear(GL_COLOR_BUFFER_BIT);
    glutPostRedisplay();

    // render the background image from camera texture
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glColor3f(1, 1, 1);
    //-------------------------------------------------------------------------------
    //Draw background by using texture from camera;

    // set up the model_view matrix so that the view is between [-1,-1] and [1,1]
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // draw the quad textured with the camera image
    glBindTexture(GL_TEXTURE_2D, gCameraTextureId);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 1);
    glVertex2f(-0.75, -1);
    glTexCoord2f(0, 0);
    glVertex2f(-0.75, 1);
    glTexCoord2f(1, 0);
    glVertex2f(0.75, 1);
    glTexCoord2f(1, 1);
    glVertex2f(0.75, -1);
    glEnd();

    // reset the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glDisable(GL_TEXTURE_2D);

    //OpenGL2OpenCV coordinate transform;
/*    cout<<"x,:  "<<x<<"  count:  "<<miss<<endl;*/
    x_0 = x/640*0.75-0.75;
/*    cout<<"x,:  "<<x_0<<"  count:  "<<rsizex<<endl;*/
    y_0 = -y/480+1;
    y_0 = y_0 - inter_sizey;

    //Set block;
    glColor3f(R[3], R[3], R[3]);
    glRectf(x_0 - rsizex/2, y_0 + rsizey/2 - inter_sizey, x_0 + rsizex/2, y_0 - inter_sizey - rsizey/2);
    glColor3f(R[0], R[0], R[0]);
    glRectf(x_0 - rsizex/2 + inter_sizex, y_0 + rsizey/2, x_0 + inter_sizex + rsizex/2, y_0 - rsizey/2);
    glColor3f(R[2], R[2], R[2]);
    glRectf(x_0 - rsizex/2, y_0 + rsizey/2 + inter_sizey, x_0 + rsizex/2, y_0 + inter_sizey - rsizey/2);
    glColor3f(R[1], R[1], R[1]);
    glRectf(x_0 - rsizex/2 - inter_sizex, y_0 + rsizey/2, x_0 - inter_sizex + rsizex/2, y_0 - rsizey/2);
/*    glColor3f(R[4], R[4], R[4]);
    glRectf(x_0 - rsizex/2, y_0 + rsizey/2, x_0 + rsizex/2, y_0 - rsizey/2);*/


/*    glRectf((300+pos)/1280, (300+pos)/960, 300/1280, 300/960);*/
/*    cout<<"x,:  "<<pos<<endl;*/
    //Set light
    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0, GL_POSITION, gLightPos);
    glutSwapBuffers();

}

void TimerFunction(int value)
{

    //-------------------------------------------------------------------------------
    //Set SSVEP_BLOCK color & Set fresh frequency;当空闲时由GLUT函数库调用（窗口未改变大小或移动时）
    //Set time to count;
    float t;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    //Set time to count;

    long time = tv.tv_sec*1000 + tv.tv_usec/1000;
    t = float(time - start_t)/1000.0;
    tt = time - timet;
    timet = time;
    //cout << timer << endl;
    //Set color with R[];
    R[0] = (sin(2.0f*pi*fre[0]*t) + 1.0f)/2.0f ;
    R[1] = (sin(2.0f*pi*fre[1]*t) + 1.0f)/2.0f ;
    R[2] = (sin(2.0f*pi*fre[2]*t) + 1.0f)/2.0f ;
    R[3] = (sin(2.0f*pi*fre[3]*t) + 1.0f)/2.0f ;
    //R[4] = (sin(2.0f*pi*fre[4]*t) + 1.0f)/2.0f ;
    R[4] = timer % 2 ;

    //TIME
    glutPostRedisplay();
    glutTimerFunc(8,TimerFunction, value);
}

void ChangeSize(int w, int h, VideoCapture cap)
{
    //-------------------------------------------------------------------------------
    // 当窗口改变大小时由GLUT函数库调用
    GLfloat aspectRatio;

    cap.set(CAP_PROP_FRAME_WIDTH, windowWidth);
    cap.set(CAP_PROP_FRAME_HEIGHT, windowHeight);

    // 防止被 0 所, cameraMatrix, distCoeffs除
    if(h == 0)
        h = 1;

    // 把视口设置为窗口的大小
    glViewport(0, 0, w, h);

    // 重置坐标系统
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // 建立裁剪区域（左、右、底、顶、近、远）
    aspectRatio = (GLfloat)w / (GLfloat)h;
    if (w <= h)
    {
        windowWidth = 100;
        windowHeight = 100 / aspectRatio;
        glOrtho (-100.0, 100.0, -windowHeight, windowHeight, 1.0, -1.0);
    }
    else
    {
        windowWidth = 100 * aspectRatio;
        windowHeight = 100;
        glOrtho (-windowWidth, windowWidth, -100.0, 100.0, 1.0, -1.0);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void glut_go()
{
    glutDisplayFunc(&SSVEP_BLOCK);
    glutTimerFunc(8, TimerFunction, 1);
    //glutReshapeFunc(ChangeSize);
    glutMainLoop();
}

int TCP_Client()
{
    int client_sockfd;
    int len1;
    int len2;
    struct sockaddr_in remote_addr;

    char recv_buf[BUFSIZ];//数据接收缓冲区
    char send_buf[BUFSIZ];//数据传输缓冲区
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family=AF_INET;//设置为IP通信
    remote_addr.sin_addr.s_addr=inet_addr("10.181.9.233");//服务器IP地址
    remote_addr.sin_port=htons(8800);//服务器端口号

    //创建客户端套接字 IPv4 tcp
    if((client_sockfd=socket(PF_INET, SOCK_STREAM, 0))<0)
    {
        cout<<"socket error";
        return 1;
    }

    //绑定服务器网络地址
    if(connect(client_sockfd, (struct sockaddr*)&remote_addr, sizeof(struct sockaddr))<0)
    {
        cout<<"connect error";
        return 1;
    }

    cout<<"connected to server"<<endl;;
/*    len=recv(client_sockfd, recv_buf, BUFSIZ, 0);//接受服务端消息
    recv_buf[len] = '\0';*/
    //循环的发送接受信息并打印接受信息（可以按需发送）--struct sockaddr))<0

    while(1)
    {
        //发送消息给服务端

        len1=send(client_sockfd, &targetList, sizeof(targetList), 0);

        //从客户端接受消息
        len2=recv(client_sockfd,(char *)&targetT,sizeof(targetT),0);
        inArea = targetT % 10;
        target = int(targetT / 10);
        if(len2 > 0){
            recv_buf[len2]='\0';
            cout<<"Received target："<<targetT<<" ，Info Length："<<len2<<endl;
            cout<<"Received inArea："<<inArea<<" ，Info Length："<<len2<<endl;
        }
    }

    close(client_sockfd);
}

void detect()
{
    int green;
    int red;
    Mat gCameraCorr;
    gFinished = 0;
    while(!gFinished)
    {

        capture>>gCameraImage;
        gCameraCorr = correction(cameraMatrix, distCoeffs, gCameraImage);
/*        gCameraCorr = gCameraImage;*/
        //cam_mutex.lock();
        k++;
        vector<std::vector<Point>> contours;
        vector<Vec4i> hireachy;
        Point2f center;
        float radius = 5;
        vector<Mat> hsvSplit;   //创建向量容器，存放HSV的三通道数据
        cvtColor(gCameraCorr, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        split(imgHSV, hsvSplit);			//分类原图像的HSV三通道
        equalizeHist(hsvSplit[2], hsvSplit[2]);    //对HSV的亮度通道进行直方图均衡
        merge(hsvSplit, imgHSV);				   //合并三种通道
        cvtColor(imgHSV, imgBGR, COLOR_HSV2BGR);    //将HSV空间转回至RGB空间，为接下来的颜色识别做准备
        //inRange(imgBGR, Scalar(0, 128, 128), Scalar(127, 255, 255), imgThresholded); //yellow
        inRange(imgBGR, Scalar(50, 170, 0), Scalar(255, 255, 145), imgThresholded); //green
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
        findContours(imgThresholded, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        //Choose the biggest area as target.
        if (!contours.empty()) {
            double maxArea = 0;
            for (int i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[static_cast<int>(i)]);
                if (area > maxArea) {
                    maxArea = area;
                    minEnclosingCircle(contours[static_cast<int>(i)], center, radius);//get center;
                }
            }
        }
        if (inArea == 1)
        {
            red = 255;
            green = 0;

        }
        else
        {
            red = 0;
            green = 255;
        }

        x = center.x;
        y = center.y;
        circle(gCameraCorr, Point(int(center.x),int(center.y)), (int)2, Scalar(255, 0, 0), 2);
        rectangle(gCameraCorr, Point2i (objsC.at(max(target-1,0)).x - 75,objsC.at(max(target-1,0)).y - 75),
                  Point2i (objsC.at(max(target-1,0)).x + 75,objsC.at(max(target-1,0)).y + 75) ,
                  Scalar(0, green, red),3, 4, 0);
        gResultImage = gCameraCorr;

        waitKey(1);
        //cam_mutex.unlock();
    }

}



int main(int argc, char* argv[])
{
    Mat Image;
    Mat Image_cut;
    Mat Image_enhance;

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
        {
            destroyAllWindows();
            break;
        }
    }

    cout<<objectPoints<<endl;

    objectRealPoints.push_back(Point3f(-0.245,-0.245,0));
    objectRealPoints.push_back(Point3f(0.25,-0.25,0));
    objectRealPoints.push_back(Point3f(-0.25,-0.6,0));
    objectRealPoints.push_back(Point3f(0.25,-0.6,0));

    pair<Mat, Mat> inner = calinnerParameter(objectPoints,objectRealPoints,cameraMatrix,distCoeffs);
    Mat rota = inner.first;
    Mat tran = inner.second;

    int ROI[5][2] = {
            {475,320},
            {475,525},
            {600,425},
            {735,320},
            {735,525}
    };
    for(int area = 0; area<5; area++)
    {
        while(1)
        {
            capture>>Image;
            tobjectPoints.clear();
            Image = correction(cameraMatrix, distCoeffs, Image);
            imshow("correction image", Image);

            Image_cut = cut(Image,ROI[area][0],ROI[area][1],125,125);//Rect(400,300,500,380);
            Image_enhance = enhance(Image_cut);
            objs = calObjMultPos(Image_enhance);
            if(objs.size()!=0)
            {
                obj = objs.at(0);
                obj.x /= 4;
                obj.y /= 4;
                obj.x += ROI[area][0];
                obj.y += ROI[area][1];

                OBJ = getWorldPoints(obj, rota, cameraMatrix, tran);
                OBJ.x = int(OBJ.x*1000);
                OBJ.y = int(OBJ.y*1000);
                OBJ.z = int(OBJ.z*1000);

                imshow("QRCode_OBJ", Image_enhance);
            }

            char kk = waitKey(0);
            if(kk == 's' || kk == 'S')
            {
                objsF.push_back(OBJ);
                objsC.push_back(obj);
                destroyAllWindows();
                break;
            }

        }
    }
    cout<<objsF<<endl;
    cout<<objsC<<endl;
    targetList[0][0] = {int(objsF.at(0).x)};targetList[0][1] = {int(objsF.at(0).y-0.02)};
    targetList[1][0] = {int(objsF.at(1).x)};targetList[1][1] = {int(objsF.at(1).y)};
    targetList[2][0] = {int(objsF.at(2).x)};targetList[2][1] = {int(objsF.at(2).y)};
    targetList[3][0] = {int(objsF.at(3).x)};targetList[3][1] = {int(objsF.at(3).y-0.02)};
    targetList[4][0] = {int(objsF.at(4).x)};targetList[4][1] = {int(objsF.at(4).y)};

    cout<<"Finished calibration process, waiting for SSVEP"<<endl;
    destroyAllWindows();

    struct timeval start_tv;
    gettimeofday(&start_tv,NULL);
    start_t = start_tv.tv_sec*1000 + start_tv.tv_usec/1000;
    //set parameters;

    parameter_set();

    //Set gl;
    glInit(argc,argv);

    thread t1(detect);
    thread t2(glut_go);
    thread t3(TCP_Client);

    t1.join();
    t2.join();
    t3.join();
    return 0;
}
