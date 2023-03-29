//
// Created by sky on 23-3-29.
//

#ifndef OPENCV_CALIBRATION_OPENGLSSVEP_H
#define OPENCV_CALIBRATION_OPENGLSSVEP_H
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

void detect(VideoCapture capture, Mat cameraMatrix, Mat distCoeffs)
{
    Mat gCameraCorr;
    gFinished = 0;
    while(!gFinished)
    {

        capture>>gCameraImage;
        gCameraCorr = correction(cameraMatrix, distCoeffs, gCameraImage);
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
        inRange(imgBGR, Scalar(120, 180, 0), Scalar(255, 255, 130), imgThresholded); //green
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
        x = center.x;
        y = center.y;

        circle(gCameraCorr, Point(int(center.x),int(center.y)), (int)2, Scalar(255, 0, 0), 2);
        gResultImage = gCameraCorr;
        waitKey(5);
        //cam_mutex.unlock();
    }

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
    //cout<<"x,:  "<<x<<"  count:  "<<miss<<endl;
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

    // 防止被 0 所除
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

//TCP通信服务端
int TCP()
{
    //1.创建socket(用于监听的套接字)
    int lfd = socket(AF_INET, SOCK_STREAM, 0);

    if(lfd==-1)
    {
        perror("socket");
        exit(-1);
    }

    //2.绑定
    struct sockaddr_in saddr;
    saddr.sin_family = PF_INET;
    saddr.sin_addr.s_addr = INADDR_ANY; //0.0.0.0
    saddr.sin_port = htons(9999);
    int ret = bind(lfd, (struct sockaddr *)&saddr, sizeof(saddr));

    if(ret == -1)
    {
        perror("bind");
        exit(-1);
    }

    //3.监听
    listen(lfd, 5);
    if(ret==-1)
    {
        perror("listen");
        exit(-1);
    }

    //4.接受客户端连接
    struct sockaddr_in caddr;
    socklen_t len = sizeof(caddr);
    int cfd = accept(lfd, (struct sockaddr *)&caddr, &len);
    if(cfd==-1)
    {
        perror("accept");
        exit(-1);
    }

    //输出客户端的信息
    char cip[16];
    inet_ntop(AF_INET, &caddr.sin_addr.s_addr, cip, sizeof(cip));
    unsigned short cport = ntohs(caddr.sin_port);
    printf("client ip is %s,port is %d\n", cip, cport);

    //5.
    //获取客户端的数据
    char recvbuf[1024] = {0};

    //给客户端发送数据,position
    char *data = "hello ,i am server";
    write(cfd, data, strlen(data));

    while(1)
    {
        int lens = read(cfd, recvbuf, sizeof(recvbuf));
        if(lens==-1)
        {
            perror("read");
            exit(-1);
        }else if(lens>0)
        {
            printf("recv client data: %s\n", recvbuf);
            prob = stod(recvbuf);
            printf("recv client data: %f\n",prob);
        }else if(lens==0)
        {//表示客户端断开连接
/*            printf("client closed...");*/
            break;
        }
    }

    //关闭文件描述符

    close(lfd);
    close(cfd);

}


#endif //OPENCV_CALIBRATION_OPENGLSSVEP_H
