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
#include <sstream>


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



void updateTexture(Mat gResultImage) {
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

void SSVEP_BLOCK(Mat gResultImage)
{
    rsizex = rsize / 1920;
    rsizey = rsize / 1080;
    inter_sizex = inter_size / 1920;
    inter_sizey = inter_size / 1080;
    miss++;
    //-------------------------------------------------------------------------------
    //Draw SSVEP_BLOCK;
    updateTexture(gResultImage);
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
    cout<<"x,:  "<<x<<"  count:  "<<miss<<endl;
    x_0 = x/1920*0.75-0.75;
    cout<<"x,:  "<<x_0<<"  count:  "<<rsizex<<endl;
    y_0 = -y/1080+1;
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

    // 1. 设置白色 , glVertex3f (GLfloat x, GLfloat y, GLfloat z)
    glColor3f(1, 1, 0);
    glRectf(double(double(pos-150)/1280.0), double(double(pos-152)/960.0),
            double(double(pos+150)/1280.0), double(double(pos-148)/960.0));

    glColor3f(1, 1, 0);
    glRectf(double(double(pos-152)/1280), double(double(pos-150)/960),
            double(double(pos-148)/1280), double(double(pos+150)/960));

    glColor3f(1, 1, 0);
    glRectf(double(double(pos+152)/1280), double(double(pos-150)/960),
            double(double(pos+148)/1280), double(double(pos+150)/960));

    glColor3f(1, 1, 0);
    glRectf(double(double(pos-150)/1280), double(double(pos+152)/960),
            double(double(pos+150)/1280), double(double(pos+148)/960));

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




#endif //OPENCV_CALIBRATION_OPENGLSSVEP_H
