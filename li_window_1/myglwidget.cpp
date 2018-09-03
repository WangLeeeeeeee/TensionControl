#include "myglwidget.h"
//#include<GL/glu.h>
#include<QKeyEvent>
#include<QTimer>
#include<math.h>
#include"mainwindow.h"
#include "getsensordata.h"

//MyGLWidget::MyGLWidget(QWidget *parent) :
//    QGLWidget(parent)
MyGLWidget::MyGLWidget(QWidget *parent) :
    QOpenGLWidget(parent)

{
    full_screen=false;

    el_xRot = 0.0f;
    el_yRot = 0.0f;
    el_zRot = 0.0f;
    sh_xRot = 0.0f;
    sh_yRot = 0.0f;
    sh_zRot = 0.0f;

    el_len=1.5f;//肘关节参数
    el_wid=1.5f;
    el_hig=5.0f;
    sh_len=2.0f;//肩关节参数
    sh_wid=2.0f;
    sh_hig=4.0f;
    plot_deep=20.0f;
}

MyGLWidget::~MyGLWidget()
{
}

void MyGLWidget::initTimer()
{
    QTimer *timer_3dpaint=new QTimer(this);//创建一个3d绘图的定时器
    //将定时器与updataGL()绑定
    connect(timer_3dpaint,SIGNAL(timeout()),this,SLOT(updateGL()));
    timer_3dpaint->start(1000);///定时10ms
}

void MyGLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0.0,0.0,0.0,0.0);//黑色背景
    glShadeModel(GL_SMOOTH);//启用阴影平滑

    glClearDepth(1.0);//设置深度缓存
    glEnable(GL_DEPTH_TEST);//启用深度测试
    glDepthFunc(GL_LEQUAL);//所作深度测试类型
    glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);//告诉系统对透视进行修正

}

void MyGLWidget::resizeGL(int w, int h)
{
    glViewport(0,0,(GLint)w,(GLint)h);//重置当前窗口
    glMatrixMode(GL_PROJECTION);//选择投影矩阵
    glLoadIdentity();//重置投影矩阵
    //设置视口大小
    //gluPerspective(45.0,(GLfloat)w/(GLfloat)h,0.1,100.0);//GLU lib cannot be used gluPerspective replaced by another function
    GLdouble zNear = 0.1;
    GLdouble zFar = 100.0;
    GLdouble fov = 45.0;
    GLdouble aspectRatio = (GLfloat)w/(GLfloat)h;
    GLdouble rFov = fov * 3.14159265 / 180.0;
    glFrustum( -zNear * tan( rFov / 2.0 ) * aspectRatio,
               zNear * tan( rFov / 2.0 ) * aspectRatio,
               -zNear * tan( rFov / 2.0 ),
               zNear * tan( rFov / 2.0 ),
               zNear, zFar );
    glMatrixMode(GL_MODELVIEW);//选择模型观察矩阵
    glLoadIdentity();//重置模型观察矩阵
}

void MyGLWidget::paintGL()
{
    float sh_x_theta,sh_y_theta,sh_z_theta,el_x_theta,el_y_theta,el_z_theta;

    el_xRot=elbow_x[receive_count_angle];
    el_yRot=elbow_y[receive_count_angle];
    el_zRot=elbow_z[receive_count_angle];
    sh_xRot=shoulder_x[receive_count_angle];
    sh_yRot=shoulder_y[receive_count_angle];
    sh_zRot=shoulder_z[receive_count_angle];

    sh_x_theta=sh_xRot*3.1415926/180;
    sh_y_theta=sh_yRot*3.1415926/180;
    sh_z_theta=sh_zRot*3.1415926/180;
    el_x_theta=el_xRot*3.1415926/180;
    el_y_theta=el_yRot*3.1415926/180;
    el_z_theta=el_zRot*3.1415926/180;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //清除屏幕和深度缓存
    glLoadIdentity();                                   //重置模型观察矩阵

    glTranslatef(0.0f,0.0f,-plot_deep);//移入屏幕
    glRotatef(sh_xRot,1.0f,0.0f,0.0f);//绕x轴旋转
    glRotatef(sh_yRot,0.0f,1.0f,0.0f);//绕y轴旋转
    glRotatef(sh_zRot,0.0f,0.0f,1.0f);//绕z轴旋转
    glBegin(GL_QUADS);//绘制肩关节
        glColor3f(0.0f,1.0f,0.0f);//绿色
        glVertex3f(sh_len/2.0f,0.0f,-sh_wid/2.0f);//右上（顶面）
        glVertex3f(-sh_len/2.0f,0.0f,-sh_wid/2.0f);//左上（顶面）
        glVertex3f(-sh_len/2.0f,0.0f,sh_wid/2.0f);//左下（顶面）
        glVertex3f(sh_len/2.0f,0.0f,sh_wid/2.0f);//右下（顶面）

        glColor3f(1.0f,0.5f,0.0f);//橙色
        glVertex3f(sh_len/2.0f,-sh_hig,-sh_wid/2.0f);//右上（底面）
        glVertex3f(-sh_len/2.0f,-sh_hig,-sh_wid/2.0f);//左上（底面）
        glVertex3f(-sh_len/2.0f,-sh_hig,sh_wid/2.0f);//左下（底面）
        glVertex3f(sh_len/2.0f,-sh_hig,sh_wid/2.0f);//右下（底面）

        glColor3f(1.0f,0.0f,0.0f);//红色
        glVertex3f(sh_len/2.0f,0.0f,sh_wid/2.0f);//右上（前面）
        glVertex3f(-sh_len/2.0f,0.0f,sh_wid/2.0f);//左上（前面）
        glVertex3f(-sh_len/2.0f,-sh_hig,sh_wid/2.0f);//左下（前面）
        glVertex3f(sh_len/2.0f,-sh_hig,sh_wid/2.0f);//右下（前面）

        glColor3f(1.0f,1.0f,0.0f);//黄色
        glVertex3f(sh_len/2.0f,0.0f,-sh_wid/2.0f);//右上（后面）
        glVertex3f(-sh_len/2.0f,0.0f,-sh_wid/2.0f);//左上（后面）
        glVertex3f(-sh_len/2.0f,-sh_hig,-sh_wid/2.0f);//左下（后面）
        glVertex3f(sh_len/2.0f,-sh_hig,-sh_wid/2.0f);//右下（后面）

        glColor3f(0.0f,0.0f,1.0f);//蓝色
        glVertex3f(sh_len/2.0f,0.0f,-sh_wid/2.0f);//右上（右面）
        glVertex3f(sh_len/2.0f,0.0f,sh_wid/2.0f);//左上（右面）
        glVertex3f(sh_len/2.0f,-sh_hig,sh_wid/2.0f);//左下（右面）
        glVertex3f(sh_len/2.0f,-sh_hig,-sh_wid/2.0f);//右下（右面）

        glColor3f(1.0f,0.0f,1.0f);//紫色
        glVertex3f(-sh_len/2.0f,0.0f,sh_wid/2.0f);//右上（左面）
        glVertex3f(-sh_len/2.0f,0.0f,-sh_wid/2.0f);//左上（左面）
        glVertex3f(-sh_len/2.0f,-sh_hig,-sh_wid/2.0f);//左下（左面）
        glVertex3f(-sh_len/2.0f,-sh_hig,sh_wid/2.0f);//右下（左面）
    glEnd();

    glLoadIdentity();//重置模型观测矩阵
    glTranslatef(sh_hig*cos(sh_y_theta)*sin(sh_z_theta),
                 -sh_hig*(sin(sh_x_theta)*sin(sh_y_theta)*sin(sh_z_theta)+cos(sh_x_theta)*cos(sh_z_theta)),
                 sh_hig*(cos(sh_x_theta)*sin(sh_y_theta)*sin(sh_z_theta)-sin(sh_x_theta)*cos(sh_z_theta))-plot_deep);//中心转移到肩关节底部中心
    glRotatef(el_xRot,1.0f,0.0f,0.0f);//绕x轴旋转
    glRotatef(el_yRot,0.0f,1.0f,0.0f);//绕y轴旋转
    glRotatef(el_zRot,0.0f,0.0f,1.0f);//绕z轴旋转
    glBegin(GL_QUADS);//绘制肘关节
        glColor3f(0.0f,1.0f,0.0f);//绿色
        glVertex3f(el_len/2.0f,0.0f,-el_wid/2.0f);//右上（顶面）
        glVertex3f(-el_len/2.0f,0.0f,-el_wid/2.0f);//左上（顶面）
        glVertex3f(-el_len/2.0f,0.0f,el_wid/2.0f);//左下（顶面）
        glVertex3f(el_len/2.0f,0.0f,el_wid/2.0f);//右下（顶面）

        glColor3f(1.0f,0.5f,0.0f);//橙色
        glVertex3f(el_len/2.0f,-el_hig,-el_wid/2.0f);//右上（底面）
        glVertex3f(-el_len/2.0f,-el_hig,-el_wid/2.0f);//左上（底面）
        glVertex3f(-el_len/2.0f,-el_hig,el_wid/2.0f);//左下（底面）
        glVertex3f(el_len/2.0f,-el_hig,el_wid/2.0f);//右下（底面）

        glColor3f(1.0f,0.0f,0.0f);//红色
        glVertex3f(el_len/2.0f,0.0f,el_wid/2.0f);//右上（前面）
        glVertex3f(-el_len/2.0f,0.0f,el_wid/2.0f);//左上（前面）
        glVertex3f(-el_len/2.0f,-el_hig,el_wid/2.0f);//左下（前面）
        glVertex3f(el_len/2.0f,-el_hig,el_wid/2.0f);//右下（前面）

        glColor3f(1.0f,1.0f,0.0f);//黄色
        glVertex3f(el_len/2.0f,0.0f,-el_wid/2.0f);//右上（后面）
        glVertex3f(-el_len/2.0f,0.0f,-el_wid/2.0f);//左上（后面）
        glVertex3f(-el_len/2.0f,-el_hig,-el_wid/2.0f);//左下（后面）
        glVertex3f(el_len/2.0f,-el_hig,-el_wid/2.0f);//右下（后面）

        glColor3f(0.0f,0.0f,1.0f);//蓝色
        glVertex3f(el_len/2.0f,0.0f,-el_wid/2.0f);//右上（右面）
        glVertex3f(el_len/2.0f,0.0f,el_wid/2.0f);//左上（右面）
        glVertex3f(el_len/2.0f,-el_hig,el_wid/2.0f);//左下（右面）
        glVertex3f(el_len/2.0f,-el_hig,-el_wid/2.0f);//右下（右面）

        glColor3f(1.0f,0.0f,1.0f);//紫色
        glVertex3f(-el_len/2.0f,0.0f,el_wid/2.0f);//右上（左面）
        glVertex3f(-el_len/2.0f,0.0f,-el_wid/2.0f);//左上（左面）
        glVertex3f(-el_len/2.0f,-el_hig,-el_wid/2.0f);//左下（左面）
        glVertex3f(-el_len/2.0f,-el_hig,el_wid/2.0f);//右下（左面）
    glEnd();
}

void MyGLWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
       {
       case Qt::Key_F1:                                    //F1为全屏和普通屏的切换键
           full_screen = !full_screen;
           if (full_screen)
           {
               showFullScreen();
           }
           else
           {
               showNormal();
           }
           break;
       case Qt::Key_Escape:                                //ESC为退出键
           close();
           break;
       }
}
