#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
//#include <QGLWidget>

//class GLUquadric;
//class MyGLWidget : public QGLWidget
class MyGLWidget : public QOpenGLWidget,protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit MyGLWidget(QWidget *parent = 0);
    void initTimer();
     ~MyGLWidget();

protected:
    //对3个虚函数重新定义
    void initializeGL();
    void resizeGL(int w,int h);
    void paintGL();

    void keyPressEvent(QKeyEvent *event);//处理键盘事件

private:
    bool full_screen;//是否全屏显示

    GLfloat el_xRot;//肘关节绕x轴旋转的角度
    GLfloat el_yRot;//肘关节绕y轴旋转的角度
    GLfloat el_zRot;//肘关节绕z轴旋转的角度

    GLfloat sh_xRot;//肩关节绕x轴旋转的角度
    GLfloat sh_yRot;//肩关节绕y轴旋转的角度
    GLfloat sh_zRot;//肩关节绕z轴旋转的角度

    int sh_len;//肩关节尺寸
    int sh_wid;
    int sh_hig;
    int el_len;//肘关节尺寸
    int el_wid;
    int el_hig;
    int plot_deep;

signals:
    
public slots:
    
};

#endif // MYGLWIDGET_H
