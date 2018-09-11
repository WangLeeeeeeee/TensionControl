#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QMessageBox>
#include <QInputDialog>
#include <QStandardItemModel>
#include <QFile>
#include <QFileDialog>
#include <QInputDialog>
#include <QTextStream>
#include <QSlider>
#include "qcustomplot.h"
#include "aboutdialog.h"
#include "getsensordata.h"
#include "plotcurves.h"
#include "linkdisplay.h"
//#include "vrdisplay.h"
//#include "motorcontrol.h"
class VRDisplay;
class MotorControl;
class TensionControl;
//#include <QtSerialPort/QSerialPort> //put the com send data in motorcontrol
//#include "myglwidget.h"

// set the read/write com delay
//#define TIME_OUT  2

// set the read com interval
//#define TIMER_INTERVAL 10

// set the plot interval
#define TIME_PLOT_INTERVAL 100


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void startInit();


protected:
    void setActionsEnable(bool status);
    void setComboxEnable(bool status);
    
private slots:
    void on_actionOpen_triggered();
    void on_actionClose_triggered();
    void on_actionInfor_triggered();
    void on_actionExit_triggered();
    void on_actionAdd_triggered();
    void on_actionSave_triggered();
    void on_actionClean_triggered();
    void on_sendmsgButton_clicked();
    //void readMyCom();
    void plot();
    void setLine1EditValue();
    void setLine2EditValue();
    void setLine3EditValue();
    void setLine4EditValue();
    void setLine5EditValue();
    void setLine6EditValue();
    void setLine7EditValue();
    void setLine8EditValue();
    void setLine9EditValue();
    void setLine10EditValue();
    void Plot_Init();

    void on_Openglaction_triggered();

    void on_dataGetButton_clicked();

    void on_dataStopGetButton_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_VRDisplay_clicked();

private:
    aboutdialog aboutdlg;
    //QSerialPort serial; // declare a serial com
    QTimer *plot_timer;
    unsigned int plot_timerdly;//set the serial port receive/send interval
    GetSensordata *getsensordata;
    MotorControl *motorcontrol;
    VRDisplay *vrdisplay;
    TensionControl *tensioncontrol;
    Ui::MainWindow *ui;

signals:
    void sigSerialInit();
    void sigSerialClose();
    void sigBeforeTigh();
    void sigQianqu();
    void sigWaizhan();
    void sigSerialCtrl(bool TensionOrAngle, unsigned int *Data);
    void sigHuishou();
    void sigHuishouWaizhan();
    void sigJianqianqu();
    void sigHuishouJianqianqu();
    void sigVRSerialOpen();
};

#endif // MAINWINDOW_H
