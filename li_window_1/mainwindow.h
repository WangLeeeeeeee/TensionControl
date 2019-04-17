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
#include "emg_tcp.h"
//#include "modbus.h"
//#include "motorcontrol.h"

class VRDisplay;
class modbus;
class motorcontrol;


// set the plot interval
#define TIME_PLOT_INTERVAL 200


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

private slots:
    void on_actionOpen_triggered();
    void on_actionClose_triggered();
    void on_actionExit_triggered();
    void on_actionSave_triggered();
    void on_actionClean_triggered();
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
    void slotEmgThetaFit(double* fiteff, double* bufferX, double* bufferY, unsigned int dimension, int sizenum);
    void on_VRDisplay_clicked();
    void on_sendmsgButton_clicked();
    void on_actionStartMeasure_triggered();
    void on_actionStopMeasure_triggered();
    void on_BeforeTightenButton_clicked();
    void on_TeachButton_clicked();
    void on_StopteachButton_clicked();
    void on_ReplayButton_clicked();
    void on_pushButton_Listen_clicked();
    void on_pushButton_Start_clicked();
    void on_pushButton_Trigger_clicked();
    void on_StopMotorButton_clicked();
    void modMessage(QString kind, QString message);

private:
    Ui::MainWindow *ui;
    aboutdialog aboutdlg;
    QTimer *plot_timer;
    unsigned int plot_timerdly;//set the serial port receive/send interval
    GetSensordata *getsensordata;
    VRDisplay *vrdisplay;
    EMG_server* emg_server;
    motorcontrol* motorctrl;

    // modbus thread
    QThread* threadModbus;
    modbus *mb;

    // 3D surface
    Q3DScatter *graph = new Q3DScatter();
    QWidget *container = QWidget::createWindowContainer(graph);
    LinkDisplay *linkdisplay = new LinkDisplay(graph);

signals:
    // for com connect vr device
    void sigVRSerialOpen();

    // for emg
    void sigEmgStart();
    void sigEmgTrigger();

    // for modbus
    void sigModbusClose();

    // for motorcontrol
    void sigDisableMotor();
    void sigMdBeforeTigh(unsigned int *Data);
    void sigMdSerialCtrl(unsigned int TensionOrAngle,int *Data);
    void sigMdTeachStart();
    void sigMdTeachStop();
    void sigMdReplayTeach();

};

#endif // MAINWINDOW_H
