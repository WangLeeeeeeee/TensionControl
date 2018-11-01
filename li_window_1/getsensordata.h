#ifndef GETSENSORDATA_H
#define GETSENSORDATA_H

#include <QThread>
#include <QDebug>
#include <QTextStream>
#include <QMessageBox>
#include <QInputDialog>
#include <QtGui>
#include <QStandardItemModel>
#include "C:/Advantech/DAQNavi/Inc/bdaqctrl.h"
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"

using namespace Automation::BDaq;
#define     deviceDescription  L"PCI-1716L,BID#0"
#define     deviceDescription1 L"PCI-1784,BID#0"
#define     deviceDescription2 L"PCI-1784,BID#9"
#define minize(a,b)            (((a) < (b)) ? (a) : (b))

// extern parameter using for plot
extern QVector<double> elbow_x,elbow_y,elbow_z,shoulder_x,shoulder_y,shoulder_z;
extern QVector<double> tension_y,tension_y2,tension_y3;
extern QVector<double> tension_y4,tension_y5,tension_y6;
extern QVector<double> Motor1Count,Motor2Count,Motor3Count,Motor4Count,Motor5Count,Motor6Count;
extern QVector<double> surpressure_elbow,surpressure_shou1,surpressure_shou2;
extern QVector<double> time_x_tension,time_x_angle,time_x_surpressure,time_x_mocount;
extern double max_tension[];
extern double max_motor_count[];
extern double min_motor_count[];
extern unsigned int receive_count_tension;
extern unsigned int receive_count_angle;
extern unsigned int receive_count_pressure;
extern unsigned int receive_count_mocount;


class GetSensordata : public QThread
{
    Q_OBJECT
private:
    void run();
    WaveformAiCtrl * wfAiCtrl;
    ErrorCode ret;

    // Gets a LpmsSensorManager instance
    LpmsSensorManagerI* manager1 = LpmsSensorManagerFactory();
    LpmsSensorManagerI* manager2 = LpmsSensorManagerFactory();
    // Connects to LPMS-CU sensor with address A1234567
    LpmsSensorI* lpms1 = manager1->addSensor(DEVICE_LPMS_U, "A5022WAT");
    LpmsSensorI* lpms2 = manager2->addSensor(DEVICE_LPMS_U, "A5022WCL");

    QTimer *getsensorTimer;
    void delay(int mseconds);
    UdCounterCtrl* udCounterCtrl;
    UdCounterCtrl* udCounterCtrl1;
    //WaveformAiCtrl * wfAiCtrl;

public:
    explicit GetSensordata(QObject *parent = 0);
    // a static function cannot call a nonstatic member function
    static void BDAQCALL OnStoppedEvent(void * sender, BfdAiEventArgs * args, void * userParam);
    void CheckError(ErrorCode errorCode);

private slots:
    void slotChangeSenFlag();

};

#endif // GETSENSORDATA_H
