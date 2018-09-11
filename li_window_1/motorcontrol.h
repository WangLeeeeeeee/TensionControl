#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <QThread>
#include <QDebug>
#include "mainwindow.h"
#include <QMessageBox>
//#include <QtSerialPort/QSerialPort>
//#include "currentpid.h"
#include "getsensordata.h"
#include "tensioncontrol.h"

/*
typedef struct PID
{
    float KP;
    float KI;
    float KD;
    float PreError;
    float LastError;
}PID;
*/


// set the com send interval
//#define TIME_SEND_INTERVAL 100

class MotorControl:public QThread
{
    Q_OBJECT
public:
    explicit MotorControl(QObject *parent=0);

private:
    void run();
   // QSerialPort serial1; // declare a serial com
   // QTimer *send_timer;
    //float PIDCalculate(PID *pid_tension,float ThisError);
    //void TensionControl(int index,float i);
    void AngleControl(int index, float aim_angle);
    void UpdateAllState(void);

private slots:
    void slotUiParRec(bool TensionOrAngle, unsigned int *Data);
    //void readMyCom1();
   // void slotSendCommand();
    //void slotSerialInit();
    void slotBeforeTigh();
   // void slotSerialClose();
    void slotQianqu();
    void slotWaizhan();
    void slotHuishou();
    void slotHuishouWaizhan();
    void slotJianqianqu();
    void slotHuishouJianqianqu();


};

#endif // MOTORCONTROL_H
