#ifndef TENSIONCONTROL_H
#define TENSIONCONTROL_H

#include "getsensordata.h"
#include <QtSerialPort/QSerialPort>
#include <QThread>
#include <QDebug>

typedef struct PID
{
    float KP;           // proportion parameter
    float KI;           // integral parameter
    float KD;           // differential parameter
    float Error;        //
    float LastError;    //
    float integral;     //
    float velocity;     //
}TensionPID;


#define TIME_INTERVAL 1000

class TensionControl:public QThread
{
    Q_OBJECT
public:
    explicit TensionControl(QObject *parent=0);
    ~TensionControl();
    void TensionSet(TensionPID *pid_tension, int index, float aimTension);

private:
    void run();
    QSerialPort serial1; // declare a serial com
    QTimer *send_timer;
    QTimer *cycleJointTimer;
    QTimer *linearControlTimer;
    void TensionValueUpdate();
    void CalculateCabelLen(uint i, float* cableLen, float* tempAngle);
    void SendAimCircle(float* aimCircle, int setVel, int setAcc);
    QMutex m_mutex;
    bool m_quit = false;

private slots:
    void slotReadMyCom();
    void slotSendCommand();
    void slotSerialInit();
    void slotSerialClose();
    void slotSerialCtrl(uint tensionOrAngle, int* Data);
    void slotCirculJoint();
    void slotLinearControl();
};

#endif // TENSIONCONTROL_H
