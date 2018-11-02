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

typedef struct TorqueAnglePID
{
    float KP;
    float KI;
    float KD;
    float Error;
    float LastError;
    float integral;
}TorAnPID;


#define TIME_INTERVAL 1000

class TensionControl:public QThread
{
    Q_OBJECT
public:
    explicit TensionControl(QObject *parent=0);
    ~TensionControl();

private:
    void run();
    QSerialPort serial1; // declare a serial com
    QTimer *tensionCtrlTimer;
    QTimer *cycleJointTimer;
    QTimer *linearControlTimer;
    QTimer *teachTimer;
    QTimer *replayTimer;
    QTimer *torqueTimer;
    void TensionValueUpdate();
    void CalculateCabelLen(uint i, float* cableLen, float* tempAngle);
    void SendAimCircle(float* aimCircle, int setVel, int setAcc);
    QMutex m_mutex;
    bool m_quit = false;

private slots:
    void slotReadMyCom();
    void slotSerialInit();
    void slotSerialClose();
    void slotSerialCtrl(uint tensionOrAngle, int* Data);
    void slotCirculJoint();
    void slotLinearControl();
    void slotBeforeTigh(unsigned int *Data);
    void TensionSet();
    void slotTeachStart();
    void slotTeachStop();
    void TeachRecord();
    void slotReplayTeach();
    void replayTeach();
    void torqueControl();
signals:
    void sigStopplot();
    void sigStartplot();

};

#endif // TENSIONCONTROL_H
