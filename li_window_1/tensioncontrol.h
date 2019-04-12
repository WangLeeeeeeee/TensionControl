#ifndef TENSIONCONTROL_H
#define TENSIONCONTROL_H

#include "getsensordata.h"
#include <QtSerialPort>
#include <QThread>
#include <QDebug>

typedef struct PID
{
    double KPTight;     // proportion of tight
    double KPLoose;     // proportion of loose
    double KI;           // integral parameter
    double KD;           // differential parameter
    double Error;        //
    double LastError;    //
    float integral;     //
    double velocity;     //
    int flag;          // 1 means it should be control by tension, -1 means do not controled by tension
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
    QSerialPort *serial1; // declare a serial com

private:
    void run();
    //QSerialPort *serial1; // declare a serial com
    QTimer *tensionCtrlTimer;
    QTimer *cycleJointTimer;
    QTimer *linearControlTimer;
    QTimer *teachTimer;
    QTimer *replayTimer;
    QTimer *torqueTimer;
    void TensionValueUpdate();
    void CalculateCabelLen(uint i, double* cableLen, double* tempAngle);
    void SendAimCircle(double* aimCircle, int setVel, int setAcc);
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
    void slotEmgTenctrl(unsigned int* data); // for emg
signals:
    void sigStopplot();
    void sigStartplot();

};

#endif // TENSIONCONTROL_H
