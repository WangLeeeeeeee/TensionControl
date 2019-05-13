#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

//#include "getsensordata.h"
#include "encoderread.h"
#include "tensionread.h"
#include "imuread.h"
#include <QTimer>
#include <QDebug>
#include <QtCore/qmath.h>

// 一些基本的电机驱动器的指令
#define MODESELECT 0x3100
#define TORQUESET 0x0703
#define SPEEDSET 0x3109
#define DISABLE 0
#define VELOCITYMODE 3
#define TORQUEMODE 1

// 定义控制模式
#define TENSIONCONTROL 0
#define JOINTANGLECONTROL 1
#define JOINTWITHTENSION 2
#define LINEARCONTROL 3
#define JOINTTORQUECONTROL 4

// 定义JOINTWITHTENSION下每根绳索的控制模式
#define TENSION 1
#define SPEED 0

typedef struct ModPID
{
    double KPTight;     // proportion of tight
    double KPLoose;     // proportion of loose
    double KI;           // integral parameter
    double KD;           // differential parameter
    double PreError;     // e(k-2)
    double LastError;    // e(k-1)
    double Output;     // output
}ModTensionPID;

typedef struct TorqueAnglePID
{
    double KP;           // proportion of loose
    double KI;           // integral parameter
    double KD;           // differential parameter
    double PreError;     // e(k-2)
    double LastError;    // e(k-1)
    double Output;     // output
}TorAnPID;


class motorcontrol: public QObject
{
    Q_OBJECT
public:
    motorcontrol();
private:
    QTimer *tensionCtrlTimer; // tension PID control timer
    QTimer *cycleJointTimer; // joint control timer
    QTimer *linearControlTimer; // robot end move line control timer
    QTimer *teachTimer; //  robot demonstration control timer
    QTimer *replayTimer; // robot playback demonstration control timer
    QTimer *torqueTimer; // robot joint torque control timer
    void InitialCableLen(double* initAngle);
    void InitialMotor();
    void ModTensionValueUpdate();
    void CalculateCabelLen(uint i, double* cableLen, double* tempAngle);
    void SendAimCircle(double* aimCircle, int setVel, int setAcc);
private slots:
    // from mainwindow signals
    void slotDisableMotor();
    void slotEnableMotor();
    void slotMdBeforeTigh(unsigned int *Data);
    void slotMdSerialCtrl(uint tensionOrAngle, int *Data);
    void slotMdTeachStart();
    void slotMdTeachStop();
    void slotMdReplayTeach();
    // from timer(tensionCtrlTimer, cycleJointTimer linearControlTimer, teachTimer, replayTimer, torqueTimer)
    void ModTensionSet();
    void CirculJoint();
    void LinearControl();
    void TeachRecord();
    void ReplayTeach();
    void TorqueControl();
    // from emg_tcp signals
    void slotEmgTenctrl(unsigned int* data); // for emg
signals:
    void sigMotorControl(uint Seraddress, int Startaddress, qint32 Data, bool writeLength);
};

#endif // MOTORCONTROL_H
