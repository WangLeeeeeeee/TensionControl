#ifndef MODBUS_H
#define MODBUS_H

#include <QModbusDataUnit>
#include <QSerialPort>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include "getsensordata.h"
#include <QDebug>

class QModbusClient;
class QModbusReply;

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
    float KP;
    float KI;
    float KD;
    float Error;
    float LastError;
    float integral;
}TorAnPID;


class modbus:public QThread
{
    Q_OBJECT

public:
    explicit modbus(QObject *parent=0);
    ~modbus();

private:
    // about modbus read and write
    void readModbus(uint Seraddress, int Startaddress, uint number);
    void readReady();
    void writeModbus(uint Seraddress, int Startaddress, qint32 Data, bool writeLength);
    QModbusDataUnit readRequest() const;
    QModbusDataUnit writeRequest() const;

    void InitialCableLen();
    void ModTensionValueUpdate();
    void CalculateCabelLen(uint i, double* cableLen, double* tempAngle);
    void SendAimCircle(double* aimCircle, int setVel, int setAcc);

private:
    QModbusReply *lastRequest;
    QModbusClient *modbusDevice;
    QTimer *tensionCtrlTimer; // tension PID control timer
    QTimer *cycleJointTimer; // joint control timer
    QTimer *linearControlTimer; // robot end move line control timer
    QTimer *teachTimer; //  robot demonstration control timer
    QTimer *replayTimer; // robot playback demonstration control timer
    QTimer *torqueTimer; // robot joint torque control timer

private slots:
    //void slotModbusRead(); // read encoder
    //void slotInitialization(); // initialize motor parameter

    // from mainwindow signals
    void slotDisableMotor();
    void slotMdSerialInit();
    void slotMdSerialClose();
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
    // it is about encoder.
    void encReturn(QString);
    // the signal send to mainwindow to stop or start plot.
    void sigMdStopplot();
    void sigMdStartplot();
};

#endif // MODBUS_H
