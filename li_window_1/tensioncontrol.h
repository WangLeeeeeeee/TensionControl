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


#define TIME_INTERVAL 20

class TensionControl:public QThread
{
    Q_OBJECT
public:
    explicit TensionControl(QObject *parent=0);
    void TensionSet(TensionPID *pid_tension, int index, float aimTension);

private:
    void run();
    QSerialPort serial1; // declare a serial com
    QTimer *send_timer;
    void TensionValueUpdate();

private slots:
    void slotReadMyCom();
    void slotSendCommand();
    void slotSerialInit();
    void slotSerialClose();
};

#endif // TENSIONCONTROL_H
