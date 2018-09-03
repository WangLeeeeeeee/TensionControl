#ifndef VRDISPLAY_H
#define VRDISPLAY_H

#include <QThread>
#include <QDebug>
#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include "getsensordata.h"
#include "mainwindow.h"


// set the com send interval
#define TIME_SEND_INTERVAL 100

class VRDisplay:public QThread
{
    Q_OBJECT
public:
    explicit VRDisplay(QObject *parent=0);
private:
    QSerialPort vrSerial; // declare a serial com
    QTimer *vrTimer;
    void run();
    char ConvertHexChar(char ch);
    QByteArray QString2Hex(QString str);
    void SendData(int sendvalue);
private slots:
    void slotVRSerialOpen();
    void slotSendVRdata();
    void slotReadVRcom();
};

#endif // VRDISPLAY_H
