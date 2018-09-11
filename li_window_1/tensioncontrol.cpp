#include "tensioncontrol.h"

float TensionSensor[6];
TensionPID tension_pid[6] = {{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{6,0,0,0,0,0,0}};

float MAXSPEED=6000;
float MINSPEED=-6000;

float AimTension[6] = {0,0,0,0,0,0};

int testCount = 0;

TensionControl::TensionControl(QObject *parent):QThread(parent)
{

}

void TensionControl::run()
{

}

void TensionControl::TensionValueUpdate()
{
    TensionSensor[0] = tension_y[receive_count_tension-1];
    TensionSensor[1] = tension_y2[receive_count_tension-1];
    TensionSensor[2] = tension_y3[receive_count_tension-1];
    TensionSensor[3] = tension_y4[receive_count_tension-1];
    TensionSensor[4] = tension_y5[receive_count_tension-1];
    TensionSensor[5] = tension_y6[receive_count_tension-1];
}


void TensionControl::TensionSet(TensionPID *pid_tension, int index, float aimTension)
{
    TensionValueUpdate();

    QString SendData;

    pid_tension->Error = aimTension - TensionSensor[index];


    if((pid_tension->Error > 100) || (pid_tension->Error < -100))
    {
        pid_tension->integral += pid_tension->Error;
        pid_tension->velocity = pid_tension->KP*pid_tension->Error + pid_tension->KI*pid_tension->integral + pid_tension->KD*(pid_tension->Error-pid_tension->LastError);
        pid_tension->LastError = pid_tension->Error;
    }
    else
        pid_tension->velocity = 0;

    if(pid_tension->velocity > MAXSPEED)
        pid_tension->velocity = MAXSPEED;
    if(pid_tension->velocity < MINSPEED)
        pid_tension->velocity = MINSPEED;

    for(int i=0; i<3; i++)
    {
        SendData = "\r";
        serial1.write(SendData.toLatin1());
    }


    SendData = QString::number(long(index)) + "V" + QString::number(long(pid_tension->velocity)) + "\r";
    qDebug()<<SendData<<endl;
    serial1.write(SendData.toLatin1());

}

// Com open configure which connect with the 'open com' button
void TensionControl::slotSerialInit()
{
    serial1.setPortName("COM3");
    serial1.setBaudRate(QSerialPort::Baud9600);
    serial1.setDataBits(QSerialPort::Data8);
    serial1.setStopBits(QSerialPort::OneStop);
    serial1.setFlowControl(QSerialPort::NoFlowControl);
    serial1.setParity(QSerialPort::NoParity);
    serial1.close();

    if(serial1.open(QIODevice::ReadWrite))
    {
        QObject::connect(&serial1,SIGNAL(readyRead()),this,SLOT(slotReadMyCom()));
        QMessageBox::information(0,tr("open sucessful"),tr("sucessful open com"),QMessageBox::Ok);
    }
    else
    {
        QMessageBox::critical(0,tr("open failed"),tr("cannot open com"),QMessageBox::Ok);
        return;
    }

    send_timer = new QTimer(this);
    QObject::connect(send_timer, SIGNAL(timeout()), this, SLOT(slotSendCommand()));
    int send_interval = TIME_INTERVAL;
    send_timer->start(send_interval);

}

// Com close
void TensionControl::slotSerialClose()
{
    serial1.close();
}

void TensionControl::slotReadMyCom()
{
    // analyse receive data
}

void TensionControl::slotSendCommand()
{
    testCount ++;
    if(testCount < 50)
    {
        AimTension[5] = 200;
    }
    else if(testCount < 100)
    {
        AimTension[5] = 1150;
    }
    else
    {
        testCount = 0;
    }
    TensionSet(tension_pid+5, 5, AimTension[5]);
    qDebug()<<"now the testCount is:"<<testCount;
    qDebug()<<"now the AimTension is:"<<AimTension[5];

}




















