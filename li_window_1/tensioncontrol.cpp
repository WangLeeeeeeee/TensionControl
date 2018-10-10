#include "tensioncontrol.h"

float TensionSensor[6];
TensionPID tension_pid[6] = {{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{1,0,0,0,0,0,0},{1.8,0,0,0,0,0,0}};

//float MAXSPEED=600;
//float MINSPEED=-600;

float MAXSPEED=3000;
float MINSPEED=-3000;

float AimTension[6] = {0,0,0,0,0,0};

int testCount = 0;

const float pi=3.1415926;
float cableLenInit[6];

TensionControl::TensionControl(QObject *parent):QThread(parent)
{
    cableLenInit[0] = 139.769;
    cableLenInit[1] = 234.459;
    cableLenInit[2] = 234.459;
    cableLenInit[3] = 139.769;
    cableLenInit[4] = 392.226;
    cableLenInit[5] = 391.833;
}

TensionControl::~TensionControl()
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
    //send_timer->start(send_interval);

}

// Com close
void TensionControl::slotSerialClose()
{
    send_timer->stop();
    serial1.close();
}

void TensionControl::slotReadMyCom()
{
    // analyse receive data
}

void TensionControl::slotSendCommand()
{
    QString SendData;

    TensionSet(tension_pid+5, 5, 1200);
    /*
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
    */

    /*
    if(testCount < 4)
    {
        SendData = "5SP" + QString::number(long(MAXSPEED)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = QString::number(long(5)) + "LA" + QString::number(230000) + "\r";
        qDebug()<<SendData<<endl;
        serial1.write(SendData.toLatin1());
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
        testCount ++;
    }
    else if(testCount < 8)
    {
        SendData = "5SP" + QString::number(long(MAXSPEED)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = QString::number(long(5)) + "LA" + QString::number(0) + "\r";
        qDebug()<<SendData<<endl;
        serial1.write(SendData.toLatin1());
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
        testCount ++;
        if(testCount == 8)
            testCount = 0;
    }
    */

}

void TensionControl::slotSerialCtrl(uint tensionOrAngle, int* Data)
{
    unsigned int tempTension[6];
    unsigned int tempPos[3];
    float tempAngle[4];
    float cableLen[6],cableLenDeta[6],aimCircle[6];
    QString SendData;
    int runTime = 5;
    int setAcc = 1;
    int flag = 0;
    // tension control mode
    if(tensionOrAngle == 0)
    {
        for(int i=0; i<6; i++)
            tempTension[i] = Data[i];
    }
    // Joint angle control mode
    else if(tensionOrAngle == 1)
    {
        for(int i=0; i<4; i++)
        {
            tempAngle[i] = Data[i]*3.14/180;
            qDebug()<<tempAngle[i];
        }

        cableLen[0] = qPow((qPow((35*cos(tempAngle[0])*sin(tempAngle[2]) +
                        125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        35*cos(tempAngle[2])*cos(tempAngle[1] - pi/2)*sin(tempAngle[0]) +
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 + 142),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 -
                        35*cos(tempAngle[2])*cos(tempAngle[0]) +
                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 +
                        35*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])*sin(tempAngle[0]) + 27),2) +
                       qPow(((343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 -
                        125*cos(tempAngle[1] - pi/2) +
                        35*sin(tempAngle[0])*sin(tempAngle[1] - pi/2) + 33),2)),0.5);
        qDebug()<<"cable1 length:"<<cableLen[0];
        cableLen[1] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 +
                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 142),2) +
                       qPow(((343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 -
                        125*cos(tempAngle[1] - pi/2) + 33),2)),0.5);
        qDebug()<<"cable2 length:"<<cableLen[1];
        cableLen[2] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 -
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 -
                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 142),2) +
                       qPow((125*cos(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 + 33),2)),0.5);
        qDebug()<<"cable3 length:"<<cableLen[2];
        cableLen[3] = qPow((qPow((35*cos(tempAngle[0])*sin(tempAngle[2]) -
                        125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        35*cos(tempAngle[2])*cos(tempAngle[1] - pi/2)*sin(tempAngle[0]) +
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 - 142),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 -
                        35*cos(tempAngle[2])*cos(tempAngle[0]) -
                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 +
                        35*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])*sin(tempAngle[0]) + 27),2) +
                       qPow((125*cos(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 +
                        35*sin(tempAngle[0])*sin(tempAngle[1] - pi/2) + 33),2)),0.5);
        qDebug()<<"cable4 length:"<<cableLen[3];
        cableLen[4] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 + 142),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 +
                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 27),2) +
                       qPow(((343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 - 125*cos(tempAngle[1] - pi/2) + 33),2)),0.5) +
                qPow((qPow((125*cos(tempAngle[1] - pi/2) - 118*cos(tempAngle[3])*cos(tempAngle[1] - pi/2) +
                  132*cos(tempAngle[0])*sin(tempAngle[1] - pi/2) + 118*cos(tempAngle[1] - pi/2)*sin(tempAngle[3]) +
                  118*cos(tempAngle[0])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) +
                  118*cos(tempAngle[0])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2)),2) +
                 qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                  132*sin(tempAngle[2])*sin(tempAngle[0]) +
                  118*cos(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) +
                  118*sin(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) -
                  118*cos(tempAngle[2])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) +
                  118*cos(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) -
                  132*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)),2) +
                 qPow((132*cos(tempAngle[2])*sin(tempAngle[0]) - 125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                  118*cos(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) +
                  118*sin(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) +
                  132*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]) +
                  118*cos(tempAngle[3])*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                  118*sin(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2)),2)),0.5);
        qDebug()<<"cable5 length:"<<cableLen[4];
        cableLen[5] = qPow((qPow((118*cos(tempAngle[3])*cos(tempAngle[1] - pi/2) +
                        (607*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 +
                        118*cos(tempAngle[1] - pi/2)*sin(tempAngle[3]) +
                        118*cos(tempAngle[0])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) -
                        118*cos(tempAngle[0])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) + 33),2) +
                       qPow(((607*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        118*cos(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) -
                        118*sin(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) +
                        118*cos(tempAngle[2])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) +
                        118*cos(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) -
                        (607*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 + 142),2) +
                       qPow(((607*cos(tempAngle[2])*sin(tempAngle[0]))/2 +
                        118*cos(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) -
                        118*sin(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) +
                        (607*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 -
                        118*cos(tempAngle[3])*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                        118*sin(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) + 45),2)),0.5);
        qDebug()<<"cable6 length:"<<cableLen[5];
        for(int i=0; i<6; i++)
        {
            cableLenDeta[i] = cableLen[i] - cableLenInit[i];
            cableLenInit[i] = cableLen[i];
            qDebug()<<"cable"<<i<<"deta length:"<<cableLenDeta[i];
            aimCircle[i] = -1*cableLenDeta[i]/(2*pi*20);
            if(setAcc*qPow(runTime,2) < 4*qAbs(aimCircle[i]))
                flag = 1;
        }
        if(flag == 1)
        {
            QMessageBox::critical(0,tr("Wrong"),tr("no numerical result"),QMessageBox::Ok);
            //send_timer->stop();
        }
        else
        {
            for(int i=0; i<6; i++)
            {
                float accTime = (setAcc*runTime-qSqrt(qPow(setAcc*runTime,2)-4*setAcc*qAbs(aimCircle[i])))/(2*setAcc);
                float maxSpeed = accTime*setAcc*60;
                SendData = QString::number(long(i)) + "SP" + QString::number(long(111*maxSpeed)) + "\r";
                serial1.write(SendData.toLatin1());
                SendData = QString::number(long(i)) + "AC" + QString::number(long(111*setAcc)) + "\r";
                serial1.write(SendData.toLatin1());
                SendData = QString::number(long(i)) + "DEC" + QString::number(long(111*setAcc)) + "\r";
                serial1.write(SendData.toLatin1());
                SendData = QString::number(long(i)) + "LR" + QString::number(long(aimCircle[i]*111*512*4)) + "\r";
                serial1.write(SendData.toLatin1());
                SendData = "M\r";
                serial1.write(SendData.toLatin1());
            }
        }
    }
    else
    {
        for(int i=0; i<3; i++)
        {
            tempPos[i] = Data[i];

        }

    }
}




















