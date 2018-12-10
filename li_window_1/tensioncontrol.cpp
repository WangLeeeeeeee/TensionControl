#include "tensioncontrol.h"

// matlab: theta1:tempAngle[2] theta2:tempAngle[1] theta3:tempAngle[0] theta4:tempAngle[3]
double TensionSensor[6];
TensionPID tension_pid[6] = {{0.6,0,0,0,0,0,0,1},{0.6,0,0,0,0,0,0,1},{0.6,0,0,0,0,0,0,1},{0.6,0,0,0,0,0,0,1},{0.6,0,0,0,0,0,0,1},{0.6,0,0,0,0,0,0,1}};
TorAnPID torAn_pid[4] = {{0.6,0,0,0,0,0},{0.6,0,0,0,0,0},{0.6,0,0,0,0,0},{1,0,0,0,0,0}};
double Testq1[101] = {0,	0.002,	0.011,	0.018,	0.024,	0.029,	0.033,	0.038,	0.042,	0.047,	0.052,	0.057,
                      0.062,	0.067,	0.072,	0.077,	0.082,	0.088,	0.094,	0.1,	0.105,	0.111,	0.116,
                      0.121,	0.126,	0.131,	0.136,	0.14,	0.145,	0.149,	0.153,	0.157,	0.161,	0.165,
                      0.169,	0.172,	0.175,	0.178,	0.181,	0.184,	0.187,	0.189,	0.191,	0.193,	0.195,
                      0.197,	0.198,	0.2,	0.201,	0.202,	0.202,	0.203,	0.203,	0.203,	0.203,	0.203,
                      0.202,	0.201,	0.2,	0.199,	0.197,	0.195,	0.193,	0.191,	0.189,	0.186,	0.183,
                      0.18,	0.176,	0.172,	0.168,	0.164,	0.159,	0.154,	0.149,	0.144,	0.138,	0.132,	0.125,
                      0.119,	0.112,	0.104,	0.096,	0.088,	0.08,	0.072,	0.065,	0.057,	0.05,	0.043,	0.036,
                      0.03,	0.024,	0.019,	0.014,	0.01,	0.007,	0.004,	0.002,	0.001,	0};
double Testq2[101] = {0,	-0.004,	-0.014,	-0.026,	-0.039,	-0.052,	-0.065,	-0.078,	-0.091,	-0.103,	-0.115,	-0.127,
                      -0.138,	-0.15,	-0.16,	-0.17,	-0.18,	-0.189,	-0.198,	-0.206,	-0.212,	-0.219,	-0.224,
                      -0.229,	-0.233,	-0.236,	-0.239,	-0.242,	-0.244,	-0.245,	-0.246,	-0.247,	-0.248,	-0.248,
                      -0.247,	-0.247,	-0.246,	-0.244,	-0.243,	-0.241,	-0.239,	-0.236,	-0.234,	-0.231,	-0.228,
                      -0.225,	-0.221,	-0.217,	-0.214,	-0.21,	-0.205,	-0.201,	-0.197,	-0.192,	-0.187,	-0.182,
                      -0.177,	-0.172,	-0.167,	-0.161,	-0.156,	-0.15,	-0.145,	-0.139,	-0.133,	-0.128,	-0.122,
                      -0.116,	-0.11,	-0.104,	-0.098,	-0.093,	-0.087,	-0.081,	-0.075,	-0.069,	-0.064,	-0.058,
                      -0.052,	-0.047,	-0.041,	-0.036,	-0.031,	-0.026,	-0.022,	-0.018,	-0.014,	-0.011,	-0.009,
                      -0.007,	-0.005,	-0.003,	-0.002,	-0.001,	-0.001,	0,	0,	0,	0,	0,	0};
double Testq3[101] = {0,	0,	0.001,	0.001,	0.003,	0.004,	0.007,	0.009,	0.012,	0.015,	0.019,	0.023,
                      0.028,	0.033,	0.039,	0.045,	0.051,	0.058,	0.066,	0.073,	0.081,	0.089,	0.097,
                      0.105,	0.113,	0.121,	0.129,	0.138,	0.146,	0.154,	0.163,	0.171,	0.18,	0.188,
                      0.197,	0.206,	0.214,	0.223,	0.232,	0.241,	0.25,	0.259,	0.268,	0.277,	0.286,
                      0.295,	0.304,	0.313,	0.323,	0.332,	0.341,	0.351,	0.36,	0.37,	0.379,	0.389,
                      0.399,	0.409,	0.418,	0.428,	0.438,	0.448,	0.458,	0.468,	0.479,	0.489,	0.499,
                      0.51,	0.52,	0.53,	0.541,	0.552,	0.562,	0.573,	0.584,	0.595,	0.606,	0.617,	0.628,
                      0.64,	0.651,	0.662,	0.674,	0.685,	0.696,	0.706,	0.716,	0.725,	0.733,	0.741,	0.749,
                      0.755,	0.761,	0.767,	0.772,	0.776,	0.779,	0.782,	0.784,	0.785,	0.785};
double Testq4[101] = {0,	0.061,	0.083,	0.11,	0.139,	0.169,	0.199,	0.23,	0.261,	0.292,	0.323,	0.353,
                      0.384,	0.414,	0.444,	0.474,	0.504,	0.533,	0.562,	0.59,	0.616,	0.641,	0.664,
                      0.686,	0.707,	0.727,	0.746,	0.763,	0.78,	0.797,	0.812,	0.827,	0.841,	0.854,
                      0.867,	0.879,	0.891,	0.902,	0.912,	0.922,	0.931,	0.94,	0.949,	0.956,	0.964,
                      0.97,	0.977,	0.983,	0.988,	0.993,	0.997,	1.001,	1.005,	1.008,	1.011,	1.013,	1.015,
                      1.016,	1.017,	1.018,	1.018,	1.017,	1.016,	1.015,	1.013,	1.011,	1.009,	1.006,	1.002,
                      0.998,	0.994,	0.989,	0.984,	0.978,	0.972,	0.965,	0.958,	0.95,	0.942,	0.933,	0.924,
                      0.915,	0.904,	0.894,	0.883,	0.873,	0.863,	0.854,	0.845,	0.836,	0.828,	0.82,	0.813,
                      0.807,	0.801,	0.797,	0.793,	0.789,	0.787,	0.786,	0.785};
unsigned int linearCount = 0; // 读取角度位置数
unsigned int lineCycleCount = 0; //直线来回走
unsigned int InverseJoint = 0; //角度三次规划，到达目标角度，往回走

unsigned int ControlMode = 0; // control mode: joint, tension, linear or others

double MAXSPEED=4000;
double MINSPEED=-4000;

double AimTension[6] = {300,300,300,300,300,300};

int testCount = 0;

const double pi=3.1415926;
double cableLenInit[6];
int cycleCount = 0;
int setCir;
double aimCircle[6];
int setAcc,setVel;
double tempAngle[4];

// record if the latest 10 times tension data is below one number
bool tensionLowFlag[6];

//--------------------------------------------------
// Record the teach data of motor count and IMU data
//--------------------------------------------------
unsigned int length = 131072;
QVector<double> MoRecord1(length),MoRecord2(length),MoRecord3(length),MoRecord4(length),MoRecord5(length),MoRecord6(length);
QVector<double> elbowxRecord(length),elbowyRecord(length),elbowzRecord(length);
QVector<double> shouxRecord(length),shouyRecord(length),shouzRecord(length);
unsigned int teachRecordCout = 0;
unsigned int replayCount = 0;

//---------------------------------------------------
// Using the cubic polynomial to plan the angle
//---------------------------------------------------
double runTime = 5;
double thetaStart[4] = {0,0,0,0};
double thetaEnd[4] = {0,0,0,0};
double thetaTf = 50.0;

TensionControl::TensionControl(QObject *parent):QThread(parent)
{
    serial1 = new QSerialPort(this);
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
    unsigned int mintension;
    mintension = 30;
    for(int i=0; i<3; i++)
    {
        if(tension_y[receive_count_tension-i] > mintension)
            tensionLowFlag[0] = 1;
        if(tension_y2[receive_count_tension-i] > mintension)
            tensionLowFlag[1] = 1;
        if(tension_y3[receive_count_tension-i] > mintension)
            tensionLowFlag[2] = 1;
        if(tension_y4[receive_count_tension-i] > mintension)
            tensionLowFlag[3] = 1;
        if(tension_y5[receive_count_tension-i] > mintension)
            tensionLowFlag[4] = 1;
        if(tension_y6[receive_count_tension-i] > mintension)
            tensionLowFlag[5] = 1;
    }
    double count = 5.0;
    for(int i=0; i<count; i++)
    {
        TensionSensor[0] += tension_y[receive_count_tension-i];
        TensionSensor[1] += tension_y2[receive_count_tension-i];
        TensionSensor[2] += tension_y3[receive_count_tension-i];
        TensionSensor[3] += tension_y4[receive_count_tension-i];
        TensionSensor[4] += tension_y5[receive_count_tension-i];
        TensionSensor[5] += tension_y6[receive_count_tension-i];
    }
    TensionSensor[0] /= count;
    TensionSensor[1] /= count;
    TensionSensor[2] /= count;
    TensionSensor[3] /= count;
    TensionSensor[4] /= count;
    TensionSensor[5] /= count;
}


void TensionControl::TensionSet()
{
    TensionValueUpdate();

    QString SendData;

    for(int i=5; i<6; i++) // remember just for emg test!!!!
    {
        if(TensionSensor[i]>8000)
        {
            // it exceed the max tension motor must be stop
            for(int j=0; j<6; j++)
            {
                SendData = QString::number(long(j)) + "V0" + "\r";
                serial1->write(SendData.toLatin1());
            }
            qDebug()<<"the tension is out of the max value, too dangerous";
            tensionCtrlTimer->stop();
        }
        else
        {
            // only the flag equal 1 means it should be controled by the tension
            if(tension_pid[i].flag == 1)
            {
                tension_pid[i].Error = AimTension[i] - TensionSensor[i];
                if(tension_pid[i].Error > 10)
                {
                    tension_pid[i].integral += tension_pid[i].Error;
                    tension_pid[i].velocity = 0.8*tension_pid[i].KP*tension_pid[i].Error + tension_pid[i].KI*tension_pid[i].integral + tension_pid[i].KD*(tension_pid[i].Error-tension_pid[i].LastError);
                    tension_pid[i].LastError = tension_pid[i].Error;

                    // ten array oftension is below one set value means it is totally loose, so speed up to make cable tense.
                    if(tensionLowFlag[i] == 0)
                    {
                        if((i==4) || (i==5))
                            tension_pid[i].velocity = 1400;
                        else
                            tension_pid[i].velocity = 500;
                    }
                }
                else if(tension_pid[i].Error < -10)
                {
                    tension_pid[i].integral += tension_pid[i].Error;
                    tension_pid[i].velocity = tension_pid[i].KP*tension_pid[i].Error + tension_pid[i].KI*tension_pid[i].integral + tension_pid[i].KD*(tension_pid[i].Error-tension_pid[i].LastError);
                    tension_pid[i].LastError = tension_pid[i].Error;
                }
                else
                {
                    tension_pid[i].velocity = 0;
                }

                if(tension_pid[i].velocity > MAXSPEED)
                    tension_pid[i].velocity = MAXSPEED;
                if(tension_pid[i].velocity < MINSPEED)
                    tension_pid[i].velocity = MINSPEED;

                SendData = QString::number(long(i)) + "V" + QString::number(long(tension_pid[i].velocity)) + "\r";
                serial1->write(SendData.toLatin1());
            }
        }
        //qDebug()<<SendData;
    }

    for(int i=0; i<6; i++)
        tensionLowFlag[i] = 0;
}

// Com open configure which connect with the 'open com' button
void TensionControl::slotSerialInit()
{
    serial1->setPortName("COM16");
    serial1->setBaudRate(QSerialPort::Baud9600);
    serial1->setDataBits(QSerialPort::Data8);
    serial1->setStopBits(QSerialPort::OneStop);
    serial1->setFlowControl(QSerialPort::NoFlowControl);
    serial1->setParity(QSerialPort::NoParity);
    serial1->close();

    if(serial1->open(QIODevice::ReadWrite))
    {
        QObject::connect(serial1,SIGNAL(readyRead()),this,SLOT(slotReadMyCom()));
        QMessageBox::information(0,tr("open sucessful"),tr("sucessful open com"),QMessageBox::Ok);
    }
    else
    {
        QMessageBox::critical(0,tr("open failed"),tr("cannot open com"),QMessageBox::Ok);
        return;
    }

    tensionCtrlTimer = new QTimer(this);
    QObject::connect(tensionCtrlTimer, SIGNAL(timeout()), this, SLOT(TensionSet()));

    cycleJointTimer = new QTimer(this);
    QObject::connect(cycleJointTimer, SIGNAL(timeout()), this, SLOT(slotCirculJoint()));

    linearControlTimer = new QTimer(this);
    QObject::connect(linearControlTimer, SIGNAL(timeout()), this, SLOT(slotLinearControl()));

    teachTimer = new QTimer(this);
    QObject::connect(teachTimer, SIGNAL(timeout()), this, SLOT(TeachRecord()));

    replayTimer = new QTimer(this);
    QObject::connect(replayTimer, SIGNAL(timeout()), this, SLOT(replayTeach()));

    torqueTimer = new QTimer(this);
    QObject::connect(torqueTimer, SIGNAL(timeout()), this, SLOT(torqueControl()));
}

// Com close
void TensionControl::slotSerialClose()
{
    tensionCtrlTimer->stop();
    cycleJointTimer->stop();
    linearControlTimer->stop();
    teachTimer->stop();
    replayTimer->stop();
    serial1->close();
}

void TensionControl::slotReadMyCom()
{
    // analyse receive data
}

void TensionControl::slotSerialCtrl(uint tensionOrAngle, int* Data)
{
    unsigned int tempPos[3];
    double cableLen[6],cableLenDeta[6];
    QString SendData;

    // tension control mode
    if(tensionOrAngle == 0)
    {
        ControlMode = 0;
        for(int i=0; i<6; i++)
            AimTension[i] = Data[i];
        // here we change the kp parameter to test
        tension_pid[5].KP = 0.01 * Data[6];
        tension_pid[5].KI = 0.01 * Data[7];

        for(int i=0; i<6; i++)
        {
            SendData = QString::number(long(i)) + "AC" + QString::number(long(111*3)) + "\r";
            serial1->write(SendData.toLatin1());

            SendData = QString::number(long(i)) + "SP" + QString::number(long(MAXSPEED)) + "\r";
            serial1->write(SendData.toLatin1());qazzzzzzzzzzzzzzzzzzzzzaz
        }
        tensionCtrlTimer->start(100);
        cycleJointTimer->stop();
        linearControlTimer->stop();
        replayTimer->stop();
        //teachTimer->stop();
        //emit sigStopplot();
    }
    // Joint angle control mode
    else if(tensionOrAngle == 1)
    {
        ControlMode = 1;
        lineCycleCount = 0;
        cycleJointTimer->stop();
        tensionCtrlTimer->stop();
        linearControlTimer->start(100);
        teachTimer->stop();
        replayTimer->stop();
        emit sigStopplot();
        for(int i=0; i<4; i++)
        {
            thetaEnd[i] = Data[i]*3.14/180;
            //qDebug()<<tempAngle[i];
        }
        for(int i=0; i<6; i++)
        {
            SendData = QString::number(long(i)) + "SP" + QString::number(long(3000)) + "\r";
            serial1->write(SendData.toLatin1());
            SendData = QString::number(long(i)) + "AC" + QString::number(long(222)) + "\r";
            serial1->write(SendData.toLatin1());
            SendData = QString::number(long(i)) + "DEC" + QString::number(long(222)) + "\r";
            serial1->write(SendData.toLatin1());
        }
        /*
        cycleJointTimer->start(5000);
        tensionCtrlTimer->stop();
        linearControlTimer->stop();
        teachTimer->stop();
        replayTimer->stop();
        for(int i=0; i<4; i++)
        {
            tempAngle[i] = Data[i]*3.14/180;
            //qDebug()<<tempAngle[i];
        }
        setVel = Data[4];
        setAcc = Data[5];
        setCir = Data[6];
        for(int i=0; i<6; i++)
        {
            CalculateCabelLen(i,cableLen,tempAngle);
            cableLenDeta[i] = cableLen[i] - cableLenInit[i];
            //cableLenInit[i] = cableLen[i];
            aimCircle[i] = -1*cableLenDeta[i]/(2*pi*20);
            qDebug()<<"aimCircle"<<i<<"is: "<<aimCircle[i];

            /*
            // test a stragety that define the positive and negative cabel
            // for positive cable we control the position that is aimcircle
            // for negative cable we control the tension make it comply the motion
            for(int i=0; i<6; i++)
            {
                if(aimCircle[i]>0.001)
                    tension_pid[i].flag = -1;
            }
        }
        */
    }
    // PTP control mode
    else if(tensionOrAngle == 2)
    {
        ControlMode = 2;
        for(int i=0; i<3; i++)
        {
            tempPos[i] = Data[i];
        }

        // 由于需要指定末端的位置矩阵，包括三维坐标及绕坐标轴的旋转，但由于只有4个自由度，相互之间有关联，指定起来困难
        // 故指定相应关节角（相当于逆解已经求完）
        // 只做演示用途
        tempAngle[0] = 0;
        tempAngle[1] = 30*3.14/180;
        tempAngle[2] = 0;
        tempAngle[3] = 40*3.14/180;
        setVel = Data[3];
        setAcc = Data[4];
        for(int i=0; i<6; i++)
        {
            CalculateCabelLen(i,cableLen,tempAngle);
            cableLenDeta[i] = cableLen[i] - cableLenInit[i];
            cableLenInit[i] = cableLen[i];
            aimCircle[i] = -1*cableLenDeta[i]/(2*pi*20);
            qDebug()<<"aimCircle"<<i<<"is: "<<aimCircle[i];
        }
        SendAimCircle(aimCircle,setVel,setAcc);
    }
    // LINEAR CONTROL MODE
    else if(tensionOrAngle == 3)
    {
        ControlMode = 3;
        lineCycleCount = 0;        

        for(int i=0; i<6; i++)
        {
            SendData = QString::number(long(i)) + "SP" + QString::number(long(3000)) + "\r";
            serial1->write(SendData.toLatin1());
            SendData = QString::number(long(i)) + "AC" + QString::number(long(222)) + "\r";
            serial1->write(SendData.toLatin1());
            SendData = QString::number(long(i)) + "DEC" + QString::number(long(222)) + "\r";
            serial1->write(SendData.toLatin1());
        }
        linearControlTimer->start(50);
        tensionCtrlTimer->stop();
        cycleJointTimer->stop();
        teachTimer->stop();
        replayTimer->stop();
        emit sigStopplot();
    }
    // TORQUE CONTROL MODE
    else
    {
        ControlMode = 4;
        torqueTimer->stop();
        //emit sigStopplot();
        for(int i=0; i<4; i++)
        {
            tempAngle[i] = Data[i];
            //qDebug()<<tempAngle[i];
        }
        torqueTimer->start(50);
    }
}

void TensionControl::CalculateCabelLen(uint i, double *cableLen, double* tempAngle)
{
    switch (i) {
    case 0:
        cableLen[i] = qPow((qPow((35*cos(tempAngle[0])*sin(tempAngle[2]) +
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
                        35*sin(tempAngle[0])*sin(tempAngle[1] - pi/2) + 33),2)),0.5);break;
    case 1:
        cableLen[i] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 +
                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 142),2) +
                       qPow(((343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 -
                        125*cos(tempAngle[1] - pi/2) + 33),2)),0.5);break;
    case 2:
        cableLen[i] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 -
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 -
                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 142),2) +
                       qPow((125*cos(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 + 33),2)),0.5);break;
    case 3:
        cableLen[i] = qPow((qPow((35*cos(tempAngle[0])*sin(tempAngle[2]) -
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
                        35*sin(tempAngle[0])*sin(tempAngle[1] - pi/2) + 33),2)),0.5);break;
    case 4:
        cableLen[i] = qPow((qPow(((607*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 -
                        118*cos(tempAngle[3])*cos(tempAngle[1] - pi/2) +
                        118*cos(tempAngle[1] - pi/2)*sin(tempAngle[3]) +
                        118*cos(tempAngle[0])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) +
                        118*cos(tempAngle[0])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) + 33),2) +
                       qPow(((607*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        118*cos(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) +
                        118*sin(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) -
                        118*cos(tempAngle[2])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) +
                        118*cos(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) -
                        (607*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 - 142),2) +
                       qPow(((607*cos(tempAngle[2])*sin(tempAngle[0]))/2 +
                        118*cos(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) +
                        118*sin(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) +
                        (607*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 +
                        118*cos(tempAngle[3])*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                        118*sin(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2)),2)),0.5);break;
//        cableLen[i] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) -
//                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
//                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 + 142),2) +
//                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 +
//                        125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
//                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 27),2) +
//                       qPow(((343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 - 125*cos(tempAngle[1] - pi/2) + 33),2)),0.5) +
//                qPow((qPow((125*cos(tempAngle[1] - pi/2) - 118*cos(tempAngle[3])*cos(tempAngle[1] - pi/2) +
//                  132*cos(tempAngle[0])*sin(tempAngle[1] - pi/2) + 118*cos(tempAngle[1] - pi/2)*sin(tempAngle[3]) +
//                  118*cos(tempAngle[0])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) +
//                  118*cos(tempAngle[0])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2)),2) +
//                 qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) +
//                  132*sin(tempAngle[2])*sin(tempAngle[0]) +
//                  118*cos(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) +
//                  118*sin(tempAngle[3])*(sin(tempAngle[2])*sin(tempAngle[0]) - cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)) -
//                  118*cos(tempAngle[2])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) +
//                  118*cos(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) -
//                  132*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)),2) +
//                 qPow((132*cos(tempAngle[2])*sin(tempAngle[0]) - 125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
//                  118*cos(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) +
//                  118*sin(tempAngle[3])*(cos(tempAngle[2])*sin(tempAngle[0]) + cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])) +
//                  132*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]) +
//                  118*cos(tempAngle[3])*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) -
//                  118*sin(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2)),2)),0.5);break;
    case 5:
        cableLen[i] = qPow((qPow((118*cos(tempAngle[3])*cos(tempAngle[1] - pi/2) +
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
                        118*sin(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2) + 45),2)),0.5);break;
    default:
        break;
    }
}

void TensionControl::SendAimCircle(double *aimCircle, int setVel, int setAcc)
{
    QString SendData;
    //int runTime = 5;
    //int setAccel = 1;
    //int flag = 0;
    // Recommend vel is 16 acc is 1
    int vel, acc;
    double tA, tS, tAll, tMax;
    tMax = 0;
    aimCircle[5] = aimCircle[5];
    for(int i=0; i<6; i++)
    {
        // set the decellerate quicker than the accelerate, means drag slow, loose quick
//            if(aimCircle[i]<0)
//                vel = 2.1*setVel;
//            else
            vel = setVel;
            acc = setAcc;
            tA = double(vel)/(60*acc);
            if(acc == 0)
            {
                QMessageBox::critical(0,tr("Wrong"),tr("are you stupid enough to set the acc zero"),QMessageBox::Ok);
                return;
            }
            if((acc*tA*tA) >= qAbs(aimCircle[i]))
            {
                tA = qSqrt(qAbs(aimCircle[i])/acc);
                tS = 0;
                vel = tA * acc;
            }
            else
            {
                tA = double(vel)/(60*acc);
                tS = (qAbs(aimCircle[i])-acc*tA*tA)*60/vel;
            }
            tAll = 2*tA + tS;
            if(tAll > tMax) tMax = tAll;

            SendData = QString::number(long(i)) + "SP" + QString::number(long(111*vel)) + "\r";
            serial1->write(SendData.toLatin1());
             qDebug()<<SendData<<endl;
            SendData = QString::number(long(i)) + "AC" + QString::number(long(111*acc)) + "\r";
            serial1->write(SendData.toLatin1());
             qDebug()<<SendData<<endl;
            SendData = QString::number(long(i)) + "DEC" + QString::number(long(111*acc)) + "\r";
            serial1->write(SendData.toLatin1());
             qDebug()<<SendData<<endl;
            SendData = QString::number(long(i)) + "LR" + QString::number(long(aimCircle[i]*111*512*4)) + "\r";
            serial1->write(SendData.toLatin1());
             qDebug()<<SendData<<endl;
            SendData = "M\r";
            serial1->write(SendData.toLatin1());
             qDebug()<<SendData<<endl;
    }

}

void TensionControl::slotCirculJoint()
{
    if(cycleCount < 2*setCir)
    {
        SendAimCircle(aimCircle,setVel,setAcc);
        for(int i=0; i<6; i++)
        {
            aimCircle[i] = -aimCircle[i];
        }
        cycleCount++;
    }
    else
    {
        cycleJointTimer->stop();
        tensionCtrlTimer->stop();
        for(int i=0; i<6; i++)
        {
            aimCircle[i] = 0;
            tension_pid[i].flag = 1;
        }
        cycleCount = 0;
    }
}

void TensionControl::slotLinearControl()
{
    QString SendData;
    double tempAngle[4];
    double cableLen[6],cableLenDeta[6];
    double vel[6];
    double inteTime,inteJointTime;
    inteTime = 0.05;
    inteJointTime = 0.1;
    if(ControlMode == 3)
    {
        linearCount++;
        for(int i=0; i<6; i++)
        {
            if(linearCount > 150)
            {
                linearCount = 0;
                lineCycleCount++;
                if(lineCycleCount == 6)
                {
                    lineCycleCount = 0;
                    linearControlTimer->stop();
                    emit sigStartplot();
                }
            }
            else if(linearCount > 100)
            {
                vel[i] = 0;
            }
            else
            {
                if(lineCycleCount%2 == 0)
                {
                    tempAngle[0] = Testq3[linearCount];
                    tempAngle[1] = Testq2[linearCount];
                    tempAngle[2] = Testq1[linearCount];
                    tempAngle[3] = Testq4[linearCount];
                }
                else
                {
                    tempAngle[0] = Testq3[100-linearCount];
                    tempAngle[1] = Testq2[100-linearCount];
                    tempAngle[2] = Testq1[100-linearCount];
                    tempAngle[3] = Testq4[100-linearCount];
                }
                CalculateCabelLen(i,cableLen,tempAngle);
                cableLenDeta[i] = cableLen[i] - cableLenInit[i];
                //qDebug()<<"cablelendeta"<<i<<"is"<<cableLenDeta[i];
                cableLenInit[i] = cableLen[i];
                aimCircle[i] = -1*cableLenDeta[i]/(2*pi*20);
                qDebug()<<"aimCircle"<<i<<"is"<<aimCircle[i];
                vel[i] = double(111*60*aimCircle[i])/double(inteJointTime); // the last speed must be zero!!!
            }
        }
        for(int i=0; i<6; i++)
        {
            //qDebug()<<"the speed is:"<<vel[i]<<"n/min";
            SendData = QString::number(long(i)) + "V" + QString::number(long(vel[i])) + "\r";
            serial1->write(SendData.toLatin1());
            qDebug()<<SendData<<endl;
        }
    }
    // joint control mode
    if(ControlMode == 1)
    {
        if(linearCount == 0)
        {
            InverseJoint = 0;
            lineCycleCount++;
        }
        if(linearCount == thetaTf)
        {
            InverseJoint = 1;
        }
        if(InverseJoint == 0)
        {
            linearCount++;
        }
        else
        {
            linearCount--;
        }
        if(lineCycleCount == 4)
        {
            lineCycleCount = 0;
            for(int i=0; i<6; i++)
            {
                //qDebug()<<"the speed is:"<<vel[i]<<"n/min";
                SendData = QString::number(long(i)) + "V0" + "\r";
                serial1->write(SendData.toLatin1());
                qDebug()<<SendData<<endl;
            }
            linearControlTimer->stop();
            emit sigStartplot();
        }
        else
        {
            double theta[4]; // cubic poly nomial theta
            double a0[4],a1[4],a2[4],a3[4];
            unsigned int t;
            for(int i=0; i<6; i++)
            {
                for(int a=0; a<4; a++)
                {
                    t = linearCount;
                    a0[a] = thetaStart[a];
                    a1[a] = 0;
                    a2[a] = 3*(thetaEnd[a] - thetaStart[a])/qPow(thetaTf,2);
                    a3[a] = -2*(thetaEnd[a] - thetaStart[a])/qPow(thetaTf,3);
                    theta[a] = a0[a] + a1[a]*t + a2[a]*qPow(t,2) + a3[a]*qPow(t,3);
                }
                CalculateCabelLen(i,cableLen,theta);
                cableLenDeta[i] = cableLen[i] - cableLenInit[i];
                cableLenInit[i] = cableLen[i];
                aimCircle[i] = -1*cableLenDeta[i]/(2*pi*20);
                qDebug()<<"aimCircle"<<i<<"is"<<aimCircle[i];
                vel[i] = 0.5*double(111*60*aimCircle[i])/double(inteTime); // the last speed must be zero!!!
                qDebug()<<"vel"<<i<<"is"<<vel[i];
            }
            for(int i=0; i<6; i++)
            {
                //qDebug()<<"the speed is:"<<vel[i]<<"n/min";
                SendData = QString::number(long(i)) + "V" + QString::number(long(vel[i])) + "\r";
                serial1->write(SendData.toLatin1());
                qDebug()<<SendData<<endl;
            }
        }
    }
    //qDebug()<<"the lineCycleCount is"<<lineCycleCount;
    //qDebug()<<"the linearCout is"<<linearCount;
}

void TensionControl::slotBeforeTigh(unsigned int *Data)
{
    for(int i=0; i<6; i++)
        AimTension[i] = Data[i];
}

void TensionControl::slotTeachStart()
{
    //tensionCtrlTimer->stop();
    cycleJointTimer->stop();
    linearControlTimer->stop();
    teachTimer->start(50);
    //emit sigStopplot();
}

void TensionControl::slotTeachStop()
{
    teachTimer->stop();
    tensionCtrlTimer->stop();
    emit sigStartplot();
}

void TensionControl::TeachRecord()
{
    MoRecord1[teachRecordCout] = Motor1Count[receive_count_mocount];
    MoRecord2[teachRecordCout] = Motor2Count[receive_count_mocount];
    MoRecord3[teachRecordCout] = Motor3Count[receive_count_mocount];
    MoRecord4[teachRecordCout] = Motor4Count[receive_count_mocount];
    MoRecord5[teachRecordCout] = Motor5Count[receive_count_mocount];
    MoRecord6[teachRecordCout] = Motor6Count[receive_count_mocount];
    elbowxRecord[teachRecordCout] = elbow_x[receive_count_angle];
    elbowyRecord[teachRecordCout] = elbow_y[receive_count_angle];
    elbowzRecord[teachRecordCout] = elbow_z[receive_count_angle];
    shoulder_x[teachRecordCout] = shoulder_x[receive_count_angle];
    shoulder_y[teachRecordCout] = shoulder_y[receive_count_angle];
    shoulder_z[teachRecordCout] = shoulder_z[receive_count_angle];
    teachRecordCout++;
    qDebug()<<"there are"<<teachRecordCout<<"are recorded.";
}

void TensionControl::slotReplayTeach()
{
    replayTimer->start(100);
    emit sigStopplot();
}

void TensionControl::replayTeach()
{
    QString SendData;
    double vel[6];
    if(replayCount<teachRecordCout-1)
    {
        vel[0] = (MoRecord1[replayCount+1]-MoRecord1[replayCount])*10*60/512;
        vel[1] = (MoRecord2[replayCount+1]-MoRecord2[replayCount])*10*60/512;
        vel[2] = (MoRecord3[replayCount+1]-MoRecord3[replayCount])*10*60/512;
        vel[3] = (MoRecord4[replayCount+1]-MoRecord4[replayCount])*10*60/512;
        vel[4] = (MoRecord5[replayCount+1]-MoRecord5[replayCount])*10*60/512;
        vel[5] = (MoRecord6[replayCount+1]-MoRecord6[replayCount])*10*60/512;
        qDebug()<<"the speed is:"<<vel[5]<<"n/min";
        for(int i=0; i<6; i++)
        {
            SendData = QString::number(long(i)) + "V" + QString::number(long(vel[i])) + "\r";
            //serial1->write(SendData.toLatin1());
            qDebug()<<SendData<<endl;
        }
        replayCount++;
    }
    else
    {
        emit sigStartplot();
    }
    //emit sigStartplot();
}

void TensionControl::torqueControl()
{
    // just the elbow control for test
    for(int i=0; i<5; i++)
    {
        AimTension[i] = 100;
    }
    torAn_pid[3].Error = tempAngle[3] + elbow_y[receive_count_angle]; // because abduction is decrease
    if((torAn_pid[3].Error > 3) || (torAn_pid[3].Error < -3))
    {
        torAn_pid[3].integral += torAn_pid[3].Error;
        //AimTension[5] = AimTension[5] + torAn_pid[3].KP*torAn_pid[3].Error + torAn_pid[3].KI*torAn_pid[3].integral + torAn_pid[3].KD*(torAn_pid[3].Error-torAn_pid[3].LastError);
        AimTension[5] = torAn_pid[3].KP*torAn_pid[3].Error + torAn_pid[3].KI*torAn_pid[3].integral + torAn_pid[3].KD*(torAn_pid[3].Error-torAn_pid[3].LastError);
        torAn_pid[3].LastError = torAn_pid[3].Error;
    }
    else
    {
        AimTension[5] = AimTension[5];
    }

    qDebug()<<"now the elbow angle is:"<<elbow_y[receive_count_angle];
    qDebug()<<"aimtension5 is:"<<AimTension[5]<<endl;
    TensionSet();
}

void TensionControl::slotEmgTenctrl(unsigned int *data)
{
    if(serial1->isOpen())
    {
        for(int i=0; i<6; i++)
            AimTension[i] = data[i];
        TensionSet();
    }
    else
    {
        slotSerialInit();
        if(serial1->open(QIODevice::ReadWrite))
            serial1->write("ok");
        else
        {
            //error
            qDebug()<<serial1->errorString();
        }
    }
}



















