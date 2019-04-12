#include "modbus.h"
#include <QModbusRtuSerialMaster>

// 走直线轨迹时的关节角度中间值
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

//---------------------------------------------------
//
//---------------------------------------------------
double cableLenInit[6]; // 绳长初始值
const double pi=3.1415926;
float rollRadius = 60; // 卷轮半径（mm）
double MODMAXSPEED=4000; // 最大速度限制
double MODMINSPEED=-4000; // 最小速度限制
unsigned int ControlMode = 0; // control mode

//---------------------------------------------------
// 关节角控制相关参数
//---------------------------------------------------
double thetaStart[4] = {0,0,0,0}; // 关节角规划的起始角度
double thetaEnd[4] = {0,0,0,0}; // 关节角规划的终止角度（由界面设定决定）
unsigned int jointIntervalTime = 100; // 关节角运动定时器周期ms（关节角规划的detaT）
bool InverseJoint = 0; // 来回运行
unsigned int jointCount = 0; // 关节角运动当前时间=jointCount*detaT
unsigned int jointCycleCount = 0; // 关节角当前运行来回计数
unsigned int setJointCirCount = 0; // 界面设定的关节角来回数
double thetaTf = 20.0; // 关节角规划的整个时间（t = thetaTf*JointIntervalTime）

//---------------------------------------------------
// 末端走直线轨迹相关参数
//---------------------------------------------------
unsigned int lineIntervalTime = 100; // 关节角运动定时器周期ms（关节角规划的detaT）
bool InverseLine = 0; // 角度三次规划，是否到达目标角度
unsigned int linearCount = 0; // 走直线轨迹时读取关节角度计数
unsigned int lineCycleCount = 0; // 直线来回走的次数
unsigned int setLineCirCount = 0; // 关节角当前运行来回计数

//---------------------------------------------------
// 示教运动相关参数
//---------------------------------------------------
unsigned int length = 131072;
QVector<double> MoRecord1(length),MoRecord2(length),MoRecord3(length),MoRecord4(length),MoRecord5(length),MoRecord6(length);
QVector<double> elbowxRecord(length),elbowyRecord(length),elbowzRecord(length);
QVector<double> shouxRecord(length),shouyRecord(length),shouzRecord(length);
unsigned int teachRecordCout = 0;
unsigned int replayCount = 0;
unsigned int teachIntervalTime = 50;
unsigned int replayIntervalTime = 50;

//---------------------------------------------------
// 张力控制相关参数
//---------------------------------------------------
double AimTension[6] = {300,300,300,300,300,300}; // 目标张力值
double ModTensionSensor[6]; // 当前张力值
ModTensionPID Modtension_pid[6] = {{0.7,0.4,0,0,0,0,0},{0.7,0.4,0,0,0,0,0},{0.7,0.4,0,0,0,0,0},{0.7,0.4,0,0,0,0,0},{0.70,0.4,0.001,0.01,0,0,0},{0.55,0.19,0.001,0.01,0,0,0}};
unsigned int tensionIntervalTime = 50;

//---------------------------------------------------
// 关节力矩控制相关参数
//---------------------------------------------------
TorAnPID torAn_pid[4] = {{0.6,0,0,0,0,0},{0.6,0,0,0,0,0},{0.6,0,0,0,0,0},{1,0,0,0,0,0}};
unsigned int torqueIntervalTime = 50;

modbus::modbus(QObject *parent)
    :QThread(parent)
    , lastRequest(nullptr)
    , modbusDevice(nullptr)
{
    // Instantiation of modbusDevice
    if(modbusDevice){
        modbusDevice->disconnectDevice();
        delete modbusDevice;
        modbusDevice = nullptr;
    }
    modbusDevice = new QModbusRtuSerialMaster(this);
    if(!modbusDevice){
        QMessageBox::warning(0,tr("error"),tr("Could not create Modbus master"),QMessageBox::Ok);
    }
    // Instantiation of timer
    tensionCtrlTimer = new QTimer(this);
    cycleJointTimer = new QTimer(this);
    linearControlTimer = new QTimer(this);
    replayTimer = new QTimer(this);
    teachTimer = new QTimer(this);
    torqueTimer = new QTimer(this);
    QObject::connect(tensionCtrlTimer, SIGNAL(timeout()), this, SLOT(ModTensionSet()));
    QObject::connect(cycleJointTimer, SIGNAL(timeout()), this, SLOT(CirculJoint()));
    QObject::connect(linearControlTimer, SIGNAL(timeout()), this, SLOT(LinearControl()));
    QObject::connect(teachTimer, SIGNAL(timeout()), this, SLOT(TeachRecord()));    
    QObject::connect(replayTimer, SIGNAL(timeout()), this, SLOT(replayTeach()));  
    QObject::connect(torqueTimer, SIGNAL(timeout()), this, SLOT(torqueControl()));
}

modbus::~modbus()
{

}

//---------------------------------------
// Function: initial the cable length
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::InitialCableLen()
{
    // Initialization of all six cables
    // 每次界面电机start(运行)都需要初始化绳长一次
    cableLenInit[0] = 139.769;
    cableLenInit[1] = 234.459;
    cableLenInit[2] = 234.459;
    cableLenInit[3] = 139.769;
    cableLenInit[4] = 392.226;
    cableLenInit[5] = 391.833;
}

//---------------------------------------
// Function: initial the com and connect the com with Modbus protocol
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotMdSerialInit()
{
    if(!modbusDevice)
        return;

    if(modbusDevice->state() != QModbusDevice::ConnectedState){
        modbusDevice->setConnectionParameter(QModbusDevice::SerialPortNameParameter,"COM9");
        modbusDevice->setConnectionParameter(QModbusDevice::SerialBaudRateParameter,QSerialPort::Baud19200);
        modbusDevice->setConnectionParameter(QModbusDevice::SerialDataBitsParameter,QSerialPort::Data8);
        modbusDevice->setConnectionParameter(QModbusDevice::SerialParityParameter,QSerialPort::NoParity);
        modbusDevice->setConnectionParameter(QModbusDevice::SerialStopBitsParameter,QSerialPort::TwoStop);
        modbusDevice->setTimeout(1);
        modbusDevice->setNumberOfRetries(3);
        if(!modbusDevice->connectDevice()){
            QMessageBox::warning(0, tr("error"),tr("Connect failed: ") + modbusDevice->errorString(), QMessageBox::Ok);
        } else {
            QMessageBox::warning(0, tr("error"),tr("Connect successful: ") + modbusDevice->errorString(), QMessageBox::Ok);
        }
    } else{
        modbusDevice->disconnectDevice();
    }
}

//---------------------------------------
// Function: read data of server address with Modbus protocol
// Input parameter: Seraddress: server address
//                  Startaddress: start address
//                  number: register number
// Output parameter: no
//---------------------------------------
void modbus::readModbus(uint Seraddress, int Startaddress, uint number) //定义读modbus函数
{
    if(!modbusDevice)
        return;

    QModbusDataUnit readUnit(QModbusDataUnit::HoldingRegisters,Startaddress,number);//发送的数据信息（数据类型 ，起始地址，个数）
    if(auto *reply = modbusDevice->sendReadRequest(readUnit, Seraddress)){
        if(!reply->isFinished())
            connect(reply, &QModbusReply::finished, this, &modbus::readReady);
        else
            delete reply;
    } else {
        QMessageBox::warning(0, tr("error"), tr("Read error: ") + modbusDevice->errorString(), QMessageBox::Ok);
    }
}

//---------------------------------------
// Function: enter the function automatically after the read request is finished
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::readReady()
{
    auto reply = qobject_cast<QModbusReply *>(sender());
    if(!reply)
        return;

    if(reply->error() == QModbusDevice::NoError) {
        const QModbusDataUnit unit = reply->result();   //unit就是Modbus读的数据
        for(uint i = 0; i < unit.valueCount(); i++) {
            const QString entry = tr("Address: %1, Value: %2").arg(unit.startAddress() + i)
                                     .arg(QString::number(unit.value(i),
                                          unit.registerType() <= QModbusDataUnit::Coils ? 10 : 16));
            const QString EncoderNumber = tr("%1").arg(QString::number(unit.value(i),
                                                                       unit.registerType() <= QModbusDataUnit::Coils ? 10 : 16));
            qDebug()<<"read data is:"<<entry;   //显示数据entry
            emit encReturn(EncoderNumber);  //encReturn信号发射数据entry
            //emit encNumberReturn(EncoderNumber);
        }
    } else if(reply->error() == QModbusDevice::ProtocolError) {
        QMessageBox::warning(0, tr("error"), tr("Read response error: %1 (Mobus exception: 0x%2)").
                             arg(reply->errorString()).
                             arg(reply->rawResult().exceptionCode(), -1, 16), QMessageBox::Ok);
    } else {
        QMessageBox::warning(0, tr("error"), tr("Read response error: %1 (code: 0x%2)").
                             arg(reply->errorString()).
                             arg(reply->error(), -1, 16), QMessageBox::Ok);
    }

    reply->deleteLater();
}

//---------------------------------------
// Function: write data into server address with Modbus protocol
// Input parameter: Seraddress: server address
//                  Startaddress: start address
//                  Data: data(int type)
//                  writeLength: 0: write 16 function code(0x06), 1: write 32 function code(0x10)
//---------------------------------------
void modbus::writeModbus(uint Seraddress, int Startaddress, qint32 Data, bool writeLength)
{
    if(!modbusDevice)
        return;

    QModbusDataUnit writeUnit(QModbusDataUnit::HoldingRegisters,Startaddress,writeLength+1); //发送的数据信息（数据类型 ，起始地址，个数）0x030b

    int hex;
    if(writeLength == 0)
    {
        hex = Data;
        writeUnit.setValue(0,Data); // 相当于写单个寄存器
    }
    else {
        hex = Data>>16;
        qDebug()<<"hex1"<<hex;
        writeUnit.setValue(1,hex); // 相当于写单个寄存器
        hex = Data;
        qDebug()<<"hex2"<<hex;
        writeUnit.setValue(0,hex); // 相当于写单个寄存器
    }
    if(auto *reply = modbusDevice->sendWriteRequest(writeUnit, Seraddress)){// server address   sendWriteRequest是向服务器写数据
        if(!reply->isFinished()){ //reply Returns true when the reply has finished or was aborted.
            if(reply->error() == QModbusDevice::NoError)
                qDebug()<<"no error";

        } else {
            reply->deleteLater();
        }
    } else {
        QMessageBox::warning(0, tr("error"), tr("Write error: ") + modbusDevice->errorString(), QMessageBox::Ok);
    }
}

//---------------------------------------
// Function: slot function, after click the "SEND" button
// Input parameter: TensionOrAngle: 1(tension control mode), 2(velocity control mode), 3(sine function mode)
// Output parameter: no
//---------------------------------------
void modbus::slotMdSerialCtrl(uint tensionOrAngle, int *Data)
{
    if(tensionOrAngle == 0)// tension control mode
    {
        ControlMode = 0;
        for(int i=0; i<6; i++)
            AimTension[i] = Data[i];
        //tension_pid[5].KP = 0.01 * Data[6];
        //tension_pid[5].KD = 0.01 * Data[7];
        //tension_pid[5].KI = 0.001 * Data[8];
        tensionCtrlTimer->start(tensionIntervalTime);
        cycleJointTimer->stop();
        linearControlTimer->stop();
        teachTimer->stop();
        replayTimer->stop();
    }
    else if(tensionOrAngle == 1)// Joint angle control mode
    {
        ControlMode = 1;
        InitialCableLen();
        cycleJointTimer->start(jointIntervalTime);
        tensionCtrlTimer->stop();
        linearControlTimer->stop();
        teachTimer->stop();
        replayTimer->stop();
        emit sigMdStopplot(); // have to stop the plot time in mainwindow because they interfere
        for(int i=0; i<4; i++)
        {
            thetaEnd[i] = Data[i]*3.14/180; // Get the four degree joint angle
        }
        setJointCirCount = Data[4];
    }
    else if(tensionOrAngle == 2)// PTP control mode
    {
        ControlMode = 2;
        InitialCableLen();
    }
    else if(tensionOrAngle == 3)// Move line control mode
    {
        ControlMode = 3;
        InitialCableLen();
        linearControlTimer->start(lineIntervalTime);
        tensionCtrlTimer->stop();
        cycleJointTimer->stop();
        teachTimer->stop();
        replayTimer->stop();
        emit sigMdStopplot();
    }
    else// TORQUE CONTROL MODE
    {
        ControlMode = 4;
        for(int i=0; i<4; i++)
        {
            //tempAngle[i] = Data[i];
        }
        torAn_pid[3].KP = Data[4]*0.01;
        torAn_pid[3].KD = Data[5]*0.01;
        torAn_pid[3].KI = Data[6]*0.001;
        torqueTimer->start(50);
    }
}

/*
void modbus::slotMdSerialCtrl(unsigned int TensionOrAngle,int *Data);   //send按钮的槽事件
{
    // tension set control
    if(TensionOrAngle == 1)   //tenOrvel表示力矩模式
    {
        ModAimTension[0] = Data[0];     //ModAimTension就是实时的期望拉力
        Modtension_pid[0].KPTight = 0.01*Data[1];
        Modtension_pid[0].KPLoose = 0.01*Data[1];
        Modtension_pid[0].KI = 0.001*Data[2];
        Modtension_pid[0].KD = 0.01*Data[3];

        // 设置力矩模式
        //writeModbus(1,0x0200,2,0);
        ModtensionCtrlTimer->start(50);     //ModtensionCtrlTimer是一个QTimer对象，持续发射timeout()信号，周期50ms
        //writeModbus(1,0x0703,ModAimTension[0],0);
        // 使能电机驱动器
        //writeModbus(1,0x030b,1,0);
    }
    // velocity set control
    else if(tenOrvel == 2)
    {
        // 首先需要设置速度模式H0200设为0
        writeModbus(1,0x0200,0,0);
        // 设置速度值
        writeModbus(1,0x0603,Data[0],0);
        // 使能电机驱动器
        writeModbus(1,0x030b,1,0);
    }
    // sine tension value control
    else if(tenOrvel == 3)
    {
        ModSineFlag = 1;
        ModAmplitude = float(Data[0]);
        Modtension_pid[0].KPTight = 0.01*Data[1];
        Modtension_pid[0].KPLoose = 0.01*Data[1];
        Modtension_pid[0].KI = 0.001*Data[2];
        Modtension_pid[0].KD = 0.01*Data[3];
        ModPeriod = 20 * float(Data[4]);

        ModtensionCtrlTimer->start(50);
    }

}
*/

//---------------------------------------
// Function: update tension value, execute in pid control
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::ModTensionValueUpdate()
{
    ModTensionSensor[0] = tension_y[receive_count_tension];
    ModTensionSensor[1] = tension_y2[receive_count_tension];
    ModTensionSensor[2] = tension_y3[receive_count_tension];
    ModTensionSensor[3] = tension_y4[receive_count_tension];
    ModTensionSensor[4] = tension_y5[receive_count_tension];
    ModTensionSensor[5] = tension_y6[receive_count_tension];
}

//---------------------------------------
// Function: pid control algorithm, run in tensionctrl timer interruption
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::ModTensionSet()    //信号来自slotModbusCtrl，QTimer的槽
{
    ModTensionValueUpdate();
    // Incremental PID
    float ThisError, pError, dError, iError, temp;
    for(int i=0; i<6; i++)
    {
        ModTensionSensor[0] = ModTensionSensor[5];
        // it exceed the max tension motor must be stop
        if(ModTensionSensor[i]>8000)
        {
            qDebug()<<"the tension is out of the max value, too dangerous";
            tensionCtrlTimer->stop();
            // motor must be stopped!!!
            writeModbus(i+1,0x3100,0,0);// 失能电机
        }
        else
        {
            ThisError = AimTension[i] - ModTensionSensor[i]; //定义误差e=期望拉力-实际拉力
            pError = ThisError - Modtension_pid[i].LastError; // e(k)-e(k-1)
            iError = ThisError; // e(k)
            dError = ThisError - 2*(Modtension_pid[i].LastError) + Modtension_pid[i].PreError; // e(k)-2e(k-1)+e(k-2)
            Modtension_pid[i].PreError = Modtension_pid[i].LastError;
            Modtension_pid[i].LastError = ThisError;
            if(ThisError > 50)
            {
                temp = Modtension_pid[i].KPTight*pError+Modtension_pid[i].KI*iError+Modtension_pid[i].KD*dError;
                Modtension_pid[i].Output = Modtension_pid[i].Output + temp;
            }
            else if(ThisError < -50)
            {
                temp = Modtension_pid[i].KPLoose*pError+Modtension_pid[i].KI*iError+Modtension_pid[i].KD*dError;
                Modtension_pid[i].Output = Modtension_pid[i].Output + temp;
            }
            else
            {
                Modtension_pid[i].Output = Modtension_pid[i].Output;
            }
            if(Modtension_pid[i].Output > 100)
                Modtension_pid[i].Output = 100;
            if(Modtension_pid[i].Output < -100)
                Modtension_pid[i].Output = -100;
            writeModbus(i+1,0x0703,int(Modtension_pid[i].Output),0);  //发送转矩信号 .Output PID的输出
        }
        qDebug()<<Modtension_pid[0].Output;
    }
}

//---------------------------------------
// Function: disable motor controller
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotDisableMotor()
{
    // 失能电机驱动器
    for(int i=1; i<7; i++)
    {
        writeModbus(i,0x3100,0,0);

    }
}

//---------------------------------------
// Function: read Holding Register
// Input parameter: no
// Output parameter: no
/*
//---------------------------------------
void modbus::slotModbusRead()   //read按钮的槽事件
{
    //读增量编码器
    readModbus(1,0x0b07,2);
}
*/

//---------------------------------------
// Function: Initialization
// Input parameter: no
// Output parameter: no
//---------------------------------------
/*
void modbus::slotInitialization()   //初始化电机参数 槽事件
{
    for(int i=1; i<7; i++)
    {
        writeModbus(i,0x0200,2,0);  //转矩模式
        writeModbus(i,0x3100,1,0);  //VDI5端口给高电平，驱动器使能
        writeModbus(i,0x0711,0,0);
        writeModbus(i,0x0713,0x003c,0); //正转最高速度60rpm
        writeModbus(i,0x0714,0x003c,0); //反转最高速度60rpm

    }
}
*/

void modbus::slotMdSerialClose()
{
    tensionCtrlTimer->stop();
    torqueTimer->stop();
    cycleJointTimer->stop();
    linearControlTimer->stop();
    teachTimer->stop();
    replayTimer->stop();
    modbusDevice->disconnectDevice();
}

//---------------------------------------
// Function: line control slot function
// Input parameter: no
// Output parameter: no
// Statement: for joint control and line control
//---------------------------------------
void modbus::LinearControl()
{
    double tempAngle[4];
    double cableLen[6],cableLenDeta[6];
    double aimCircle[6];
    double vel[6];
    double inteTime;
    inteTime = lineIntervalTime/1000;// detaT(s)
    // 直线来回运动，需要判断运动过程
    if(linearCount == 0)
    {
        InverseLine = 0;
        lineCycleCount++;
    }
    if(linearCount == 100)
    {
        InverseLine = 1;
    }
    if(InverseLine == 0)
    {
        linearCount++;
    }
    else
    {
        linearCount--;
    }
    // 到达运动周期后，停止运动
    if(lineCycleCount == setLineCirCount)
    {
        lineCycleCount = 0;
        linearCount = 0;
        InverseJoint = 0;
        for(int i=0; i<6; i++)// Stop all motor
        {
            writeModbus(i+1,0x0603,0,0);
        }
        linearControlTimer->stop();
        emit sigMdStartplot();
    }
    else
    {
        for(int i=0; i<6; i++)
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
            cableLenInit[i] = cableLen[i];
            aimCircle[i] = -1*cableLenDeta[i]/(2*pi*rollRadius); // 计算间隔的电机转动圈数
            qDebug()<<"aimCircle"<<i<<"is"<<aimCircle[i];
            vel[i] = double(60*aimCircle[i])/double(inteTime); // 计算电机转速
            qDebug()<<"vel"<<i<<"is"<<vel[i];
        }
        for(int i=0; i<6; i++)
        {
            writeModbus(i+1,0x0603,vel[i],0);// 发送电机速度指令
        }
    }
}

//---------------------------------------
// Function: joint control slot function
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::CirculJoint()
{
    double cableLen[6],cableLenDeta[6];
    double aimCircle[6];
    double vel[6];
    double inteTime;
    inteTime = jointIntervalTime/1000; // detaT(s)
    // 关节角来回运动，需要判断运动过程
    if(jointCount == 0)
    {
        InverseJoint = 0;
        jointCycleCount++;
    }
    if(jointCount == thetaTf)
    {
        InverseJoint = 1;
    }
    if(InverseJoint == 0)
    {
        jointCount++;
    }
    else
    {
        jointCount--;
    }
    // 到达运动周期后，停止运动
    if(jointCycleCount == setJointCirCount)
    {
        jointCycleCount = 0;
        jointCount = 0;
        InverseJoint = 0;
        for(int i=0; i<6; i++)// Stop all motor
        {
            writeModbus(i+1,0x0603,0,0);
        }
        cycleJointTimer->stop();
        emit sigMdStartplot();
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
                t = jointCount;
                a0[a] = thetaStart[a];
                a1[a] = 0;
                a2[a] = 3*(thetaEnd[a] - thetaStart[a])/qPow(thetaTf,2);
                a3[a] = -2*(thetaEnd[a] - thetaStart[a])/qPow(thetaTf,3);
                theta[a] = a0[a] + a1[a]*t + a2[a]*qPow(t,2) + a3[a]*qPow(t,3);
            }
            CalculateCabelLen(i,cableLen,theta); // 计算当前绳长
            cableLenDeta[i] = cableLen[i] - cableLenInit[i]; // 相邻时间间隔的绳长变化量
            cableLenInit[i] = cableLen[i];
            aimCircle[i] = -1*cableLenDeta[i]/(2*pi*rollRadius); // 计算间隔的电机转动圈数
            qDebug()<<"aimCircle"<<i<<"is"<<aimCircle[i];
            vel[i] = double(60*aimCircle[i])/double(inteTime); // 计算电机转速
            qDebug()<<"vel"<<i<<"is"<<vel[i];
        }
        for(int i=0; i<6; i++)
        {
            writeModbus(i+1,0x0603,vel[i],0); // 发送电机速度指令
        }
    }
}

//---------------------------------------
// Function: demonstration start function from mainwindow techstart click
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotMdTeachStart()
{
    teachTimer->start(teachIntervalTime);
    // 保持绳索上的张力一定
    for(int i=0; i<6; i++)
        AimTension[i] = 300;
    tensionCtrlTimer->start(tensionIntervalTime);
}

//---------------------------------------
// Function: demonstration stop function from mainwindow techstop click
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotMdTeachStop()
{
    teachTimer->stop();
    tensionCtrlTimer->stop();
}

//---------------------------------------
// Function: teach timer slot, record
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::TeachRecord()
{
    // 记录电机转过的圈数
    MoRecord1[teachRecordCout] = Motor1Count[receive_count_mocount];
    MoRecord2[teachRecordCout] = Motor2Count[receive_count_mocount];
    MoRecord3[teachRecordCout] = Motor3Count[receive_count_mocount];
    MoRecord4[teachRecordCout] = Motor4Count[receive_count_mocount];
    MoRecord5[teachRecordCout] = Motor5Count[receive_count_mocount];
    MoRecord6[teachRecordCout] = Motor6Count[receive_count_mocount];
    // 记录IMU数据
    elbowxRecord[teachRecordCout] = elbow_x[receive_count_angle];
    elbowyRecord[teachRecordCout] = elbow_y[receive_count_angle];
    elbowzRecord[teachRecordCout] = elbow_z[receive_count_angle];
    shoulder_x[teachRecordCout] = shoulder_x[receive_count_angle];
    shoulder_y[teachRecordCout] = shoulder_y[receive_count_angle];
    shoulder_z[teachRecordCout] = shoulder_z[receive_count_angle];
    teachRecordCout++;
}

//---------------------------------------
// Function: replay demonstration motion from mainwindow replayteach click
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotMdReplayTeach()
{
    replayTimer->start(replayIntervalTime);
}

//---------------------------------------
// Function: replay timer slot, replay
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::ReplayTeach()
{
    double vel[6];
    if(replayCount<teachRecordCout-1)
    {
        /*
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
        */
        replayCount++;
    }
}

//---------------------------------------
// Function: torque timer slot, control joint torque
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::TorqueControl()
{

}

//---------------------------------------
// Function: from emg_tcp control cable tension
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotEmgTenctrl(unsigned int *data)
{
    if(modbusDevice->connectDevice())
    {
        for(int i=0; i<6; i++)
            AimTension[i] = data[i];
        ModTensionSet();
    }
    else
    {
        slotMdSerialInit();
    }
}

//---------------------------------------
// Function: preload control from mainwindow replayteach click
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotMdBeforeTigh(unsigned int *Data)
{
    for(int i=0; i<6; i++)
        AimTension[i] = Data[i];
    tensionCtrlTimer->start(tensionIntervalTime);
}

//---------------------------------------
// Function: calculate the length of cable i according to four joint angle
// Input parameter: temAngle(four joint angle)
// Output parameter: cableLen(the length of cable i)
//---------------------------------------
void modbus::CalculateCabelLen(uint i, double *cableLen, double* tempAngle)
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

