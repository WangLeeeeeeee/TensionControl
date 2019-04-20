#include "motorcontrol.h"

// 走直线轨迹时的关节角度中间值
// matlab: theta1:tempAngle[2] theta2:tempAngle[1] theta3:tempAngle[0] theta4:tempAngle[3]
double Testq1[21] = {0, 0,	0.032,	0.037,	0.041,	0.045,	0.05,	0.056,	0.063,	0.072,	0.083,	0.095,	0.109,	0.125,	0.143,	0.163,	0.184,	0.208,	0.234,	0.254,	0.262};
double Testq2[21] = {0, 0,	-0.095,	-0.147,	-0.188,	-0.222,	-0.253,	-0.28,	-0.304,	-0.326,	-0.345,	-0.361,	-0.376,	-0.387,	-0.396,	-0.402,	-0.405,	-0.405,	-0.401,	-0.395,	-0.393};
double Testq3[21] = {0, 0,	0.017,	0.036,	0.055,	0.075,	0.095,	0.115,	0.136,	0.157,	0.179,	0.201,	0.224,	0.247,	0.27,	0.294,	0.318,	0.343,	0.368,	0.386,	0.393};
double Testq4[21] = {0, 0,	0.229,	0.313,	0.374,	0.422,	0.461,	0.492,	0.518,	0.539,	0.555,	0.567,	0.575,	0.579,	0.579,	0.576,	0.569,	0.558,	0.543,	0.529,	0.524};
unsigned int pointAll = 21;

//---------------------------------------------------
//
//---------------------------------------------------
double cableLenInit[6] = {0,0,0,0}; // 绳长初始值
double initAngle[4] = {0,0,0,0}; // 关节角初始值
const double pi=3.1415926;
float rollRadius = 35.0; // 绕线轮半径（mm）
double MODMAXSPEED=120; // 正转最大速度限制
double MODMINSPEED=120; // 反转最大速度限制
unsigned int ControlMode = 0; // control mode

//---------------------------------------------------
// 关节角控制相关参数
//---------------------------------------------------
double thetaStart[4] = {0,0,0,0}; // 关节角规划的起始角度
double thetaEnd[4] = {0,0,0,0}; // 关节角规划的终止角度（由界面设定决定）
unsigned int jointIntervalTime = 100; // 关节角运动定时器周期ms（关节角规划的detaT） 注意：这个周期同时也是modbus发送指令的周期，不能太小
bool InverseJoint = 0; // 来回运行
unsigned int jointCount = 0; // 关节角运动当前时间=jointCount*detaT
unsigned int jointCycleCount = 0; // 关节角当前运行来回计数
unsigned int setJointCirCount = 0; // 界面设定的关节角来回数
double thetaTf = 15.0; // 关节角规划的整个时间（t = thetaTf*JointIntervalTime）

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

motorcontrol::motorcontrol()
{
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
    QObject::connect(replayTimer, SIGNAL(timeout()), this, SLOT(ReplayTeach()));
    QObject::connect(torqueTimer, SIGNAL(timeout()), this, SLOT(TorqueControl()));
}

//---------------------------------------
// Function: initial the cable length
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::InitialCableLen(double* initAngle)
{
    // Initialization of all six cables
    // 每次界面电机send(运行)都需要初始化绳长一次
    for(int i=0; i<6; i++)
        CalculateCabelLen(i,cableLenInit,initAngle);
}

//---------------------------------------
// Function: limit the max speed
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::InitialMotor()
{
    for(int i=0; i<6; i++)
    {
        emit sigMotorControl(i+1,0x0607,MODMAXSPEED,0); //正转最高速度120rpm
        emit sigMotorControl(i+1,0x0608,MODMINSPEED,0); //反转最高速度120rpm
    }
}

//---------------------------------------
// Function: slot function, after click the "SEND" button
// Input parameter: TensionOrAngle: 1(tension control mode), 2(velocity control mode), 3(sine function mode)
// Output parameter: no
//---------------------------------------
void motorcontrol::slotMdSerialCtrl(uint tensionOrAngle, int *Data)
{
    qDebug()<<"motorcontrol slotmdSerialCtrl: "<<QThread::currentThreadId();

    switch(tensionOrAngle){
    case TENSIONCONTROL:
        ControlMode = TENSIONCONTROL;
        cycleJointTimer->stop();
        linearControlTimer->stop();
        tensionCtrlTimer->start(tensionIntervalTime);
        for(int i=0; i<6; i++) // 将驱动器设置为转矩模式
        {
            emit sigMotorControl(i+1,MODESELECT,TORQUEMODE,0);
        }
        for(int i=0; i<6; i++) // 设置目标张力
            AimTension[i] = Data[i];
        break;
    case JOINTANGLECONTROL:
        ControlMode = JOINTANGLECONTROL;
        linearControlTimer->stop();
        tensionCtrlTimer->stop();
        cycleJointTimer->start(jointIntervalTime);
        for(int i=0; i<6; i++)// 将驱动器设置为速度模式
            emit sigMotorControl(i+1,MODESELECT,VELOCITYMODE,0);       
        InitialCableLen(initAngle); // 初始化绳长
        for(int i=0; i<4; i++)
            thetaEnd[i] = Data[i]*3.14/180; // 设置目标关节角
        setJointCirCount = Data[4]; // 设置来回的循环数
        break;
    case JOINTWITHTENSION:
        ControlMode = JOINTWITHTENSION;
        for(int i=0; i<6; i++)// 将驱动器设置为速度模式
            emit sigMotorControl(i+1,MODESELECT,VELOCITYMODE,0);
        InitialCableLen(initAngle);
        for(int i=0; i<4; i++)
            thetaEnd[i] = Data[i]*3.14/180; // 设置目标关节角
        setJointCirCount = Data[4]; // 设置来回的循环数
        break;
    case LINEARCONTROL:
        ControlMode = LINEARCONTROL;
        cycleJointTimer->stop();
        tensionCtrlTimer->stop();
        linearControlTimer->start(lineIntervalTime);
        for(int i=0; i<6; i++)// 将驱动器设置为速度模式
        {
            emit sigMotorControl(i+1,MODESELECT,VELOCITYMODE,0);
        }
        setLineCirCount = Data[0];
        InitialCableLen(initAngle);
        break;
    case JOINTTORQUECONTROL:
        ControlMode = JOINTTORQUECONTROL;
        for(int i=0; i<6; i++) // 将驱动器设置为转矩模式
            emit sigMotorControl(i+1,MODESELECT,TORQUEMODE,0);
        tensionCtrlTimer->stop();
        torqueTimer->start(50);
        break;
    default:
        break;
    }
}

//---------------------------------------
// Function: update tension value, execute in pid control
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::ModTensionValueUpdate()
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
void motorcontrol::ModTensionSet()    //信号来自slotModbusCtrl，QTimer的槽
{
    ModTensionValueUpdate();
    // Incremental PID
    float ThisError, pError, dError, iError, temp;
    for(int i=0; i<6; i++)
    {
        /* PID闭环
        ModTensionSensor[0] = ModTensionSensor[5];
        // it exceed the max tension motor must be stop
        if(ModTensionSensor[i]>8000)
        {
            qDebug()<<"the tension is out of the max value, too dangerous";
            tensionCtrlTimer->stop();
            // motor must be stopped!!!
            emit sigMotorControl(i+1,ENBLESET,DISABLE,0);// 失能电机
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
            emit sigMotorControl(i+1,TORQUESET,int(Modtension_pid[i].Output),0);  //发送转矩信号 .Output PID的输出
        }
        qDebug()<<Modtension_pid[0].Output;
        */
        // 开环
        emit sigMotorControl(i+1,TORQUESET,-AimTension[i]*0.5,0);  //发送转矩信号 ([-300.0 300.0])
    }
}

//---------------------------------------
// Function: disable motor controller
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::slotDisableMotor()
{
    qDebug()<<"stop the motor";
    tensionCtrlTimer->stop();
    linearControlTimer->stop();
    cycleJointTimer->stop();
    // 失能电机驱动器
    for(int i=0; i<6; i++)
    {
        emit sigMotorControl(i+1,MODESELECT,DISABLE,0);
    }
}

//---------------------------------------
// Function: line control slot function
// Input parameter: no
// Output parameter: no
// Statement: for joint control and line control
//---------------------------------------
void motorcontrol::LinearControl()
{
    qDebug()<<"LineControl "<<"the thread is:"<<QThread::currentThreadId();
    double tempAngle[4];
    double cableLen[6],cableLenDeta[6];
    double aimCircle[6];
    double vel[6];
    double inteTime;
    inteTime = double(lineIntervalTime)/1000;// detaT(s)
    if(lineCycleCount == setLineCirCount) // 执行完所有次数
    {
        lineCycleCount = 0;
        linearCount = 0;
        InverseJoint = 0;
        for(int i=0; i<6; i++)// Stop all motor
        {
            emit sigMotorControl(i+1,SPEEDSET,0,1);
        }
        linearControlTimer->stop();
    }
    else{
        for(int i=0; i<6; i++)
        {
            tempAngle[0] = Testq3[linearCount];
            tempAngle[1] = Testq2[linearCount];
            tempAngle[2] = Testq1[linearCount];
            tempAngle[3] = Testq4[linearCount];
            CalculateCabelLen(i,cableLen,tempAngle);
            cableLenDeta[i] = cableLen[i] - cableLenInit[i];
            cableLenInit[i] = cableLen[i];
            aimCircle[i] = cableLenDeta[i]/(2*pi*rollRadius); // 计算间隔的电机转动圈数
            vel[i] = double(60*aimCircle[i])/double(inteTime); // 计算电机转速
        }
        for(int i=0; i<6; i++)
        {
            if(i==5)
            {
                qDebug()<<"linecount is:"<<linearCount;
                qDebug()<<"Speed"<<i<<"is:"<<vel[i]*1000;
            }
            emit sigMotorControl(i+1,SPEEDSET,qint32(vel[i]*1000),1);// 发送电机速度指令
        }
        if(linearCount == 0) // 起始点
        {
            lineCycleCount++; //次数加一
            InverseLine = 0;
        }
        if(linearCount == pointAll-1) // 终止点
        {
            InverseLine = 1;
        }
        // 直线来回运动，需要判断运动过程
        if(InverseLine == 0) // 正向运动过程
        {
            linearCount++;
        }
        else // 反向运动过程
        {
            linearCount--;
        }
    }
}

//---------------------------------------
// Function: joint control slot function
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::CirculJoint()
{
    double cableLen[6],cableLenDeta[6];
    double aimCircle[6];
    double vel[6];
    double inteTime;
    inteTime = double(jointIntervalTime)/1000; // detaT(s)
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
            emit sigMotorControl(i+1,SPEEDSET,0,1);
        }
        cycleJointTimer->stop();
    }
    else
    {
        double theta[4]; // cubic poly nomial theta
        double a0[4],a1[4],a2[4],a3[4];
        unsigned int t;
        for(int a=0; a<4; a++)
        {
            t = jointCount;
            a0[a] = thetaStart[a];
            a1[a] = 0;
            a2[a] = 3*(thetaEnd[a] - thetaStart[a])/qPow(thetaTf,2);
            a3[a] = -2*(thetaEnd[a] - thetaStart[a])/qPow(thetaTf,3);
            theta[a] = a0[a] + a1[a]*t + a2[a]*qPow(t,2) + a3[a]*qPow(t,3);
        }
        for(int i=0; i<6; i++)
        {
            CalculateCabelLen(i,cableLen,theta); // 计算当前绳长
            cableLenDeta[i] = cableLen[i] - cableLenInit[i]; // 相邻时间间隔的绳长变化量
            cableLenInit[i] = cableLen[i];
            aimCircle[i] = cableLenDeta[i]/(2*pi*rollRadius); // 计算间隔的电机转动圈数
            vel[i] = double(60*aimCircle[i])/double(inteTime); // 计算电机转速
        }
        for(int i=0; i<6; i++)
        {
            if(ControlMode == JOINTANGLECONTROL) // 如果是单纯的角度位置模式
                emit sigMotorControl(i+1,SPEEDSET,qint32(vel[i]*1000),1); // 发送电机速度指令
            if(ControlMode == JOINTWITHTENSION)
            {
                if(vel[i] > 0) // 如果是速度大于0则使用力矩模式
                {
                    emit sigMotorControl(i+1,MODESELECT,TORQUEMODE,0);
                    emit sigMotorControl(i+1,TORQUESET,-80,0);
                }
                else
                {
                    emit sigMotorControl(i+1,MODESELECT,VELOCITYMODE,0);
                    emit sigMotorControl(i+1,SPEEDSET,qint32(vel[i]*1000),1);
                }
            }
            if((i==5)||(i==6))
            {
                qDebug()<<"vel"<<i<<"is"<<qint32(vel[i]*1000);
                qDebug()<<"cable 5"<<cableLen[i];
                qDebug()<<"tempAngle[3] is:"<<theta[3];
            }
        }
    }
}

//---------------------------------------
// Function: demonstration start function from mainwindow techstart click
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::slotMdTeachStart()
{
    teachTimer->start(teachIntervalTime);
    for(int i=0; i<6; i++)
        AimTension[i] = 300;
    // 保持绳索上的张力一定
    for(int i=0; i<6; i++)
        emit sigMotorControl(i+1,TORQUESET,-AimTension[i]*0.5,0);  //发送转矩信号 ([-300.0 300.0])
    //tensionCtrlTimer->start(tensionIntervalTime);
}

//---------------------------------------
// Function: demonstration stop function from mainwindow techstop click
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::slotMdTeachStop()
{
    teachTimer->stop();
    tensionCtrlTimer->stop();
}

//---------------------------------------
// Function: teach timer slot, record
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::TeachRecord()
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
void motorcontrol::slotMdReplayTeach()
{
    replayTimer->start(replayIntervalTime);
}

//---------------------------------------
// Function: replay timer slot, replay
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::ReplayTeach()
{
    double vel[6];
    if(replayCount<teachRecordCout-1)
    {
        vel[0] = ((MoRecord1[replayCount+1]-MoRecord1[replayCount])/10000)/(teachIntervalTime/(1000*60));
        vel[1] = ((MoRecord2[replayCount+1]-MoRecord1[replayCount])/10000)/(teachIntervalTime/(1000*60));
        vel[2] = ((MoRecord3[replayCount+1]-MoRecord1[replayCount])/10000)/(teachIntervalTime/(1000*60));
        vel[3] = ((MoRecord4[replayCount+1]-MoRecord1[replayCount])/10000)/(teachIntervalTime/(1000*60));
        vel[4] = ((MoRecord5[replayCount+1]-MoRecord1[replayCount])/10000)/(teachIntervalTime/(1000*60));
        vel[5] = ((MoRecord6[replayCount+1]-MoRecord1[replayCount])/10000)/(teachIntervalTime/(1000*60));
        qDebug()<<"the speed is:"<<vel[5]<<"n/min";
        for(int i=0; i<6; i++)
        {
            emit sigMotorControl(i+1,SPEEDSET,qint32(vel[i]*1000),1); // 发送电机速度指令
        }
        replayCount++;
    }
}

//---------------------------------------
// Function: torque timer slot, control joint torque
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::TorqueControl()
{

}

//---------------------------------------
// Function: from emg_tcp control cable tension
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::slotEmgTenctrl(unsigned int *data)
{
    /*
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
    */
}

//---------------------------------------
// Function: preload control from mainwindow replayteach click
// Input parameter: no
// Output parameter: no
//---------------------------------------
void motorcontrol::slotMdBeforeTigh(unsigned int *Data)
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
void motorcontrol::CalculateCabelLen(uint i, double *cableLen, double* tempAngle)
{
    switch (i) {
    case 0:
       cableLen[i] = qPow((qPow((30*cos(tempAngle[0])*sin(tempAngle[2]) +
                      125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                       (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                      30*cos(tempAngle[2])*cos(tempAngle[1] - pi/2)*sin(tempAngle[0]) +
                       (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 + 142),2) +
                       qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 -
                             30*cos(tempAngle[2])*cos(tempAngle[0]) +
                             125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 +
                             30*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])*sin(tempAngle[0]) + 110),2) +
                       qPow(((343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 - 125*cos(tempAngle[1] - pi/2) +
                             30*sin(tempAngle[0])*sin(tempAngle[1] - pi/2) + 33),2)),0.5);break;
    case 1:
        cableLen[i] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) - (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 + 37),2) +
                        qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 + 125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                         (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 137),2) +
                        qPow(((343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 - 125*cos(tempAngle[1] - pi/2) + 20),2)),0.5);break;
    case 2:
        cableLen[i] = qPow((qPow((125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                        (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 -
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 + 37),2) +
                        qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 - 125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) +
                         (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 137),2) +
                        qPow((125*cos(tempAngle[1] - pi/2) + (343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 + 20),2)),0.5);break;
    case 3:
        cableLen[i] = qPow((qPow((30*cos(tempAngle[0])*sin(tempAngle[2]) -
                        125*cos(tempAngle[2])*sin(tempAngle[1] - pi/2) - (343*sin(tempAngle[2])*sin(tempAngle[0]))/2 +
                        30*cos(tempAngle[2])*cos(tempAngle[1] - pi/2)*sin(tempAngle[0]) +
                        (343*cos(tempAngle[2])*cos(tempAngle[0])*cos(tempAngle[1] - pi/2))/2 - 142),2) +
                        qPow(((343*cos(tempAngle[2])*sin(tempAngle[0]))/2 - 30*cos(tempAngle[2])*cos(tempAngle[0]) -
                         125*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) + (343*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 +
                         30*cos(tempAngle[1] - pi/2)*sin(tempAngle[2])*sin(tempAngle[0]) + 30),2) +
                        qPow((125*cos(tempAngle[1] - pi/2) + (343*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 +
                         30*sin(tempAngle[0])*sin(tempAngle[1] - pi/2) + 33),2)),0.5);break;
    case 4:
        cableLen[i] = qPow((qPow((118*cos(tempAngle[3])*cos(tempAngle[1] - pi/2) + (607*cos(tempAngle[0])*sin(tempAngle[1] - pi/2))/2 +
                        118*cos(tempAngle[1] - pi/2)*sin(tempAngle[3]) + 118*cos(tempAngle[0])*cos(tempAngle[3])*sin(tempAngle[1] - pi/2) -
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
                         (607*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 - 118*cos(tempAngle[3])*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                         118*sin(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2)),2)),0.5);break;
    case 5:
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
                         (607*cos(tempAngle[0])*cos(tempAngle[1] - pi/2)*sin(tempAngle[2]))/2 + 118*cos(tempAngle[3])*sin(tempAngle[2])*sin(tempAngle[1] - pi/2) -
                         118*sin(tempAngle[2])*sin(tempAngle[3])*sin(tempAngle[1] - pi/2)),2)),0.5);break;
    default:
        break;
    }
}

