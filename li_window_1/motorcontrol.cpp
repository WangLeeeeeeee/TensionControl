#include "motorcontrol.h"

int control_mode = 0;
unsigned int UI[6];
unsigned int MotorParam[6];

bool flag = 0;

// Begin or stop self learn flag
bool selfLearn = 0;
unsigned int plusOneSecCnt = 0;

typedef struct LearnResult
{
    float aimTension[6];
    float aimElbowAngle;
    float aimShoulAnglex;
    float aimShoulAngley;
    float aimShoulAnglez;
}LearnResult;

LearnResult TestLearn[100];
unsigned int learnResultCnt = 0;

/*
float KP_TENSION[6] = {1,1,1,1,1};
float KI_TENSION[6] = {0,0,0,0,0};
float KD_TENSION[6] = {0,0,0,0,0};
float PERROR_TENSION[6] = {0,0,0,0,0};
float LERROR_TENSION[6] = {0,0,0,0,0};
double tensionCnt[6];
*/

typedef struct Tension_Control
{
    float TensionCnt;
    float Tension_error;
    float Vel_out;
    float Error_Integer;
}Tension_Control;

typedef struct Angle_Control
{
    float AngleCnt;
    float Angle_error;
    float Vel_out;
    float Error_Integer;
}Angle_Control;

PID TENSION_PID[6]={{1,0,0,0,0},{1,0,0,0,0},{1,0,0,0,0},{1,0,0,0,0},{1,0,0,0,0},{1,0,0,0,0}};
PID TENSION_PID_1[6]={{1.9,0,0,0,0},{1.9,0,0,0,0},{1.9,0,0,0,0},{1,0,0,0,0},{1.9,0,0,0,0},{1.9,0,0,0,0}};
PID Angle_PID[4]={{5,0,0.5,0,0},{5,0,0.5,0,0},{5,0,0.5,0,0},{3,0,0,0,0}};
float KP_Ten[6] = {0.5,0.5,0.5,0.5,0.5,1.5};
Tension_Control Now_tension;
Tension_Control Past_tension;
Angle_Control Now_angle;
Angle_Control Past_angle;
float MAXSPEED=200;
float MINSPEED=-200;
float tension_cnt[6];
float tension_ctrlval[6];
float angle_cnt[4];
float angle_ctrlval[4];

float MAXLR = 600;
float MINLR = -600;

int tension_ctrlmode = 0;
float aimAngle = 0;
unsigned int angleIndex = 0;

// test
unsigned int tension6_aimval;
unsigned int pre_tension6;

MotorControl::MotorControl(QObject *parent):QThread(parent)
{

}


void MotorControl::run()
{
    float angleError;

    // Every fify second
    while(1)
    {
        angleError = aimAngle + angle_cnt[angleIndex];
        qDebug()<<"angleError="<<angleError<<endl;

        if((angleError<4)&&(angleError>-4)&&(selfLearn==0))
        {
            tension6_aimval = tension_y6[receive_count_tension-1];
            tension_ctrlval[5] = tension6_aimval;
            plusOneSecCnt = 60;

            // Reach the aim angle, means learning finish, record the aim tension value
            selfLearn = 1;
            for(int i=0; i<6; i++)
                TestLearn[learnResultCnt].aimTension[i] = tension_ctrlval[i];
            if(angleIndex == 3)
                TestLearn[learnResultCnt].aimElbowAngle = aimAngle;
        }

        if(plusOneSecCnt >= 40)
        {
            // Return to the initial state
            //for(int i=0; i<6; i++)
                //tension_ctrlval[i] = 250;
            while(elbow_y[receive_count_angle-1] < -15)
            {
                tension_ctrlval[4] = 400 + 20*(-15 - elbow_y[receive_count_angle-1]);
                tension_ctrlval[5] = 100;
                Sleep(300);
            }
                plusOneSecCnt = 0;

                // Learn mode not finish
                 if(selfLearn == 0)
                {
                    // Did not reach the aim angle, puls the tension value
                    if((angleError>4) || (angleError<-4))
                    {
                        // Elbow angle control mode
                        if(angleIndex == 3)
                        {
                            tension_ctrlval[4] = 100;
                            tension_ctrlval[5] = pre_tension6 + 10*angleError;// change the tension value
                            pre_tension6 = tension_ctrlval[5];
                        }
                    }
                    /*
                    else
                    {
                        // Reach the aim angle, means learning finish, record the aim tension value
                        selfLearn = 1;
                        for(int i=0; i<6; i++)
                            TestLearn[learnResultCnt].aimTension[i] = tension_ctrlval[i];
                        if(angleIndex == 3)
                            TestLearn[learnResultCnt].aimElbowAngle = aimAngle;
                    }
                    */
                }
        }
    }

}



void MotorControl::slotUiParRec(bool TensionOrAngle, unsigned int *Data)
{
    // Tension control mode
    if(TensionOrAngle == 0)
    {
        control_mode = 1;
        for(int i=0; i<6; i++)
        {
            tension_ctrlval[i] = Data[i];
        }
    }
    // Joint angle control mode
    else if(TensionOrAngle == 1)
    {
        control_mode = 2;
        for(int i=0; i<6; i++)
        {
            angle_ctrlval[i] = Data[i];
        }
    }
}

void MotorControl::readMyCom1()
{
    // do something here
    QString SendData1;
    SendData1 = "isahioashd";
    serial1.write(SendData1.toLatin1());
}

// Send the command data every fixed time interval
void MotorControl::slotSendCommand()
{
    QString SendData;
    SendData = QString::number(long(5));
    serial1.write(SendData.toLatin1());
    /*
    switch(control_mode){
    case 0:
        qDebug()<<"now the control mode is 0"; break; // do nothing
    case 1:
        qDebug()<<"now the control mode is 1";
        flag = 0;
        //TensionControl(3,tension_ctrlval[3]);
        for(int i=4; i<6; i++){
            TensionControl(i,tension_ctrlval[i]);
        }break;

    case 2:
        qDebug()<<"now the control mode is 2";
        qDebug()<<"now the tension6 is"<<tension_ctrlval[5]<<endl;
        qDebug()<<"now the elbow angle is"<<elbow_y[receive_count_angle-1]<<endl;
        AngleControl(3,angle_ctrlval[3]);
        break;
    case 3:
        qDebug()<<"now the control mode is zhou qianqu";
        TensionControl(4,tension_ctrlval[4]);
        AngleControl(3,50);
        break;
    case 4:
        qDebug()<<"now the control mode is waizhan";
        if(tension_y2[receive_count_tension-1] < 300)
            TensionControl(1,300);
        if(tension_y3[receive_count_tension-1] < 300)
            TensionControl(2,300);
        if((tension_y2[receive_count_tension-1] > 200)&&(tension_y3[receive_count_tension-1] > 200))
            AngleControl(0,25);
        break;
    case 5:
        qDebug()<<"now the control mode is huishou";
        TensionControl(5,tension_ctrlval[5]);
        AngleControl(3,0);
        break;
    case 6:
        qDebug()<<"now the control mode is waizhan huishou";
        TensionControl(1,tension_ctrlval[1]);
        TensionControl(2,tension_ctrlval[2]);
        break;
    case 7:
        qDebug()<<"now the control mode is jian qianqu";
        TensionControl(0,tension_ctrlval[0]);
        //TensionControl(4,tension_ctrlval[4]);
        //TensionControl(5,tension_ctrlval[5]);
        AngleControl(1,50);
        break;
    case 8:
        qDebug()<<"now the control mode is huishou jianqianqu";
        TensionControl(3,tension_ctrlval[3]);
        //TensionControl(4,tension_ctrlval[4]);
        //TensionControl(5,tension_ctrlval[5]);
        //AngleControl(1,10);
        break;

    }
    */

}

// Com open configure which connect with the 'open com' button
void MotorControl::slotSerialInit()
{
    serial1.setPortName("COM10");
    serial1.setBaudRate(QSerialPort::Baud9600);
    serial1.setDataBits(QSerialPort::Data8);
    serial1.setStopBits(QSerialPort::OneStop);
    serial1.setFlowControl(QSerialPort::NoFlowControl);
    serial1.setParity(QSerialPort::NoParity);
    serial1.close();

    if(serial1.open(QIODevice::ReadWrite))
    {
        connect(&serial1,SIGNAL(readyRead()),this,SLOT(readMyCom1()));
        QMessageBox::information(0,tr("open sucessful"),tr("sucessful open com"),QMessageBox::Ok);
    }
    else
    {
        QMessageBox::critical(0,tr("open failed"),tr("cannot open com"),QMessageBox::Ok);
        return;
    }

    send_timer = new QTimer(this);
    connect(send_timer, SIGNAL(timeout()), this, SLOT(slotSendCommand()));
    int send_interval = TIME_SEND_INTERVAL;
    send_timer->start(send_interval);
}

// Com close
void MotorControl::slotSerialClose()
{
    serial1.close();
}

// Get the tension data and the angle data
void MotorControl::UpdateAllState()
{
    tension_cnt[0] = tension_y[receive_count_tension-1];
    tension_cnt[1] = tension_y2[receive_count_tension-1];
    tension_cnt[2] = tension_y3[receive_count_tension-1];
    tension_cnt[3] = tension_y4[receive_count_tension-1];
    tension_cnt[4] = tension_y5[receive_count_tension-1];
    tension_cnt[5] = tension_y6[receive_count_tension-1];

    angle_cnt[0] = shoulder_x[receive_count_angle-1];
    angle_cnt[1] = -1 * shoulder_y[receive_count_angle-1];
    angle_cnt[2] = shoulder_z[receive_count_angle-1];
    angle_cnt[3] = -1 * elbow_y[receive_count_angle-1];
}

// Calculate PID
float MotorControl::PIDCalculate(PID *pid_tension,float ThisError)
{
    float pError,dError,iError,temp;
    pError=ThisError-pid_tension->LastError;
    iError=ThisError;
    dError=ThisError-2*(pid_tension->LastError)+pid_tension->PreError;

    temp=pid_tension->KP*pError+pid_tension->KI*iError+pid_tension->KD*dError;

    pid_tension->PreError=pid_tension->LastError;
    pid_tension->LastError=ThisError;

    return temp;
}

// Control the index motor with i tension value
void MotorControl::TensionControl(int index,float i)
{
    UpdateAllState();

    QString SendData;
    float speed_out;

    Now_tension.TensionCnt=tension_cnt[index];
    Now_tension.Tension_error=i-Now_tension.TensionCnt;
    qDebug()<<"the tension error is:"<<Now_tension.Tension_error<<endl;

    if((Now_tension.Tension_error > 70) || (Now_tension.Tension_error < -70))
    {
        speed_out = KP_Ten[index]*Now_tension.Tension_error;
        //if(speed_out>MAXSPEED)	speed_out=MAXSPEED;
        //if(speed_out<MINSPEED)	speed_out=MINSPEED;
        //if(speed_out>MAXLR) speed_out = MAXLR;
        //if(speed_out<MINLR) speed_out = MINLR;
    }
    else
    {
        speed_out = 0;
    }

    for(int i=0; i<10; i++)
    {
        SendData = "\r";
        serial1.write(SendData.toLatin1());
    }
    //SendData = "\r" + QString::number(long(index)) + "V" + QString::number(long(speed_out)) + "\r";
    //qDebug()<<SendData<<endl;
    //serial1.write(SendData.toLatin1());

    SendData = QString::number(long(index)) + "SP" + QString::number(long(MAXSPEED)) + "\r";
    serial1.write(SendData.toLatin1());
    SendData = QString::number(long(index)) + "LR" + QString::number(long(speed_out)) + "\r";
    qDebug()<<SendData<<endl;
    serial1.write(SendData.toLatin1());
    SendData = "M\r";
    serial1.write(SendData.toLatin1());

    /*
    if(Now_tension.Tension_error>100)
    {
        static float Vel_out_last=0;
        Now_tension.Vel_out=PIDCalculate((TENSION_PID+index),Now_tension.Tension_error);
        Past_tension.Tension_error=Now_tension.Tension_error;
        Past_tension.Vel_out=Now_tension.Vel_out;
        Past_tension.Error_Integer=Now_tension.Error_Integer;
        Past_tension.TensionCnt=Now_tension.TensionCnt;
        speed_out=Now_tension.Vel_out+Vel_out_last;

        if(speed_out>MAXLR)	speed_out=MAXLR;
        if(speed_out<MINLR)	speed_out=MINLR;

        Vel_out_last=speed_out;
    }
    else if(Now_tension.Tension_error<-100)
    {
        static float Vel_out_last=0;
        Now_tension.Vel_out=PIDCalculate((TENSION_PID_1+index),Now_tension.Tension_error);
        Past_tension.Tension_error=Now_tension.Tension_error;
        Past_tension.Vel_out=Now_tension.Vel_out;
        Past_tension.Error_Integer=Now_tension.Error_Integer;
        Past_tension.TensionCnt=Now_tension.TensionCnt;
        speed_out=Now_tension.Vel_out+Vel_out_last;

        if(speed_out>MAXLR)	speed_out=MAXLR;
        if(speed_out<MINLR)	speed_out=MINLR;

        Vel_out_last=speed_out;
    }
    */

    /*
    if((Now_tension.Tension_error>100)||(Now_tension.Tension_error<-100))
    {
        static float Vel_out_last=0;
        Now_tension.Vel_out=PIDCalculate((TENSION_PID+index),Now_tension.Tension_error);
        Past_tension.Tension_error=Now_tension.Tension_error;
        Past_tension.Vel_out=Now_tension.Vel_out;
        Past_tension.Error_Integer=Now_tension.Error_Integer;
        Past_tension.TensionCnt=Now_tension.TensionCnt;
        speed_out=Now_tension.Vel_out+Vel_out_last;

        if(speed_out>MAXSPEED)	speed_out=MAXSPEED;
        if(speed_out<MINSPEED)	speed_out=MINSPEED;

        //if(speed_out>MAXLR)	speed_out=MAXLR;
        //if(speed_out<MINLR)	speed_out=MINLR;

        Vel_out_last=speed_out;
    }

    else speed_out=0;
    SendData = QString::number(long(index)) + "V" + QString::number(long(speed_out)) + "\r";
    qDebug()<<SendData<<endl;
    serial1.write(SendData.toLatin1());
    */

    /*
    SendData = QString::number(long(index)) + "SP" + QString::number(long(MAXSPEED)) + "\r";
    serial1.write(SendData.toLatin1());
    SendData = QString::number(long(index)) + "LR" + QString::number(long(speed_out)) + "\r";
    qDebug()<<SendData<<endl;
    serial1.write(SendData.toLatin1());
    SendData = "M\r";
    serial1.write(SendData.toLatin1());
    */
}

void MotorControl::AngleControl(int index, float aim_angle)
{
    /* self learning
    UpdateAllState();

    plusOneSecCnt++;
    qDebug()<<"now the plusOneSec="<<plusOneSecCnt<<endl;

    aimAngle = aim_angle;
    angleIndex = index;

    TensionControl(4,tension_ctrlval[4]);
    TensionControl(5,tension_ctrlval[5]);
    */

    /*
    QString SendData;
    float speed_out;

    speed_out = aim_angle * 5000;

    if(index == 0)
    {
        TensionControl(0,200);
        TensionControl(3,200);
        qDebug()<<"shoulder x angle is:"<<shoulder_x[receive_count_angle-1];
        qDebug()<<"shoulder y angle is:"<<shoulder_y[receive_count_angle-1];
        qDebug()<<"shoulder z angle is:"<<shoulder_z[receive_count_angle-1];
        if(flag == 0)
        {
            for(int i=0; i<20; i++)
            {
                SendData = "1V0\r";
                serial1.write(SendData.toLatin1());
                SendData = "1HO\r";
                serial1.write(SendData.toLatin1());
                SendData = "2V0\r";
                serial1.write(SendData.toLatin1());
                SendData = "2HO\r";
                serial1.write(SendData.toLatin1());
            }
            flag = 1;
        }
        SendData = "1SP" + QString::number(long(MAXSPEED)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = "1LA" + QString::number(long(speed_out)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
        SendData = "2SP" + QString::number(long(MAXSPEED)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = "2LA" + QString::number(long(speed_out)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());

    }

    if(index == 3)
    {
        TensionControl(4,200);
        if(flag == 0)
        {
            for(int i=0; i<20; i++)
            {
                SendData = "5V0\r";
                serial1.write(SendData.toLatin1());
                SendData = "5HO\r";
                serial1.write(SendData.toLatin1());
            }
            flag = 1;
        }
        SendData = "5SP" + QString::number(long(MAXSPEED)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = "5LA" + QString::number(long(speed_out)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
    }
    */

    QString SendData;
    float detaR;
    UpdateAllState();
    Now_angle.AngleCnt = angle_cnt[index];
    Now_angle.Angle_error = aim_angle - Now_angle.AngleCnt;
    if((Now_angle.Angle_error > 3) || (Now_angle.Angle_error < -3))
    {
        detaR = 140 * Now_angle.Angle_error;
    }
    if(control_mode == 3)
    {
        SendData = "5LR" + QString::number(long(detaR)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
    }
    if(control_mode == 4)
    {
        for(int i=0; i<10; i++)
        {
            SendData = "\r";
            serial1.write(SendData.toLatin1());
        }

        SendData = "1LR" + QString::number(long(-detaR)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());

        /*
        SendData = "2LR" + QString::number(long(-detaR)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
        */
    }
    if(control_mode == 5)
    {
        SendData = "4LR" + QString::number(long(-detaR)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());

    }
    if(control_mode == 7)
    {
        for(int i=0; i<10; i++)
        {
            SendData = "\r";
            serial1.write(SendData.toLatin1());
        }

        SendData = "3LR" + QString::number(long(detaR)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
    }
    if(control_mode == 8)
    {
        SendData = "0LR" + QString::number(long(-detaR)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
        SendData = "M\r";
        serial1.write(SendData.toLatin1());
    }
    /*
    UpdateAllState();

    Now_angle.AngleCnt = angle_cnt[index];
    Now_angle.Angle_error = aim_angle - Now_angle.AngleCnt;

    if((Now_angle.Angle_error > 5) || (Now_angle.Angle_error < -5))
    {
        static float Vel_out_last=0;
        Now_angle.Vel_out=PIDCalculate((Angle_PID+index),Now_angle.Angle_error);
        Past_angle.Angle_error=Now_angle.Angle_error;
        Past_angle.Vel_out=Now_angle.Vel_out;
        Past_angle.Error_Integer=Now_angle.Error_Integer;
        Past_angle.AngleCnt=Now_tension.TensionCnt;
        speed_out=Now_angle.Vel_out+Vel_out_last;

        if(speed_out>MAXSPEED)	speed_out=MAXSPEED;
        if(speed_out<MINSPEED)	speed_out=MINSPEED;

        Vel_out_last=speed_out;
    }
    else speed_out=0;

    if(index == 3)
    {
        SendData = "4SP" + QString::number(long(MAXSPEED)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = "5SP" + QString::number(long(MAXSPEED)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = "4V" + QString::number(long(speed_out)) + "\r";
        serial1.write(SendData.toLatin1());
        SendData = "5V" + QString::number(long(-speed_out)) + "\r";
        serial1.write(SendData.toLatin1());
        qDebug()<<SendData<<endl;
    }
    */


}

void MotorControl::slotBeforeTigh()
{
    control_mode = 1;
    for(int i=0; i<6; i++)
        tension_ctrlval[i] = 200;
}

void MotorControl::slotQianqu()
{
    control_mode = 3;
    tension_ctrlval[4] = 100;
}

void MotorControl::slotWaizhan()
{
    control_mode = 4;
    //tension_ctrlval[0] = 100;
    //tension_ctrlval[3] = 100;
}

void MotorControl::slotHuishou()
{
    control_mode = 5;
    tension_ctrlval[5] = 100;
}

void MotorControl::slotHuishouWaizhan()
{
    control_mode = 6;
    tension_ctrlval[1] = 100;
    tension_ctrlval[2] = 100;
}

void MotorControl::slotJianqianqu()
{
    control_mode = 7;
    tension_ctrlval[0] = 100;
    //tension_ctrlval[5] = 600;
    //tension_ctrlval[4] = 100;
}

void MotorControl::slotHuishouJianqianqu()
{
    control_mode = 8;
    tension_ctrlval[3] = 100;
    //tension_ctrlval[5] = 100;
    //tension_ctrlval[4] = 600;
}
