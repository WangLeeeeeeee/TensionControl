#include "encoderread.h"

unsigned int receive_count_mocount = 0;
unsigned int Encoderlength = 131072;
QVector<double> time_x_mocount(Encoderlength);
QVector<double> Motor1Count(Encoderlength),Motor2Count(Encoderlength),Motor3Count(Encoderlength),Motor4Count(Encoderlength),Motor5Count(Encoderlength),Motor6Count(Encoderlength);
double max_motor_count[6]={0, 0, 0, 0, 0, 0};
double min_motor_count[6] = {0, 0, 0, 0, 0, 0};

encoderRead::encoderRead()
{
}

void encoderRead::slotReadEncoder()
{
    //qDebug()<<"encodeeRead slotReadEncoder: "<<QThread::currentThreadId();
    Motor1Count[0] = 0;
    Motor2Count[0] = 0;
    Motor3Count[0] = 0;
    Motor4Count[0] = 0;
    Motor5Count[0] = 0;
    Motor6Count[0] = 0;
    time_x_mocount[0] = 0;
    for(int i=0; i<6; i++)
        emit sigReadEncoder(i+1,0x0b07,2);
}

void encoderRead::slotDataToEncoder(QString data)
{
    //qDebug()<<"encoderRead slotDataToEncoder: "<<QThread::currentThreadId();
    if(HighOrLow == 0)
    {
        QString str1 = tr("%1").arg(data);
        bool ok;
        count1 = str1.toInt(&ok,16);
        HighOrLow = 1;
        qDebug()<<"low data is:"<<count1;
    }
    else
    {
        QString str2 = tr("%1").arg(data);
        bool ok;
        count2 = str2.toInt(&ok,16);
        qDebug()<<"high data is:"<<count2;
        count3 = count1 + count2*pow(16,4);  //count3是编码器返回的整形值
        if(count3>500000) //电机反转，编码器数值溢出
            count3 = count3 - 4294967295;
        encoderNumber++;
        switch(encoderNumber){
        case 1:
            Motor1Count[receive_count_mocount] = count3;
            max_motor_count[0] = (max_motor_count[0] > Motor1Count[receive_count_mocount]) ? max_motor_count[0] : Motor1Count[receive_count_mocount];
            min_motor_count[0] = (min_motor_count[0] < Motor1Count[receive_count_mocount]) ? min_motor_count[0] : Motor1Count[receive_count_mocount];
            break;
        case 2:
            Motor2Count[receive_count_mocount] = count3;
            max_motor_count[1] = (max_motor_count[1] > Motor2Count[receive_count_mocount]) ? max_motor_count[1] : Motor2Count[receive_count_mocount];
            min_motor_count[1] = (min_motor_count[1] < Motor2Count[receive_count_mocount]) ? min_motor_count[1] : Motor2Count[receive_count_mocount];
            break;
        case 3:
            Motor3Count[receive_count_mocount] = count3;
            max_motor_count[2] = (max_motor_count[2] > Motor3Count[receive_count_mocount]) ? max_motor_count[2] : Motor3Count[receive_count_mocount];
            min_motor_count[2] = (min_motor_count[2] < Motor3Count[receive_count_mocount]) ? min_motor_count[2] : Motor3Count[receive_count_mocount];
            break;
        case 4:
            Motor4Count[receive_count_mocount] = count3;
            max_motor_count[3] = (max_motor_count[3] > Motor4Count[receive_count_mocount]) ? max_motor_count[3] : Motor4Count[receive_count_mocount];
            min_motor_count[3] = (min_motor_count[3] < Motor4Count[receive_count_mocount]) ? min_motor_count[3] : Motor4Count[receive_count_mocount];
            break;
        case 5:
            Motor5Count[receive_count_mocount] = count3;
            max_motor_count[4] = (max_motor_count[4] > Motor5Count[receive_count_mocount]) ? max_motor_count[4] : Motor5Count[receive_count_mocount];
            min_motor_count[4] = (min_motor_count[4] < Motor5Count[receive_count_mocount]) ? min_motor_count[4] : Motor5Count[receive_count_mocount];
            break;
        case 6:
            Motor6Count[receive_count_mocount] = count3;
            max_motor_count[5] = (max_motor_count[5] > Motor6Count[receive_count_mocount]) ? max_motor_count[5] : Motor6Count[receive_count_mocount];
            min_motor_count[5] = (min_motor_count[5] < Motor6Count[receive_count_mocount]) ? min_motor_count[5] : Motor6Count[receive_count_mocount];
            break;
        default:
            break;
        }
        qDebug()<<"encoder"<<encoderNumber<<"is: "<<count3;
        time_x_mocount[receive_count_mocount] = receive_count_mocount;
        HighOrLow = 0;
    }

    // 完成六个电机的数据采集
    if(encoderNumber == 6)
    {
        receive_count_mocount ++;
        encoderNumber = 0;
        emit sigPlotEncoder();
    }
}

