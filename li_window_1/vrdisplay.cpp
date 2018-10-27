#include "vrdisplay.h"



VRDisplay::VRDisplay(QObject *parent):QThread(parent)
{

}


void VRDisplay::run()
{

}

void VRDisplay::slotVRSerialOpen()
{
    vrSerial.setPortName("COM3");
    vrSerial.setBaudRate(QSerialPort::Baud9600);
    vrSerial.setDataBits(QSerialPort::Data8);
    vrSerial.setStopBits(QSerialPort::OneStop);
    vrSerial.setFlowControl(QSerialPort::NoFlowControl);
    vrSerial.setParity(QSerialPort::NoParity);
    vrSerial.close();

    if(vrSerial.open(QIODevice::ReadWrite))
    {
        connect(&vrSerial,SIGNAL(readyRead()),this,SLOT(slotReadVRcom()));
        QMessageBox::information(0,tr("open sucessful"),tr("sucessful open com"),QMessageBox::Ok);
    }
    else
    {
        QMessageBox::critical(0,tr("open failed"),tr("cannot open com"),QMessageBox::Ok);
        return;
    }

    vrTimer = new QTimer(this);
    connect(vrTimer, SIGNAL(timeout()), this, SLOT(slotSendVRdata()));
    int send_interval = TIME_SEND_INTERVAL;
    vrTimer->start(send_interval);
}

void VRDisplay::slotSendVRdata()
{
    QString head = "f0";
    QString end = "f1";
    int sendValue;

    // 发送帧头
    for(int i=0; i<8; i++)
    {
        vrSerial.write(QString2Hex(head));
    }

    sendValue = shoulder_x[receive_count_angle];//shoulder waizhan
    SendData(sendValue);
    qDebug()<<"shoulder_z:"<<sendValue<<endl;

    sendValue = shoulder_z[receive_count_angle];//shoulder neixuan
    SendData(sendValue);
    qDebug()<<"shoulder_x:"<<sendValue<<endl;

    sendValue = shoulder_y[receive_count_angle];//shoulder qianqu
    SendData(sendValue);
    qDebug()<<"shoulder_y:"<<sendValue<<endl;

    sendValue = elbow_y[receive_count_angle];//elbow qianqu
    SendData(sendValue);
    qDebug()<<"elbow_y:"<<sendValue<<endl;

    //发送帧尾
    vrSerial.write(QString2Hex(end));


}

void VRDisplay::SendData(int sendvalue)
{
    QString positive = "fe";
    QString negative = "ff";
    QString endofone = "a0";
    QString SendData;

    if(sendvalue < 0)
    {
        vrSerial.write(QString2Hex(negative));
        sendvalue = -sendvalue;
    }
    else
    {
        vrSerial.write(QString2Hex(positive));
    }

    SendData = QString::number(sendvalue);
    vrSerial.write(SendData.toLatin1());
    vrSerial.write(QString2Hex(endofone));
}


void VRDisplay::slotReadVRcom()
{
    // do something here
    QString SendData1;
    SendData1 = "isahioashd";
    vrSerial.write(SendData1.toLatin1());
}

char VRDisplay::ConvertHexChar(char ch)
{
    if((ch >= '0') && (ch <= '9'))
        return ch-0x30;
    else if((ch >= 'A') && (ch <= 'F'))
        return ch-'A'+10;
    else if((ch >= 'a') && (ch <= 'f'))
        return ch-'a'+10;
    else return ch-ch;
}

QByteArray VRDisplay::QString2Hex(QString str)
{
        QByteArray senddata;
        int hexdata,lowhexdata;
        int hexdatalen = 0;
        int len = str.length();
        senddata.resize(len/2);
        char lstr,hstr;
        for(int i=0; i<len; )
        {
            hstr=str[i].toLatin1();
            if(hstr == ' ')
            {
                i++;
                continue;
            }
            i++;
            if(i >= len)
                break;
            lstr = str[i].toLatin1();
            hexdata = ConvertHexChar(hstr);
            lowhexdata = ConvertHexChar(lstr);
            if((hexdata == 16) || (lowhexdata == 16))
                break;
            else
                hexdata = hexdata*16+lowhexdata;
            i++;
            senddata[hexdatalen] = (char)hexdata;
            hexdatalen++;
        }
        senddata.resize(hexdatalen);
        return senddata;
}
