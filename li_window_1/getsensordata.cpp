#include "getsensordata.h"
#include <QElapsedTimer>


//----------------------------------------------------------------------------------
// Receive data(including: six tension, two IMU, one surface pressure, four motor angle) count
//----------------------------------------------------------------------------------
unsigned int receive_count_tension = 0;
unsigned int receive_count_angle = 0;
unsigned int receive_count_pressure = 0;
unsigned int receive_count_mocount = 0;

//----------------------------------------------------------------------------------
// Y-axis: six cable tension data(N), surface pressure, IMU data(rad), Four Motor encoder
// X-axis: time(every 50ms)
// Y-Range: MaxTension
//----------------------------------------------------------------------------------
unsigned int Datalength = 131072;
QVector<double> time_x_tension(Datalength),time_x_angle(Datalength),time_x_surpressure(Datalength),time_x_mocount(Datalength);
QVector<double> tension_y(Datalength),tension_y2(Datalength),tension_y3(Datalength);
QVector<double> tension_y4(Datalength),tension_y5(Datalength),tension_y6(Datalength);
QVector<double> surpressure_elbow(Datalength),surpressure_shou1(Datalength),surpressure_shou2(Datalength);
QVector<double> elbow_x(Datalength),elbow_y(Datalength),elbow_z(Datalength);
QVector<double> shoulder_x(Datalength),shoulder_y(Datalength),shoulder_z(Datalength);
QVector<double> Motor1Count(Datalength),Motor2Count(Datalength),Motor3Count(Datalength),Motor4Count(Datalength),Motor5Count(Datalength),Motor6Count(Datalength);
double max_tension[6] = {0, 0, 0, 0, 0, 0};
double max_motor_count[6]={0, 0, 0, 0, 0, 0};
double min_motor_count[6] = {0, 0, 0, 0, 0, 0};

//----------------------------------------------------------------------------------
// Tension calibration parameter
//----------------------------------------------------------------------------------
//double TENSION_K[6] = {995.94, 989.03, 1000.8, 1001.1, 1129.9, 993.33};
//double TENSION_B[6] = {203.71, 12.242, 52.057, -80.687, -320.2, -61.373};
double TENSION_K[6] = {995.94, 989.03, 1000.8, 1001.1, 1380.4, 993.33};
double TENSION_B[6] = {203.71, 12.242, 52.057, -80.687, -880.8, -61.373};

//-----------------------------------------------------------------------------------
// Configure the following parameters before running the PCI-1716L
//-----------------------------------------------------------------------------------
const int32 sampleCount = 20;
const int32 clockRate = 20000;

int32        startChannel = 0;
const int32  channelCount = 12;
const int32  sectionLength = 1024; // for each channel, to decide the capacity of buffer in kernel.
const int32  sectionCount = 1;

#define USER_BUFFER_SIZE channelCount*sampleCount
double       Data[USER_BUFFER_SIZE];

//-----------------------------------------------------------------------------------
// Configure the following parameters before running the PCI-1784
//-----------------------------------------------------------------------------------
int32       udchannelStart = 0;
int32       udchannelCount = 4;

//----------------------------------------------------------------------------------
// Configure the IMU parameter
//----------------------------------------------------------------------------------
float ElbowAngle[3];
float ShoulderAngle[3];
ImuData Lpms1_Data;
ImuData Lpms2_Data;

LpmsSensorI* lpms1;
LpmsSensorI* lpms2;

// use for delay
bool SenFlag = 0;

unsigned int recordflag = 0;
double elbowXinit,elbowYinit,elbowZinit;
double shoulderXinit,shoulderYinit,shoulderZinit;


GetSensordata::GetSensordata(QObject *parent):QThread(parent)
{
    //udCounterCtrl = UdCounterCtrl::Create();
    //WaveformAiCtrl * wfAiCtrl = WaveformAiCtrl::Create();
    getsensorTimer = new QTimer(this);
    //QObject::connect(getsensorTimer, SIGNAL(timeout()), this, SLOT(slotChangeSenFlag()));
    //getsensorTimer->start(50);

}

//----------------------------------------------------------------------------------
//
void GetSensordata::run()
{
    qDebug()<<"GetSensordata Thread is running"<<endl;

    ErrorCode ret = Success;

    // PCI-1784 initialize
    // Step 1: Create a 'UdCounterCtrl' for UpDown Counter function.
   UdCounterCtrl* udCounterCtrl = UdCounterCtrl::Create();
   UdCounterCtrl* udCounterCtrl1 = UdCounterCtrl::Create();

    do
    {
         //Attention do not use the data collect card by two device once!!!
        // Step 2: Select a device by device number or device description and specify the access mode.
        // in this example we use ModeWrite mode so that we can fully control the device, including configuring, sampling, etc.
        DeviceInformation devInfo1(deviceDescription1);
        DeviceInformation devInfo2(deviceDescription2);
        ret = udCounterCtrl->setSelectedDevice(devInfo1);
        CheckError(ret);
        ret = udCounterCtrl1->setSelectedDevice(devInfo2);
        CheckError(ret);

        // Step 3: Set necessary parameters
        ret = udCounterCtrl->setChannelStart(udchannelStart);
        CheckError(ret);
        ret = udCounterCtrl->setChannelCount(udchannelCount);
        CheckError(ret);
        ret = udCounterCtrl1->setChannelStart(udchannelStart);
        CheckError(ret);
        ret = udCounterCtrl1->setChannelCount(udchannelCount);
        CheckError(ret);

        // Step 4: Set counting type for UpDown Counter
        Array<UdChannel>*udChannel = udCounterCtrl->getChannels();
        for(int i = udchannelStart; i < udchannelStart + udchannelCount; i++)
        {
           ret = udChannel->getItem(i).setCountingType(PulseDirection);
           CheckError(ret);
        }
        Array<UdChannel>*udChannel1 = udCounterCtrl1->getChannels();
        for(int i = udchannelStart; i < udchannelStart + udchannelCount; i++)
        {
           ret = udChannel1->getItem(i).setCountingType(PulseDirection);
           CheckError(ret);
        }

        Motor1Count[0] = 0;
        Motor2Count[0] = 0;
        Motor3Count[0] = 0;
        Motor4Count[0] = 0;
        Motor5Count[0] = 0;
        Motor6Count[0] = 0;
        time_x_mocount[0] = 0;

        surpressure_elbow[0] = 0;
        surpressure_shou1[0] = 0;
        surpressure_shou2[0] = 0;
        time_x_surpressure[0] = 0;


    }
    while(false);

    // PCI-1716 initialize
    // Step 1: Create a 'WaveformAiCtrl' for buffered AI function.
    WaveformAiCtrl * wfAiCtrl = WaveformAiCtrl::Create();

     // Step 2: Set the notification event Handler by which we can known the state of operation effectively.
    wfAiCtrl->addStoppedHandler(OnStoppedEvent, NULL);

    do
    {
        // Step 3: Select a device by device number or device description and specify the access mode.
        // in this example we use ModeWrite mode so that we can fully control the device, including configuring, sampling, etc.
        DeviceInformation devInfo(deviceDescription);
        ret = wfAiCtrl->setSelectedDevice(devInfo);
        CheckError(ret);

        // Step 4: Set necessary parameters for Buffered AI operation,
      Conversion* conversion = wfAiCtrl->getConversion();
      ret = conversion->setChannelStart(startChannel);
      CheckError(ret);
      ret = conversion->setChannelCount(channelCount);
      CheckError(ret);
      ret = conversion->setClockRate(clockRate);
      CheckError(ret);
        Record* record = wfAiCtrl->getRecord();
      ret = record->setSectionLength(sectionLength);
      CheckError(ret);
      ret = record->setSectionCount(sectionCount);//The sectionCount is nonzero value, which means 'One Buffered' mode.
      CheckError(ret);

      // Step 5: start Asynchronous Buffered AI, 'Asynchronous' means the method returns immediately
      // after the acquisition has been started. The StoppedHandler's 'StoppedEvent' method will be called
      // after the acquisition is completed.
      printf("Asynchronous finite acquisition is in progress.\n");
      ret = wfAiCtrl->Prepare();
        CheckError(ret);

      tension_y[0] = 0;
      tension_y2[0] = 0;
      tension_y3[0] = 0;
      tension_y4[0] = 0;
      tension_y5[0] = 0;
      tension_y6[0] = 0;
      time_x_tension[0] = 0;

      elbow_x[0] = 0;
      elbow_y[0] = 0;
      elbow_z[0] = 0;
      shoulder_x[0] = 0;
      shoulder_y[0] = 0;
      shoulder_z[0] = 0;
      time_x_angle[0] = 0;

   }
   while(false);


   while(1)
   {
        // Checks, if conncted
        if((lpms1->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED) && (lpms2->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED))
        {
            // Reset the Orientation
            //lpms1->resetOrientationOffset();
            //lpms2->resetOrientationOffset();

            // Read the euler angle
            lpms1->getEulerAngle(ElbowAngle);
            lpms2->getEulerAngle(ShoulderAngle);
            Lpms1_Data = lpms1->getCurrentData();
            Lpms2_Data = lpms2->getCurrentData();

            // Recors the first data
            if(recordflag < 4)
            {
                recordflag++;
                elbowXinit += ElbowAngle[0];
                elbowYinit += ElbowAngle[1];
                elbowZinit += ElbowAngle[2];
                shoulderXinit += ShoulderAngle[0];
                shoulderYinit += ShoulderAngle[1];
                shoulderZinit += ShoulderAngle[2];
            }
            else
            {
                receive_count_angle++;
                elbow_x[receive_count_angle] = ElbowAngle[0]-elbowXinit/4;
                elbow_y[receive_count_angle] = ElbowAngle[1]-elbowYinit/4;
                elbow_z[receive_count_angle] = ElbowAngle[2]-elbowZinit/4;
                shoulder_x[receive_count_angle] = ShoulderAngle[0]-shoulderXinit/4;
                shoulder_y[receive_count_angle] = ShoulderAngle[1]-shoulderYinit/4;
                shoulder_z[receive_count_angle] = ShoulderAngle[2]-shoulderZinit/4;
                time_x_angle[receive_count_angle] = receive_count_angle;
            }
        }

        // Step 6: The device is acquiring data.
        wfAiCtrl->Start();

        // Step 5: Start UpDown Counter
        ret= udCounterCtrl->setEnabled(true);

        msleep(10);// every 10ms collect once

        int32 udCount[4],udCount1[4];
        ret = udCounterCtrl->Read(4,udCount);
        ret = udCounterCtrl1->Read(4,udCount1);
        receive_count_mocount ++;
        Motor1Count[receive_count_mocount] = Motor1Count[receive_count_mocount-1] + udCount[0];
        Motor2Count[receive_count_mocount] = Motor2Count[receive_count_mocount-1] + udCount[1];
        Motor3Count[receive_count_mocount] = Motor3Count[receive_count_mocount-1] + udCount[2];
        Motor4Count[receive_count_mocount] = Motor4Count[receive_count_mocount-1] + udCount[3];
        Motor5Count[receive_count_mocount] = Motor5Count[receive_count_mocount-1] + udCount1[0];
        Motor6Count[receive_count_mocount] = Motor6Count[receive_count_mocount-1] + udCount1[1];

        // Find the maxium of motor encorder count to set the range of the customplot
        max_motor_count[0] = (max_motor_count[0] > Motor1Count[receive_count_mocount]) ? max_motor_count[0] : Motor1Count[receive_count_mocount];
        max_motor_count[1] = (max_motor_count[1] > Motor2Count[receive_count_mocount]) ? max_motor_count[1] : Motor2Count[receive_count_mocount];
        max_motor_count[2] = (max_motor_count[2] > Motor3Count[receive_count_mocount]) ? max_motor_count[2] : Motor3Count[receive_count_mocount];
        max_motor_count[3] = (max_motor_count[3] > Motor4Count[receive_count_mocount]) ? max_motor_count[3] : Motor4Count[receive_count_mocount];
        max_motor_count[4] = (max_motor_count[4] > Motor5Count[receive_count_mocount]) ? max_motor_count[4] : Motor5Count[receive_count_mocount];
        max_motor_count[5] = (max_motor_count[5] > Motor6Count[receive_count_mocount]) ? max_motor_count[5] : Motor6Count[receive_count_mocount];

        // Find the minium of motor encorder count to set the range of the customplot
        min_motor_count[0] = (min_motor_count[0] < Motor1Count[receive_count_mocount]) ? min_motor_count[0] : Motor1Count[receive_count_mocount];
        min_motor_count[1] = (min_motor_count[1] < Motor2Count[receive_count_mocount]) ? min_motor_count[1] : Motor2Count[receive_count_mocount];
        min_motor_count[2] = (min_motor_count[2] < Motor3Count[receive_count_mocount]) ? min_motor_count[2] : Motor3Count[receive_count_mocount];
        min_motor_count[3] = (min_motor_count[3] < Motor4Count[receive_count_mocount]) ? min_motor_count[3] : Motor4Count[receive_count_mocount];
        min_motor_count[4] = (min_motor_count[4] < Motor5Count[receive_count_mocount]) ? min_motor_count[4] : Motor5Count[receive_count_mocount];
        min_motor_count[5] = (min_motor_count[5] < Motor6Count[receive_count_mocount]) ? min_motor_count[5] : Motor6Count[receive_count_mocount];

        time_x_mocount[receive_count_mocount] = receive_count_mocount;
        udCounterCtrl->setEnabled(false);
//        qDebug()<<"timex_mocount is:"<<time_x_mocount[receive_count_mocount];
//        qDebug()<<"time_x_angle is:"<<time_x_angle[receive_count_angle];
//        qDebug()<<"time_x_tension is:"<<time_x_tension[receive_count_tension];
//        qDebug()<<"time_x_surpressure is:"<<time_x_surpressure[receive_count_pressure];
//        qDebug()<<"udCount0 is:"<<udCount[0];
   }
}

void GetSensordata::CheckError(ErrorCode errorCode)
{
    if (errorCode != Success)
    {
        QString message = QObject::tr("Sorry, there are some errors occurred, Error Code: 0x") +
            QString::number(errorCode, 16).right(8).toUpper();
        QMessageBox::information(0, "Warning Information", message);
    }
}

void GetSensordata::OnStoppedEvent(void * sender, BfdAiEventArgs * args, void * userParam)
{
    WaveformAiCtrl * waveformAiCtrl = NULL;
    waveformAiCtrl = (WaveformAiCtrl *)sender;
    int32 returnedCount = 0;
    int32 getDataCount = minize(USER_BUFFER_SIZE, args->Count);
    waveformAiCtrl->GetData(getDataCount, Data, 0, &returnedCount);

    int maxvol[6],minvol[6];
    for(int i=0; i<6; i++)
    {
        maxvol[i] = 0;
        minvol[i] = 0;
    }

    // Count the receive sequence
    receive_count_tension++;
    receive_count_pressure++;

    // Save the data from the Data[] to tension(including tensiony, tensiony2,...,tensiony6,surface pressure)
    for(int i = 0; i < getDataCount; i+=channelCount)
    {
        // Find the maxium and the minium of voltage
        maxvol[0] = (maxvol[0] > Data[i])  ? maxvol[0] : Data[i];
        maxvol[1] = (maxvol[1] > Data[i])  ? maxvol[1] : Data[i];
        maxvol[2] = (maxvol[2] > Data[i])  ? maxvol[2] : Data[i];
        maxvol[3] = (maxvol[3] > Data[i])  ? maxvol[3] : Data[i];
        maxvol[4] = (maxvol[4] > Data[i])  ? maxvol[4] : Data[i];
        maxvol[5] = (maxvol[5] > Data[i])  ? maxvol[5] : Data[i];

        minvol[0] = (minvol[0] < Data[i])  ? minvol[0] : Data[i];
        minvol[1] = (minvol[1] < Data[i])  ? minvol[1] : Data[i];
        minvol[2] = (minvol[2] < Data[i])  ? minvol[2] : Data[i];
        minvol[3] = (minvol[3] < Data[i])  ? minvol[3] : Data[i];
        minvol[4] = (minvol[4] < Data[i])  ? minvol[4] : Data[i];
        minvol[5] = (minvol[5] < Data[i])  ? minvol[5] : Data[i];

        tension_y[receive_count_tension]            +=      Data[i];
        tension_y2[receive_count_tension]           +=      Data[i+1];
        tension_y3[receive_count_tension]           +=      Data[i+2];
        tension_y4[receive_count_tension]           +=      Data[i+3];
        tension_y5[receive_count_tension]           +=      Data[i+4];
        tension_y6[receive_count_tension]           +=      Data[i+5];
        surpressure_elbow[receive_count_pressure]   +=      Data[i+6];
        surpressure_shou1[receive_count_pressure]   +=      Data[i+7];
        surpressure_shou2[receive_count_pressure]   +=      Data[i+8];

    }

    // Set the x-axis data
    time_x_tension[receive_count_tension] = receive_count_tension;
    time_x_surpressure[receive_count_pressure] = receive_count_pressure;

    tension_y[receive_count_tension] -= (maxvol[0]+minvol[0]);
    tension_y2[receive_count_tension] -= (maxvol[1]+minvol[1]);
    tension_y3[receive_count_tension] -= (maxvol[2]+minvol[2]);
    tension_y4[receive_count_tension] -= (maxvol[3]+minvol[3]);
    tension_y5[receive_count_tension] -= (maxvol[4]+minvol[4]);
    tension_y6[receive_count_tension] -= (maxvol[5]+minvol[5]);
    getDataCount = getDataCount - 2*channelCount;

    // Average filter
    tension_y[receive_count_tension]            /=      (getDataCount / channelCount);
    tension_y2[receive_count_tension]           /=      (getDataCount / channelCount);
    tension_y3[receive_count_tension]           /=      (getDataCount / channelCount);
    tension_y4[receive_count_tension]           /=      (getDataCount / channelCount);
    tension_y5[receive_count_tension]           /=      (getDataCount / channelCount);
    tension_y6[receive_count_tension]           /=      (getDataCount / channelCount);
    surpressure_elbow[receive_count_pressure]   /=      (getDataCount / channelCount);
    surpressure_shou1[receive_count_pressure]   /=      (getDataCount / channelCount);
    surpressure_shou2[receive_count_pressure]   /=      (getDataCount / channelCount);


    if((surpressure_elbow[receive_count_pressure]>5)||(surpressure_elbow[receive_count_pressure]<-5))
        surpressure_elbow[receive_count_pressure] = surpressure_elbow[receive_count_pressure-1];
    if((surpressure_shou1[receive_count_pressure]>5)||(surpressure_shou1[receive_count_pressure]<-5))
        surpressure_shou1[receive_count_pressure] = surpressure_shou1[receive_count_pressure-1];
    if((surpressure_shou2[receive_count_pressure]>5)||(surpressure_shou2[receive_count_pressure]<-5))
        surpressure_shou2[receive_count_pressure] = surpressure_shou2[receive_count_pressure-1];

    // Change the voltage(v) to Tension(g)
    tension_y[receive_count_tension]  =   TENSION_K[0] * tension_y[receive_count_tension] + TENSION_B[0];
    tension_y2[receive_count_tension] =   TENSION_K[1] * tension_y2[receive_count_tension] + TENSION_B[1];
    tension_y3[receive_count_tension] =   TENSION_K[2] * tension_y3[receive_count_tension] + TENSION_B[2];
    tension_y4[receive_count_tension] =   TENSION_K[3] * tension_y4[receive_count_tension] + TENSION_B[3];
    tension_y5[receive_count_tension] =   TENSION_K[4] * tension_y5[receive_count_tension] + TENSION_B[4];
    tension_y6[receive_count_tension] =   TENSION_K[5] * tension_y6[receive_count_tension] + TENSION_B[5];

    // judge if the voltage whether is in the normal range.
    if((tension_y[receive_count_tension]>5000)||(tension_y[receive_count_tension]<-5000))
        tension_y[receive_count_tension] = tension_y[receive_count_tension-1];
    if((tension_y2[receive_count_tension]>5000)||(tension_y2[receive_count_tension]<-5000))
        tension_y2[receive_count_tension] = tension_y2[receive_count_tension-1];
    if((tension_y3[receive_count_tension]>5000)||(tension_y3[receive_count_tension]<-5000))
        tension_y3[receive_count_tension] = tension_y3[receive_count_tension-1];
    if((tension_y4[receive_count_tension]>5000)||(tension_y4[receive_count_tension]<-5000))
        tension_y4[receive_count_tension] = tension_y4[receive_count_tension-1];
    if((tension_y5[receive_count_tension]>5000)||(tension_y5[receive_count_tension]<-5000))
        tension_y5[receive_count_tension] = tension_y5[receive_count_tension-1];
    if((tension_y6[receive_count_tension]>5000)||(tension_y6[receive_count_tension]<-5000))
        tension_y6[receive_count_tension] = tension_y6[receive_count_tension-1];

//    qDebug()<<"tensiony:"<<tension_y[receive_count_tension];
//    if(tension_y[receive_count_tension]>50)
//        qDebug()<<"unnormal data";
//    qDebug()<<"tensiony2:"<<tension_y2[receive_count_tension];
//    qDebug()<<"tensiony3:"<<tension_y3[receive_count_tension];
//    qDebug()<<"tensiony4:"<<tension_y4[receive_count_tension];
//    qDebug()<<"tensiony5:"<<tension_y5[receive_count_tension];
//    qDebug()<<"tensiony6:"<<tension_y6[receive_count_tension];

    // Find the maxium of tension to set the range of the customplot
    max_tension[0] = (max_tension[0] > tension_y[receive_count_tension])  ? max_tension[0] : tension_y[receive_count_tension];
    max_tension[1] = (max_tension[1] > tension_y2[receive_count_tension]) ? max_tension[1] : tension_y2[receive_count_tension];
    max_tension[2] = (max_tension[2] > tension_y3[receive_count_tension]) ? max_tension[2] : tension_y3[receive_count_tension];
    max_tension[3] = (max_tension[3] > tension_y4[receive_count_tension]) ? max_tension[3] : tension_y4[receive_count_tension];
    max_tension[4] = (max_tension[4] > tension_y5[receive_count_tension]) ? max_tension[4] : tension_y5[receive_count_tension];
    max_tension[5] = (max_tension[5] > tension_y6[receive_count_tension]) ? max_tension[5] : tension_y6[receive_count_tension];

//    for(int i=0; i<6; i++)
//    {
//        if(max_tension[i] > 1000)
//            qDebug()<<"wow, it's out of the normal range:"<<receive_count_tension;
//    }

}

void GetSensordata::slotChangeSenFlag()
{
}

void GetSensordata::delay(int mseconds)
{
    QTime dieTime=QTime::currentTime().addMSecs(mseconds);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

