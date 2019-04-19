#include "tensionread.h"

//----------------------------------------------------------------------------------
// Receive data(including: six tension) count
//----------------------------------------------------------------------------------
unsigned int receive_count_tension = 0;
unsigned int receive_count_pressure = 0;

//----------------------------------------------------------------------------------
// Y-axis: six cable tension data(N)
// X-axis: time(every 50ms)
// Y-Range: MaxTension
//----------------------------------------------------------------------------------
unsigned int Tensionlength = 131072;
QVector<double> time_x_tension(Tensionlength),time_x_surpressure(Tensionlength);
QVector<double> tension_y(Tensionlength),tension_y2(Tensionlength),tension_y3(Tensionlength);
QVector<double> tension_y4(Tensionlength),tension_y5(Tensionlength),tension_y6(Tensionlength);
QVector<double> surpressure_elbow(Tensionlength),surpressure_shou1(Tensionlength),surpressure_shou2(Tensionlength);

double max_tension[6] = {0, 0, 0, 0, 0, 0};

//----------------------------------------------------------------------------------
// Tension calibration parameter
//----------------------------------------------------------------------------------
double TENSION_K[6] = {995.94, 989.03, 1000.8, 1001.1, 1008.1, 993.33};
double TENSION_B[6] = {203.71, 12.242, 52.057, -80.687, 54.165, -61.373};

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


tensionRead::tensionRead()
{
    qDebug()<<"tensionRead tensionRead: "<<QThread::currentThreadId();
}

void tensionRead::slotReadTensionInit()
{
    qDebug()<<"tensionRead slotReadTensionInit: "<<QThread::currentThreadId();
     ErrorCode ret = Success;

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
        //printf("Asynchronous finite acquisition is in progress.\n");
        qDebug()<<"Asynchronous finite acquisition is in progress.";
        ret = wfAiCtrl->Prepare();
        CheckError(ret);

        tension_y[0] = 0;
        tension_y2[0] = 0;
        tension_y3[0] = 0;
        tension_y4[0] = 0;
        tension_y5[0] = 0;
        tension_y6[0] = 0;
        time_x_tension[0] = 0;
    }
    while(false);
}

void tensionRead::slotReadTension()
{
//    qDebug()<<"tensionRead slotReadTension: "<<QThread::currentThreadId();
    // Step 6: The device is acquiring data.
    wfAiCtrl->Start();
    if(receive_count_tension > 0)
        emit sigPlotTension();
}

void tensionRead::CheckError(ErrorCode errorCode)
{
    if (errorCode != Success)
    {
        QString message = QObject::tr("Sorry, there are some errors occurred, Error Code: 0x") +
            QString::number(errorCode, 16).right(8).toUpper();
        qDebug()<<"Warning Information"<<message;
        //QMessageBox::information(0, "Warning Information", message);
    }
}

void tensionRead::OnStoppedEvent(void * sender, BfdAiEventArgs * args, void * userParam)
{
//    qDebug()<<"tensionRead onstoppedEvent: "<<QThread::currentThreadId();
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

    // Find the maxium of tension to set the range of the customplot
    if(receive_count_tension<50)
    {
        max_tension[0] = (max_tension[0] > tension_y[receive_count_tension])  ? max_tension[0] : tension_y[receive_count_tension];
        max_tension[1] = (max_tension[1] > tension_y2[receive_count_tension]) ? max_tension[1] : tension_y2[receive_count_tension];
        max_tension[2] = (max_tension[2] > tension_y3[receive_count_tension]) ? max_tension[2] : tension_y3[receive_count_tension];
        max_tension[3] = (max_tension[3] > tension_y4[receive_count_tension]) ? max_tension[3] : tension_y4[receive_count_tension];
        max_tension[4] = (max_tension[4] > tension_y5[receive_count_tension]) ? max_tension[4] : tension_y5[receive_count_tension];
        max_tension[5] = (max_tension[5] > tension_y6[receive_count_tension]) ? max_tension[5] : tension_y6[receive_count_tension];
    }
    else
    {
        for(int i=0; i<6; i++)
        {
            max_tension[i] = 0;
        }
        // find the latest fifty tension value max
        for(unsigned int i=receive_count_tension-50; i<receive_count_tension; i++)
        {
            max_tension[0] = (max_tension[0] > tension_y[i])  ? max_tension[0] : tension_y[i];
            max_tension[1] = (max_tension[1] > tension_y2[i]) ? max_tension[1] : tension_y2[i];
            max_tension[2] = (max_tension[2] > tension_y3[i]) ? max_tension[2] : tension_y3[i];
            max_tension[3] = (max_tension[3] > tension_y4[i]) ? max_tension[3] : tension_y4[i];
            max_tension[4] = (max_tension[4] > tension_y5[i]) ? max_tension[4] : tension_y5[i];
            max_tension[5] = (max_tension[5] > tension_y6[i]) ? max_tension[5] : tension_y6[i];
        }
    }
    // 这里不能发射信号，显示：非静态成员函数的非法调用
    //emit tenRead->sigPlotTension();
}
