#include "getsensordata.h"

//----------------------------------------------------------------------------------
// Receive data(including: six tension, two IMU, one surface pressure, six motor angle) count
//----------------------------------------------------------------------------------
unsigned int receive_count_tension = 0;
unsigned int receive_count_angle = 0;
unsigned int receive_count_pressure = 0;

//----------------------------------------------------------------------------------
// Y-axis: six motor angle
// X-axis: time(every 50ms)
// Y-Range: MaxMotorAngle
//----------------------------------------------------------------------------------
QVector<double> motoangle_1,motoangle_2,motoangle_3;
QVector<double> motoangle_4,motoangle_5,motoangle_6;
double max_motor_angle[6]={0,0,0,0,0,0};

//----------------------------------------------------------------------------------
// Y-axis: six cable tension data(N), surface pressure, IMU data(rad)
// X-axis: time(every 50ms)
// Y-Range: MaxTension
//----------------------------------------------------------------------------------
QVector<double> time_x_tension(131072),time_x_angle(131072),time_x_surpressure(131072);
QVector<double> tension_y(131072),tension_y2(131072),tension_y3(131072);
QVector<double> tension_y4(131072),tension_y5(131072),tension_y6(131072);
QVector<double> surpressure_elbow(131072),surpressure_shou1(131072),surpressure_shou2(131072);
QVector<double> elbow_x(131072),elbow_y(131072),elbow_z(131072);
QVector<double> shoulder_x(131072),shoulder_y(131072),shoulder_z(131072);
QVector<double> elbow_x_acc(131072),elbow_y_acc(131072),elbow_z_acc(131072);
QVector<double> shoulder_x_acc(131072),shoulder_y_acc(131072),shoulder_z_acc(131072);
double max_tension[6] = {0, 0, 0, 0, 0, 0};

//----------------------------------------------------------------------------------
// Tension calibration parameter
//----------------------------------------------------------------------------------
//double TENSION_K[6] = {995.94, 989.03, 1000.8, 1001.1, 1129.9, 993.33};
//double TENSION_B[6] = {203.71, 12.242, 52.057, -80.687, -320.2, -61.373};
double TENSION_K[6] = {995.94, 989.03, 1000.8, 1001.1, 1413.4, 993.33};
double TENSION_B[6] = {203.71, 12.242, 52.057, -80.687, -1351.8, -61.373};

//-----------------------------------------------------------------------------------
// Configure the following parameters before running the PCI-1716L
//-----------------------------------------------------------------------------------
const int32 sampleCount = 20;
const int32 clockRate = 1000;

int32        startChannel = 0;
const int32  channelCount = 9;
const int32  sectionLength = 1024; // for each channel, to decide the capacity of buffer in kernel.
const int32  sectionCount = 1;

#define USER_BUFFER_SIZE channelCount*sampleCount
double       Data[USER_BUFFER_SIZE];

//----------------------------------------------------------------------------------
// Configure the IMU parameter
//----------------------------------------------------------------------------------
float ElbowAngle[3];
float ShoulderAngle[3];
ImuData Lpms1_Data;
ImuData Lpms2_Data;

LpmsSensorI* lpms1;
LpmsSensorI* lpms2;

GetSensordata::GetSensordata(QObject *parent):QThread(parent)
{   
}

//----------------------------------------------------------------------------------
//
void GetSensordata::run()
{
    qDebug()<<"GetSensordata Thread is running"<<endl;

    ErrorCode ret = Success;

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
      //ret = wfAiCtrl->LoadProfile(profilePath);//Loads a profile to initialize the device.
      //CheckError(ret);

        // Step 4: Set necessary parameters for Buffered AI operation,
      Conversion* conversion = wfAiCtrl->getConversion();
      ret = conversion->setChannelStart(startChannel);
      CheckError(ret);
      ret = conversion->setChannelCount(channelCount);
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

            // Read the linear acceleration
            elbow_x_acc[receive_count_angle] = Lpms1_Data.linAcc[0];
            elbow_y_acc[receive_count_angle] = Lpms1_Data.linAcc[1];
            elbow_z_acc[receive_count_angle] = Lpms1_Data.linAcc[2];
            shoulder_x_acc[receive_count_angle] = Lpms2_Data.linAcc[0];
            shoulder_y_acc[receive_count_angle] = Lpms2_Data.linAcc[1];
            shoulder_z_acc[receive_count_angle] = Lpms2_Data.linAcc[2];

            // Save angle data
            elbow_x[receive_count_angle] = ElbowAngle[0];
            elbow_y[receive_count_angle] = ElbowAngle[1];
            elbow_z[receive_count_angle] = ElbowAngle[2];
            shoulder_x[receive_count_angle] = ShoulderAngle[0];
            shoulder_y[receive_count_angle] = ShoulderAngle[1];
            shoulder_z[receive_count_angle] = ShoulderAngle[2];


            // Shows data
            /*
            qDebug()<< "elbow_x: "<<elbow_x[receive_count_angle]<<endl;
            qDebug()<< "elbow_y: "<<elbow_y[receive_count_angle]<<endl;
            qDebug()<< "elbow_z: "<<elbow_z[receive_count_angle]<<endl;
            qDebug()<< "shoulder_x: "<<shoulder_x[receive_count_angle]<<endl;
            qDebug()<< "shoulder_y: "<<shoulder_y[receive_count_angle]<<endl;
            qDebug()<< "shoulder_z: "<<shoulder_z[receive_count_angle]<<endl;
            qDebug()<< "elbow_x_acc: "<<elbow_x_acc[receive_count_angle]<<endl;
            qDebug()<< "elbow_y_acc: "<<elbow_y_acc[receive_count_angle]<<endl;
            qDebug()<< "elbow_z_acc: "<<elbow_z_acc[receive_count_angle]<<endl;
            qDebug()<< "shoulder_x_acc: "<<shoulder_x_acc[receive_count_angle]<<endl;
            qDebug()<< "shoulder_x_acc: "<<shoulder_x_acc[receive_count_angle]<<endl;
            qDebug()<< "shoulder_x_acc: "<<shoulder_x_acc[receive_count_angle]<<endl;
            qDebug()<<"receive_count_angle"<<receive_count_angle<<endl;
            */

            receive_count_angle++;
            time_x_angle[receive_count_angle] = receive_count_angle * 0.05;
        }

        // Step 6: The device is acquiring data.
        wfAiCtrl->Start();
        Sleep(50);// every 50ms collect once
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
    int32 getDataCount = min(USER_BUFFER_SIZE, args->Count);
    waveformAiCtrl->GetData(getDataCount, Data, 0, &returnedCount);

    // Save the data from the Data[] to tension(including tensiony, tensiony2,...,tensiony6,surface pressure)
    for(int i = 0; i < getDataCount; i+=9)
    {

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
    time_x_tension[receive_count_tension] = 0.05 * receive_count_tension;
    time_x_surpressure[receive_count_pressure] = 0.05 * receive_count_pressure;


    // Average filter
    tension_y[receive_count_tension]            /=      (getDataCount / 9);
    tension_y2[receive_count_tension]           /=      (getDataCount / 9);
    tension_y3[receive_count_tension]           /=      (getDataCount / 9);
    tension_y4[receive_count_tension]           /=      (getDataCount / 9);
    tension_y5[receive_count_tension]           /=      (getDataCount / 9);
    tension_y6[receive_count_tension]           /=      (getDataCount / 9);
    surpressure_elbow[receive_count_pressure]   /=      (getDataCount / 9);
    surpressure_shou1[receive_count_pressure]   /=      (getDataCount / 9);
    surpressure_shou2[receive_count_pressure]   /=      (getDataCount / 9);

    /*
    qDebug()<<"Tension1:"<<tension_y[receive_count_tension];
    qDebug()<<"Tension2:"<<tension_y2[receive_count_tension];
    qDebug()<<"Tension3:"<<tension_y3[receive_count_tension];
    qDebug()<<"Tension4:"<<tension_y4[receive_count_tension];
    qDebug()<<"Tension5:"<<tension_y5[receive_count_tension];
    qDebug()<<"Tension6:"<<tension_y6[receive_count_tension];
    qDebug()<<"elbow Surface pressure:"<<surpressure_elbow[receive_count_pressure];
    qDebug()<<"shoulder Surface pressure:"<<surpressure_shou1[receive_count_pressure];
    qDebug()<<"shoulder Surface pressure:"<<surpressure_shou2[receive_count_pressure];
    qDebug()<<"receive_count_tension:"<<receive_count_tension;
    qDebug()<<"receive_count_pressure"<<receive_count_pressure;
    */

    // Change the voltage(v) to Tension(g)
    tension_y[receive_count_tension]  =   TENSION_K[0] * tension_y[receive_count_tension] + TENSION_B[0];
    tension_y2[receive_count_tension] =   TENSION_K[1] * tension_y2[receive_count_tension] + TENSION_B[1];
    tension_y3[receive_count_tension] =   TENSION_K[2] * tension_y3[receive_count_tension] + TENSION_B[2];
    tension_y4[receive_count_tension] =   TENSION_K[3] * tension_y4[receive_count_tension] + TENSION_B[3];
    tension_y5[receive_count_tension] =   TENSION_K[4] * tension_y5[receive_count_tension] + TENSION_B[4];
    tension_y6[receive_count_tension] =   TENSION_K[5] * tension_y6[receive_count_tension] + TENSION_B[5];

    // Find the maxium of tension to set the range of the customplot
    max_tension[0] = (max_tension[0] > tension_y[receive_count_tension])  ? max_tension[0] : tension_y[receive_count_tension];
    max_tension[1] = (max_tension[1] > tension_y2[receive_count_tension]) ? max_tension[1] : tension_y2[receive_count_tension];
    max_tension[2] = (max_tension[2] > tension_y3[receive_count_tension]) ? max_tension[2] : tension_y3[receive_count_tension];
    max_tension[3] = (max_tension[3] > tension_y4[receive_count_tension]) ? max_tension[3] : tension_y4[receive_count_tension];
    max_tension[4] = (max_tension[4] > tension_y5[receive_count_tension]) ? max_tension[4] : tension_y5[receive_count_tension];
    max_tension[5] = (max_tension[5] > tension_y6[receive_count_tension]) ? max_tension[5] : tension_y6[receive_count_tension];

    // Count the receive sequence
    receive_count_tension++;
    receive_count_pressure++;
}


