#include "imuread.h"
#include <QThread>

//----------------------------------------------------------------------------------
// Receive data(including: six tension, two IMU, one surface pressure, four motor angle) count
//----------------------------------------------------------------------------------
unsigned int receive_count_angle = 0;

//----------------------------------------------------------------------------------
// Y-axis: six cable tension data(N), surface pressure, IMU data(rad), Four Motor encoder
// X-axis: time(every 50ms)
// Y-Range: MaxTension
//----------------------------------------------------------------------------------
unsigned int Datalength = 131072;
QVector<double> time_x_angle(Datalength);
QVector<double> elbow_x(Datalength),elbow_y(Datalength),elbow_z(Datalength);
QVector<double> shoulder_x(Datalength),shoulder_y(Datalength),shoulder_z(Datalength);

//----------------------------------------------------------------------------------
// Configure the IMU parameter
//----------------------------------------------------------------------------------
float ElbowAngle[3];
float ShoulderAngle[3];
ImuData Lpms1_Data;
ImuData Lpms2_Data;

//----------------------------------------------------------------------------------
// Record the fourth data and set it to zero
//----------------------------------------------------------------------------------
unsigned int recordflag = 0;
double elbowXinit,elbowYinit,elbowZinit;
double shoulderXinit,shoulderYinit,shoulderZinit;


IMURead::IMURead()
{

}

void IMURead::slotIMUInit()
{
    qDebug()<<"IMURead slotIMUInit"<<QThread::currentThreadId();
    do{
        elbow_x[0] = 0;
        elbow_y[0] = 0;
        elbow_z[0] = 0;
        shoulder_x[0] = 0;
        shoulder_y[0] = 0;
        shoulder_z[0] = 0;
        time_x_angle[0] = 0;
    }while(false);
}

void IMURead::slotIMURead()
{
    //qDebug()<<"IMURead slotIMURead"<<QThread::currentThreadId();
    if((lpms1->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED) && (lpms2->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED))
    {
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
        emit sigPlotIMU();
    }
}
