#ifndef IMUREAD_H
#define IMUREAD_H

#include <QThread>
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#include <QtGui>

// extern parameter using for plot
extern QVector<double> elbow_x,elbow_y,elbow_z,shoulder_x,shoulder_y,shoulder_z;
extern QVector<double> time_x_angle;
extern unsigned int receive_count_angle;

class IMURead: public QObject
{
    Q_OBJECT
public:
    IMURead();
private:
    // Gets a LpmsSensorManager instance
    LpmsSensorManagerI* manager1 = LpmsSensorManagerFactory();
    LpmsSensorManagerI* manager2 = LpmsSensorManagerFactory();
    // Connects to LPMS-CU sensor with address A1234567
    LpmsSensorI* lpms1 = manager1->addSensor(DEVICE_LPMS_U, "A5022WAT");
    LpmsSensorI* lpms2 = manager2->addSensor(DEVICE_LPMS_U, "A5022WCL");
public slots:
    void slotIMUInit();
    void slotIMURead();
signals:
    void sigPlotIMU();
};

#endif // IMUREAD_H
