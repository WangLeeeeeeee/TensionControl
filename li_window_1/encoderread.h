#ifndef ENCODERREAD_H
#define ENCODERREAD_H
#include "qthread.h"
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QDebug>

extern QVector<double> Motor1Count,Motor2Count,Motor3Count,Motor4Count,Motor5Count,Motor6Count;
extern QVector<double> time_x_mocount;
extern double max_motor_count[];
extern double min_motor_count[];
extern unsigned int receive_count_mocount;

#define minize(a,b)            (((a) < (b)) ? (a) : (b))

class encoderRead: public QObject
{
    Q_OBJECT
public:
    encoderRead();
private:
    bool HighOrLow = 0;
    qint64 count1,count2,count3;
    unsigned int encoderNumber = 0;
    QTimer *encoderTimer;
public slots:
    void slotReadEncoder(); // 接受到mainwindow 的startMearsure
    void slotDataToEncoder(QString data); // 接受到modbus 的readData信号
signals:
    void sigReadEncoder(uint Seraddress, int Startaddress, uint number);
    void sigPlotEncoder();
};

#endif // ENCODERREAD_H
