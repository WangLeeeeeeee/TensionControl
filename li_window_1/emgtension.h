#ifndef EMGTENSION_H
#define EMGTENSION_H

#include <QThread>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtCore/qmath.h>
#include "imuread.h"

extern QVector<double> EmgDataSave,AngleElbow_emg;
extern unsigned int emgsaveLen;
extern QVector<float> fiteffRecord;

class emgTension: public QObject
{
    Q_OBJECT
public:
    emgTension();
    void emgSendData(const char* data);
    QTcpServer* server;
    QTcpSocket* socket;
public slots:
    void slotEmgserverInit();
    void slotRecordWithCtrl();
private slots:
    void server_New_Connect();
    void readEmgData();
    void socket_Disconnected();
signals:
    void sigEmgThetaFit(double* fiteff, double* bufferX, double* bufferY, unsigned int dimension, int sizenum);
    void sigEmgTensionctrl(uint Seraddress, int Startaddress, qint32 Data, bool writeLength);
    void sigStopEmgCtrl();
private:
    int PrintPara(double* Para, int SizeSrc);
    int ParalimitRow(double* Para, int SizeSrc, int Row);
    int Paralimit(double* Para, int SizeSrc);
    int ParaPreDealA(double* Para, int SizeSrc, int Size);
    int ParaDealA(double* Para, int SizeSrc);
    int ParaPreDealB(double* Para, int SizeSrc, int OffSet);
    int ParaDealB(double* Para, int SizeSrc);
    int ParaDeal(double* Para, int SizeSrc);
    int GetParaBuffer(double* Para, const double* X, const double* Y, int Amount, int SizeSrc);
    int Cal(const double* BufferX, const double* BufferY, int Amount, int SizeSrc, double* ParaResK);
};

#endif // EMGTENSION_H
