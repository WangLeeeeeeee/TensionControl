#ifndef EMG_TCP_H
#define EMG_TCP_H
// https://blog.csdn.net/u014695839/article/details/70041771
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include "getsensordata.h"
#include "tensioncontrol.h"


#define PORT 8090

class EMG_server:public QObject
{
    Q_OBJECT

public:
    explicit EMG_server(QObject* parent=0);
    ~EMG_server();
    void Send_Data(const char* data);
    QTcpServer* server;
    QTcpSocket* socket;


private slots:
    void server_New_Connect();
    void socket_Disconnected();
    void Read_Data();

    void slotEmgStart();
    void slotEmgTrigger();

    void slotRecord();

private:
    QTimer *recordAngleTimer;
    TensionControl *tenctrl;
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
signals:
    void sigEmgTensionctrl(unsigned int* data);


};
#endif // EMG_TCP_H
