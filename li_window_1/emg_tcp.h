#ifndef EMG_TCP_H
#define EMG_TCP_H
// https://blog.csdn.net/u014695839/article/details/70041771
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include "getsensordata.h"
//#include "tensioncontrol.h"
//class TensionControl;
class modbus;


#define PORT 8090

extern QVector<double> EmgDataSave,AngleElbow_emg;
extern unsigned int emgsaveLen;

class EMG_server:public QObject
{
    Q_OBJECT

public:
    explicit EMG_server(QObject* parent=0);
    ~EMG_server();
    void Send_Data(const char* data);
    QTcpServer* server;
    QTcpSocket* socket;
    QTime *current_time;

private slots:
    void server_New_Connect();
    void socket_Disconnected();
    void Read_Data();


    void slotEmgStart();
    void slotEmgTrigger();

    void slotRecord();
    void slotposBack(); // when we click "trigger" buttton we should let the elbow back to the start position


private:
    QTimer *recordAngleTimer;
    QTimer *posbackTimer; // when we click "trigger" buttton we should let the elbow back to the start position
    //TensionControl *tenctrl;
    modbus *mbctrl;
    void polyfit(int n,double x[],double y[],int poly_n,double p[]);
    void gauss_solve(int n,double A[],double x[],double b[]);

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
    void sigEmgThetaFit(double* fiteff, double* bufferX, double* bufferY, unsigned int dimension, int sizenum);


};
#endif // EMG_TCP_H
