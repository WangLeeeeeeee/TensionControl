#ifndef TENSIONREAD_H
#define TENSIONREAD_H

#include "C:/Advantech/DAQNavi/Inc/bdaqctrl.h"
#include <QThread>
#include <QDebug>

using namespace Automation::BDaq;
#define     deviceDescription  L"PCI-1716L,BID#0"
#define minize(a,b)            (((a) < (b)) ? (a) : (b))

extern QVector<double> tension_y,tension_y2,tension_y3;
extern QVector<double> tension_y4,tension_y5,tension_y6;
extern QVector<double> surpressure_elbow,surpressure_shou1,surpressure_shou2;
extern QVector<double> time_x_tension;
extern unsigned int receive_count_tension;
extern unsigned int receive_count_pressure;
extern double max_tension[];

class tensionRead: public QObject
{
    Q_OBJECT
private:
    //WaveformAiCtrl * wfAiCtrl;
    ErrorCode ret;
    // PCI-1716 initialize
    // Step 1: Create a 'WaveformAiCtrl' for buffered AI function.
    WaveformAiCtrl * wfAiCtrl = WaveformAiCtrl::Create();
public:
    tensionRead();
    static void BDAQCALL OnStoppedEvent(void * sender, BfdAiEventArgs * args, void * userParam);
    void CheckError(ErrorCode errorCode);
public slots:
    void slotReadTensionInit();
    void slotReadTension();
signals:
    void sigPlotTension();
};

#endif // TENSIONREAD_H
