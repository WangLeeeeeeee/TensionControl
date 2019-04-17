#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QtGui>
#include <ActiveQt/QAxObject>
#include <qdebug.h>
#include "vrdisplay.h"
#include "modbus.h"
#include "motorcontrol.h"
#include <QLayout>

QVector<float> fiteffRecord;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    startInit();
    setWindowTitle(QString("Upper Limb Console"));
    ui->statusBar->showMessage(QString("welcome to use upper limb console"));

    // Instantiation object
    getsensordata = new GetSensordata;
    vrdisplay = new VRDisplay;
    emg_server = new EMG_server;
    mb = new modbus;
    motorctrl = new motorcontrol;

    // 创建子线程
    threadModbus = new QThread(this);
    mb->moveToThread(threadModbus);

    // 3D link module visualization
    QSize screenSize = graph->screen()->size();
    container->setMinimumSize(QSize(screenSize.width()/4, screenSize.height()/4));
    container->setMaximumSize(QSize(screenSize.width(), screenSize.height()));
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container->setFocusPolicy(Qt::StrongFocus);
    QHBoxLayout *hLayout = new QHBoxLayout(ui->linkWidget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(container, 1);
    hLayout->addLayout(vLayout);
    vLayout->setAlignment(Qt::AlignTop);
    ui->linkWidget->setWindowTitle(QStringLiteral("Link model"));
    ui->linkWidget->show();

    // 通过信号与槽调用子线程
    connect(threadModbus, &QThread::started, mb, &modbus::slotMdSerialInit, Qt::DirectConnection);
    connect(mb, SIGNAL(sigModMessage(QString,QString)), this, SLOT(modMessage(QString,QString)));
    connect(this, SIGNAL(sigModbusClose()), mb, SLOT(slotMdSerialClose()));
    connect(motorctrl, SIGNAL(sigMotorControl(uint,int,qint32,bool)), mb, SLOT(writeModbus(uint,int,qint32,bool)));

    // connect the maindow signal with vr com
    connect(this,SIGNAL(sigVRSerialOpen()), vrdisplay, SLOT(slotVRSerialOpen()));

    // connect the emg_tcp signals with mainwindows
    connect(this, SIGNAL(sigEmgStart()), emg_server, SLOT(slotEmgStart()));
    connect(this, SIGNAL(sigEmgTrigger()), emg_server, SLOT(slotEmgTrigger()));
    connect(emg_server, SIGNAL(sigEmgThetaFit(double*,double*,double*,uint,int)), this, SLOT(slotEmgThetaFit(double*,double*,double*,uint,int)));

    // connect the getsensordata signals with mainwindow
    connect(getsensordata, SIGNAL(sigPlotTrigger()), this, SLOT(plot()));

    // connect the mainwindow signals with motorcontrol
    connect(this, SIGNAL(sigDisableMotor()), motorctrl, SLOT(slotDisableMotor()));
    connect(this, SIGNAL(sigMdBeforeTigh(uint*)), motorctrl, SLOT(slotMdBeforeTigh(uint*)));
    connect(this, SIGNAL(sigMdSerialCtrl(uint,int*)), motorctrl, SLOT(slotMdSerialCtrl(uint,int*)));
    connect(this, SIGNAL(sigMdTeachStart()), motorctrl, SLOT(slotMdTeachStart()));
    connect(this, SIGNAL(sigMdTeachStop()), motorctrl, SLOT(slotMdTeachStop()));
    connect(this, SIGNAL(sigMdReplayTeach()), motorctrl, SLOT(slotMdReplayTeach()));

    qDebug()<<"mainwindow the trhead is"<<QThread::currentThreadId();

}

MainWindow::~MainWindow()
{
    if(!threadModbus->isRunning())
        return;
    threadModbus->quit();
    threadModbus->wait();
    getsensordata->terminate();
    getsensordata->wait();
    delete getsensordata;
    getsensordata = 0;
    delete ui;
}

// Configure the customplot
void MainWindow::Plot_Init()
{
    //
    ui->qCustomPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot6->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot7->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot8->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot9->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot10->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot11->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->qCustomPlot12->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->Motor1Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->Motor2Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->Motor3Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->Motor4Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->Motor5Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->Motor6Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                          QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->elbpresPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                     QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->shopresPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                     QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->emgFitPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                     QCP::iSelectLegend | QCP::iSelectPlottables);

    QFont fontTick("Times New Roman", 10, 75);
    QFont fontLabel("Times New Roman", 10, 50);

    ui->qCustomPlot->addGraph();
    ui->qCustomPlot->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot->yAxis->setLabelFont(fontLabel);
    //ui->qCustomPlot->xAxis->setLabel("Time");
    ui->qCustomPlot->yAxis->setLabel("Tenison1");

    ui->qCustomPlot2->addGraph();
    ui->qCustomPlot2->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot2->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot2->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot2->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot2->yAxis->setLabelFont(fontLabel);
    //ui->qCustomPlot2->xAxis->setLabel("Time");
    ui->qCustomPlot2->yAxis->setLabel("Tenison2");

    ui->qCustomPlot3->addGraph();
    ui->qCustomPlot3->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot3->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot3->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot3->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot3->yAxis->setLabelFont(fontLabel);
    //ui->qCustomPlot3->xAxis->setLabel("Time");
    ui->qCustomPlot3->yAxis->setLabel("Tenison3");

    ui->qCustomPlot4->addGraph();
    ui->qCustomPlot4->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot4->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot4->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot4->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot4->yAxis->setLabelFont(fontLabel);
    //ui->qCustomPlot4->xAxis->setLabel("Time");
    ui->qCustomPlot4->yAxis->setLabel("Tenison4");

    ui->qCustomPlot5->addGraph();
    ui->qCustomPlot5->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot5->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot5->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot5->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot5->yAxis->setLabelFont(fontLabel);
    //ui->qCustomPlot5->xAxis->setLabel("Time");
    ui->qCustomPlot5->yAxis->setLabel("Tenison5");

    ui->qCustomPlot6->addGraph();
    ui->qCustomPlot6->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot6->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot6->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot6->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot6->yAxis->setLabelFont(fontLabel);
    //ui->qCustomPlot6->xAxis->setLabel("Time");
    ui->qCustomPlot6->yAxis->setLabel("Tenison6");

    ui->qCustomPlot7->addGraph();
    ui->qCustomPlot7->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot7->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot7->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot7->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot7->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot7->xAxis->setLabel("Time");
    ui->qCustomPlot7->yAxis->setLabel("ElbowX");

    ui->qCustomPlot8->addGraph();
    ui->qCustomPlot8->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot8->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot8->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot8->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot8->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot8->xAxis->setLabel("Time");
    ui->qCustomPlot8->yAxis->setLabel("ElbowY");

    ui->qCustomPlot9->addGraph();
    ui->qCustomPlot9->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot9->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot9->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot9->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot9->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot9->xAxis->setLabel("Time");
    ui->qCustomPlot9->yAxis->setLabel("ElbowZ");

    ui->qCustomPlot10->addGraph();
    ui->qCustomPlot10->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot10->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot10->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot10->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot10->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot10->xAxis->setLabel("Time");
    ui->qCustomPlot10->yAxis->setLabel("ShoulderX");

    ui->qCustomPlot11->addGraph();
    ui->qCustomPlot11->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot11->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot11->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot11->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot11->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot11->xAxis->setLabel("Time");
    ui->qCustomPlot11->yAxis->setLabel("ShoulderY");

    ui->qCustomPlot12->addGraph();
    ui->qCustomPlot12->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot12->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot12->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot12->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot12->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot12->xAxis->setLabel("Time");
    ui->qCustomPlot12->yAxis->setLabel("ShoulderZ");

    ui->Motor1Plot->addGraph();
    ui->Motor1Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor1Plot->xAxis->setTickLabelFont(fontTick);
    ui->Motor1Plot->yAxis->setTickLabelFont(fontTick);
    ui->Motor1Plot->xAxis->setLabelFont(fontLabel);
    ui->Motor1Plot->yAxis->setLabelFont(fontLabel);
    ui->Motor1Plot->xAxis->setLabel("Time");
    ui->Motor1Plot->yAxis->setLabel("angle");

    ui->Motor2Plot->addGraph();
    ui->Motor2Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor2Plot->xAxis->setTickLabelFont(fontTick);
    ui->Motor2Plot->yAxis->setTickLabelFont(fontTick);
    ui->Motor2Plot->xAxis->setLabelFont(fontLabel);
    ui->Motor2Plot->yAxis->setLabelFont(fontLabel);
    ui->Motor2Plot->xAxis->setLabel("Time");
    ui->Motor2Plot->yAxis->setLabel("angle");

    ui->Motor3Plot->addGraph();
    ui->Motor3Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor3Plot->xAxis->setTickLabelFont(fontTick);
    ui->Motor3Plot->yAxis->setTickLabelFont(fontTick);
    ui->Motor3Plot->xAxis->setLabelFont(fontLabel);
    ui->Motor3Plot->yAxis->setLabelFont(fontLabel);
    ui->Motor3Plot->xAxis->setLabel("Time");
    ui->Motor3Plot->yAxis->setLabel("angle");

    ui->Motor4Plot->addGraph();
    ui->Motor4Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor4Plot->xAxis->setTickLabelFont(fontTick);
    ui->Motor4Plot->yAxis->setTickLabelFont(fontTick);
    ui->Motor4Plot->xAxis->setLabelFont(fontLabel);
    ui->Motor4Plot->yAxis->setLabelFont(fontLabel);
    ui->Motor4Plot->xAxis->setLabel("Time");
    ui->Motor4Plot->yAxis->setLabel("angle");

    ui->Motor5Plot->addGraph();
    ui->Motor5Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor5Plot->xAxis->setTickLabelFont(fontTick);
    ui->Motor5Plot->yAxis->setTickLabelFont(fontTick);
    ui->Motor5Plot->xAxis->setLabelFont(fontLabel);
    ui->Motor5Plot->yAxis->setLabelFont(fontLabel);
    ui->Motor5Plot->xAxis->setLabel("Time");
    ui->Motor5Plot->yAxis->setLabel("angle");

    ui->Motor6Plot->addGraph();
    ui->Motor6Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor6Plot->xAxis->setTickLabelFont(fontTick);
    ui->Motor6Plot->yAxis->setTickLabelFont(fontTick);
    ui->Motor6Plot->xAxis->setLabelFont(fontLabel);
    ui->Motor6Plot->yAxis->setLabelFont(fontLabel);
    ui->Motor6Plot->xAxis->setLabel("Time");
    ui->Motor6Plot->yAxis->setLabel("angle");

    ui->elbpresPlot->addGraph();
    ui->elbpresPlot->graph(0)->setPen(QPen(Qt::red));
    ui->elbpresPlot->xAxis->setTickLabelFont(fontTick);
    ui->elbpresPlot->yAxis->setTickLabelFont(fontTick);
    ui->elbpresPlot->xAxis->setLabelFont(fontLabel);
    ui->elbpresPlot->yAxis->setLabelFont(fontLabel);
    ui->elbpresPlot->xAxis->setLabel("Time");
    ui->elbpresPlot->yAxis->setLabel("elbow_press");

    ui->shopresPlot->legend->setVisible(true);
    ui->shopresPlot->xAxis->setLabel("time");
    ui->shopresPlot->yAxis->setLabel("shou_press()");
    ui->shopresPlot->xAxis->setTickLabelFont(fontTick);
    ui->shopresPlot->yAxis->setTickLabelFont(fontTick);
    ui->shopresPlot->xAxis->setLabelFont(fontLabel);
    ui->shopresPlot->yAxis->setLabelFont(fontLabel);

    ui->shopresPlot->addGraph();
    ui->shopresPlot->graph(0)->setPen(QPen(Qt::red));
    ui->shopresPlot->graph(0)->setName("shou_press_z");

    ui->shopresPlot->addGraph();
    ui->shopresPlot->graph(1)->setPen(QPen(Qt::green));
    ui->shopresPlot->graph(1)->setName("shou_press_x");

    ui->emgFitPlot->legend->setVisible(true);
    ui->emgFitPlot->xAxis->setLabel("theta");
    ui->emgFitPlot->yAxis->setLabel("emg");
    ui->emgFitPlot->xAxis->setTickLabelFont(fontTick);
    ui->emgFitPlot->yAxis->setTickLabelFont(fontTick);
    ui->emgFitPlot->xAxis->setLabelFont(fontLabel);
    ui->emgFitPlot->yAxis->setLabelFont(fontLabel);

    ui->emgFitPlot->addGraph();
    ui->emgFitPlot->graph(0)->setPen(QPen(Qt::red));
    ui->emgFitPlot->graph(0)->setName("measurement");

    ui->emgFitPlot->addGraph();
    ui->emgFitPlot->graph(1)->setPen(QPen(Qt::green));
    ui->emgFitPlot->graph(1)->setName("fit curve");

    plot_timer=new QTimer(this);
    //plot_timerdly = TIME_PLOT_INTERVAL;
    //connect(plot_timer,SIGNAL(timeout()),this,SLOT(plot()));

}

// Initialize mainwindow
void MainWindow::startInit()
{
    setActionsEnable(false);
    ui->actionExit->setEnabled(true);
    ui->actionStopMeasure->setEnabled(false);
    unsigned int initialValueMin = 200;
    unsigned int initialValueMax = 4000;

    ui->horizontalSlider->setMinimum(initialValueMin);
    ui->horizontalSlider->setMaximum(initialValueMax);

    ui->horizontalSlider2->setMinimum(initialValueMin);
    ui->horizontalSlider2->setMaximum(initialValueMax);

    ui->horizontalSlider3->setMinimum(initialValueMin);
    ui->horizontalSlider3->setMaximum(initialValueMax);

    ui->horizontalSlider4->setMinimum(initialValueMin);
    ui->horizontalSlider4->setMaximum(initialValueMax);

    ui->horizontalSlider5->setMinimum(initialValueMin);
    ui->horizontalSlider5->setMaximum(initialValueMax);

    ui->horizontalSlider6->setMinimum(initialValueMin);
    ui->horizontalSlider6->setMaximum(initialValueMax);

    ui->horizontalSlider7->setMinimum(0);
    ui->horizontalSlider7->setMaximum(90);

    ui->horizontalSlider8->setMinimum(0);
    ui->horizontalSlider8->setMaximum(90);

    ui->horizontalSlider9->setMinimum(-60);
    ui->horizontalSlider9->setMaximum(60);

    ui->horizontalSlider10->setMinimum(0);
    ui->horizontalSlider10->setMaximum(120);

    //
    ui->horizontalLayout->setStretchFactor(ui->label_6,2);
    ui->horizontalLayout->setStretchFactor(ui->horizontalSlider,6);
    ui->horizontalLayout->setStretchFactor(ui->sendMsgLineEdit,2);

    ui->horizontalLayout_2->setStretchFactor(ui->label_7,2);
    ui->horizontalLayout_2->setStretchFactor(ui->horizontalSlider2,6);
    ui->horizontalLayout_2->setStretchFactor(ui->sendMsgLineEdit2,2);

    ui->horizontalLayout_3->setStretchFactor(ui->label_8,2);
    ui->horizontalLayout_3->setStretchFactor(ui->horizontalSlider3,6);
    ui->horizontalLayout_3->setStretchFactor(ui->sendMsgLineEdit3,2);

    ui->horizontalLayout_4->setStretchFactor(ui->label_9,2);
    ui->horizontalLayout_4->setStretchFactor(ui->horizontalSlider4,6);
    ui->horizontalLayout_4->setStretchFactor(ui->sendMsgLineEdit4,2);

    ui->horizontalLayout_5->setStretchFactor(ui->label_10,2);
    ui->horizontalLayout_5->setStretchFactor(ui->horizontalSlider5,6);
    ui->horizontalLayout_5->setStretchFactor(ui->sendMsgLineEdit5,2);

    ui->horizontalLayout_6->setStretchFactor(ui->label_11,2);
    ui->horizontalLayout_6->setStretchFactor(ui->horizontalSlider6,6);
    ui->horizontalLayout_6->setStretchFactor(ui->sendMsgLineEdit6,2);

    ui->horizontalLayout_7->setStretchFactor(ui->label_20,2);
    ui->horizontalLayout_7->setStretchFactor(ui->horizontalSlider7,6);
    ui->horizontalLayout_7->setStretchFactor(ui->sendMsgLineEdit7,2);

    ui->horizontalLayout_8->setStretchFactor(ui->label_21,2);
    ui->horizontalLayout_8->setStretchFactor(ui->horizontalSlider8,6);
    ui->horizontalLayout_8->setStretchFactor(ui->sendMsgLineEdit8,2);

    ui->horizontalLayout_9->setStretchFactor(ui->label_22,2);
    ui->horizontalLayout_9->setStretchFactor(ui->horizontalSlider9,6);
    ui->horizontalLayout_9->setStretchFactor(ui->sendMsgLineEdit9,2);

    ui->horizontalLayout_10->setStretchFactor(ui->label_23,2);
    ui->horizontalLayout_10->setStretchFactor(ui->horizontalSlider10,6);
    ui->horizontalLayout_10->setStretchFactor(ui->sendMsgLineEdit10,2);

    Plot_Init();

    ui->sendMsgLineEdit->setText(QString::number(long(initialValueMin)));
    ui->sendMsgLineEdit2->setText(QString::number(long(initialValueMin)));
    ui->sendMsgLineEdit3->setText(QString::number(long(initialValueMin)));
    ui->sendMsgLineEdit4->setText(QString::number(long(initialValueMin)));
    ui->sendMsgLineEdit5->setText(QString::number(long(initialValueMin)));
    ui->sendMsgLineEdit6->setText(QString::number(long(initialValueMin)));
    ui->sendMsgLineEdit7->setText("0");
    ui->sendMsgLineEdit8->setText("0");
    ui->sendMsgLineEdit9->setText("0");
    ui->sendMsgLineEdit10->setText("0");

    connect(ui->horizontalSlider,SIGNAL(valueChanged(int)),this,SLOT(setLine1EditValue()));
    connect(ui->horizontalSlider2,SIGNAL(valueChanged(int)),this,SLOT(setLine2EditValue()));
    connect(ui->horizontalSlider3,SIGNAL(valueChanged(int)),this,SLOT(setLine3EditValue()));
    connect(ui->horizontalSlider4,SIGNAL(valueChanged(int)),this,SLOT(setLine4EditValue()));
    connect(ui->horizontalSlider5,SIGNAL(valueChanged(int)),this,SLOT(setLine5EditValue()));
    connect(ui->horizontalSlider6,SIGNAL(valueChanged(int)),this,SLOT(setLine6EditValue()));
    connect(ui->horizontalSlider7,SIGNAL(valueChanged(int)),this,SLOT(setLine7EditValue()));
    connect(ui->horizontalSlider8,SIGNAL(valueChanged(int)),this,SLOT(setLine8EditValue()));
    connect(ui->horizontalSlider9,SIGNAL(valueChanged(int)),this,SLOT(setLine9EditValue()));
    connect(ui->horizontalSlider10,SIGNAL(valueChanged(int)),this,SLOT(setLine10EditValue()));

}

// Set close button and save button action
void MainWindow::setActionsEnable(bool status)
{
    ui->actionClose->setEnabled(status);
    ui->actionSave->setEnabled(status);
}

//
void MainWindow::on_actionOpen_triggered()
{
    threadModbus->start();
    ui->actionOpen->setEnabled(false);

    setActionsEnable(true);
}

//
void MainWindow::on_actionClose_triggered()
{
    emit sigModbusClose();
    ui->actionOpen->setEnabled(true);
    setActionsEnable(false);
    ui->actionSave->setEnabled(true);
    ui->actionExit->setEnabled(true);
    ui->statusBar->showMessage(tr(""));
    if(!threadModbus->isRunning())
        return;
    threadModbus->quit();
    threadModbus->wait();
}


void MainWindow::on_actionExit_triggered()
{
    this->close();
}

//save to excel file
void MainWindow::on_actionSave_triggered()
{
    plot_timer->stop();
    unsigned int i;

    QAxObject *pApplication = NULL;
    QAxObject *pWorkBooks = NULL;
    QAxObject *pWorkBook = NULL;
    QAxObject *pSheets = NULL;
    QAxObject *pSheet = NULL;

    pApplication = new QAxObject();
    pApplication->setControl("Excel.Application");
    pApplication->dynamicCall("SetVisible(bool)", false);
    pApplication->setProperty("DisplayAlerts", false);
    pWorkBooks = pApplication->querySubObject("Workbooks");

    QString filename=QFileDialog::getSaveFileName(this,tr(""),tr("test.xlsx"));
    QFile file(filename);
    if(file.exists())
    {
        pWorkBook = pWorkBooks->querySubObject("Open(const QString &)", filename);
    }
    else
    {
        pWorkBooks->dynamicCall("Add");
        pWorkBook = pApplication->querySubObject("ActiveWorkBook");
    }
    pSheets = pWorkBook->querySubObject("Sheets");
    pSheet = pSheets->querySubObject("Item(int)", 1);

    QAxObject *cell_1 = pSheet->querySubObject("Cells(int, int)", 1, 1);
    cell_1->dynamicCall("SetValue(const QVariant&)", QVariant("ten1(g)"));

    QAxObject *cell_2 = pSheet->querySubObject("Cells(int, int)", 1, 2);
    cell_2->dynamicCall("SetValue(const QVariant&)",QVariant("ten2(g)"));

    QAxObject *cell_3 = pSheet->querySubObject("Cells(int, int)", 1, 3);
    cell_3->dynamicCall("SetValue(const QVariant&)", QVariant("ten3(g)"));

    QAxObject *cell_4 = pSheet->querySubObject("Cells(int, int)", 1, 4);
    cell_4->dynamicCall("SetValue(const QVariant&)", QVariant("ten4(g)"));

    QAxObject *cell_5 = pSheet->querySubObject("Cells(int, int)", 1, 5);
    cell_5->dynamicCall("SetValue(const QVariant&)", QVariant("ten5(g)"));

    QAxObject *cell_6 = pSheet->querySubObject("Cells(int, int)", 1, 6);
    cell_6->dynamicCall("SetValue(const QVariant&)", QVariant("ten6(g)"));

    QAxObject *cell_7 = pSheet->querySubObject("Cells(int, int)", 1, 7);
    cell_7->dynamicCall("SetValue(const QVariant&)", QVariant("elbow_z"));

    QAxObject *cell_8 = pSheet->querySubObject("Cells(int, int)", 1, 8);
    cell_8->dynamicCall("SetValue(const QVariant&)", QVariant("elb_x"));

    QAxObject *cell_9 = pSheet->querySubObject("Cells(int, int)", 1, 9);
    cell_9->dynamicCall("SetValue(const QVariant&)", QVariant("elb_y"));

    QAxObject *cell_10 = pSheet->querySubObject("Cells(int, int)", 1, 10);
    cell_10->dynamicCall("SetValue(const QVariant&)", QVariant("sho_z"));

    QAxObject *cell_11 = pSheet->querySubObject("Cells(int, int)", 1, 11);
    cell_11->dynamicCall("SetValue(const QVariant&)", QVariant("sho_x"));

    QAxObject *cell_12 = pSheet->querySubObject("Cells(int, int)", 1, 12);
    cell_12->dynamicCall("SetValue(const QVariant&)", QVariant("sho_y"));

    QAxObject *cell_13 = pSheet->querySubObject("Cells(int, int)", 1, 13);
    cell_13->dynamicCall("SetValue(const QVariant&)", QVariant("moto_1"));

    QAxObject *cell_14 = pSheet->querySubObject("Cells(int, int)", 1, 14);
    cell_14->dynamicCall("SetValue(const QVariant&)", QVariant("moto_2"));

    QAxObject *cell_15 = pSheet->querySubObject("Cells(int, int)", 1, 15);
    cell_15->dynamicCall("SetValue(const QVariant&)", QVariant("moto_3"));

    QAxObject *cell_16 = pSheet->querySubObject("Cells(int, int)", 1, 16);
    cell_16->dynamicCall("SetValue(const QVariant&)", QVariant("moto_4"));

    QAxObject *cell_17 = pSheet->querySubObject("Cells(int, int)", 1, 17);
    cell_17->dynamicCall("SetValue(const QVariant&)", QVariant("moto_5"));

    QAxObject *cell_18 = pSheet->querySubObject("Cells(int, int)", 1, 18);
    cell_18->dynamicCall("SetValue(const QVariant&)", QVariant("moto_6"));

    QAxObject *cell_19 = pSheet->querySubObject("Cells(int, int)", 1, 19);
    cell_19->dynamicCall("SetValue(const QVariant&)", QVariant("elPre"));

    QAxObject *cell_20 = pSheet->querySubObject("Cells(int, int)", 1, 20);
    cell_20->dynamicCall("SetValue(const QVariant&)", QVariant("shoPre1"));

    QAxObject *cell_21 = pSheet->querySubObject("Cells(int, int)", 1, 21);
    cell_21->dynamicCall("SetValue(const QVariant&)", QVariant("shoPre2"));

    QAxObject *cell_22 = pSheet->querySubObject("Cells(int, int)", 1, 22);
    cell_22->dynamicCall("SetValue(const QVariant&)", QVariant("Emgvalue"));

    QAxObject *cell_23 = pSheet->querySubObject("Cells(int, int)", 1, 23);
    cell_23->dynamicCall("SetValue(const QVariant&)", QVariant("ElbowAngle_emg"));

    QAxObject *cell_24 = pSheet->querySubObject("Cells(int, int)", 1, 24);
    cell_24->dynamicCall("SetValue(const QVariant&)", QVariant("ElbowAngle_fit"));


    // Change one-dimension to two-dimension array
    // Using QVarient in excel
    QVariantList vars;
    QString RangeStr;

    for(i=0; i<receive_count_tension; i++)
    {
        QList<QVariant> rows;
        rows.append(tension_y[i]);
        vars.append(QVariant(rows));
    }
    QVariant res(vars);
    RangeStr = "A2:A" + QString::number(receive_count_tension+1);
    QAxObject *user_range = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range->setProperty("Value",res);
    vars.clear();

    for(i=0; i<receive_count_tension; i++)
    {
        QList<QVariant> rows;
        rows.append(tension_y2[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_1(vars);
    RangeStr = "B2:B" + QString::number(receive_count_tension+1);
    QAxObject *user_range1 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range1->setProperty("Value",res_1);
    vars.clear();

    for(i=0; i<receive_count_tension; i++)
    {
        QList<QVariant> rows;
        rows.append(tension_y3[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_2(vars);
    RangeStr = "C2:C" + QString::number(receive_count_tension+1);
    QAxObject *user_range2 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range2->setProperty("Value",res_2);
    vars.clear();

    for(i=0; i<receive_count_tension; i++)
    {
        QList<QVariant> rows;
        rows.append(tension_y4[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_3(vars);
    RangeStr = "D2:D" + QString::number(receive_count_tension+1);
    QAxObject *user_range3 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range3->setProperty("Value",res_3);
    vars.clear();

    for(i=0; i<receive_count_tension; i++)
    {
        QList<QVariant> rows;
        rows.append(tension_y5[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_4(vars);
    RangeStr = "E2:E" + QString::number(receive_count_tension+1);
    QAxObject *user_range4 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range4->setProperty("Value",res_4);
    vars.clear();

    for(i=0; i<receive_count_tension; i++)
    {
        QList<QVariant> rows;
        rows.append(tension_y6[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_5(vars);
    RangeStr = "F2:F" + QString::number(receive_count_tension+1);
    QAxObject *user_range5 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range5->setProperty("Value",res_5);
    vars.clear();


    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(elbow_z[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_6(vars);
    RangeStr = "G2:G" + QString::number(receive_count_angle);
    QAxObject *user_range6 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range6->setProperty("Value",res_6);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(elbow_x[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_7(vars);
    RangeStr = "H2:H" + QString::number(receive_count_angle);
    QAxObject *user_range7 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range7->setProperty("Value",res_7);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(elbow_y[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_8(vars);
    RangeStr = "I2:I" + QString::number(receive_count_angle);
    QAxObject *user_range8 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range8->setProperty("Value",res_8);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(shoulder_z[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_9(vars);
    RangeStr = "J2:J" + QString::number(receive_count_angle);
    QAxObject *user_range9 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range9->setProperty("Value",res_9);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(shoulder_x[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_10(vars);
    RangeStr = "K2:K" + QString::number(receive_count_angle);
    QAxObject *user_range10 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range10->setProperty("Value",res_10);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(shoulder_y[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_11(vars);
    RangeStr = "L2:L" + QString::number(receive_count_angle);
    QAxObject *user_range11 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range11->setProperty("Value",res_11);
    vars.clear();

    for(i=0; i<receive_count_mocount; i++)
    {
        QList<QVariant> rows;
        rows.append(Motor1Count[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_12(vars);
    RangeStr = "M2:M" + QString::number(receive_count_mocount);
    QAxObject *user_range12 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range12->setProperty("Value",res_12);
    vars.clear();

    for(i=0; i<receive_count_mocount; i++)
    {
        QList<QVariant> rows;
        rows.append(Motor2Count[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_13(vars);
    RangeStr = "N2:N" + QString::number(receive_count_mocount);
    QAxObject *user_range13 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range13->setProperty("Value",res_13);
    vars.clear();

    for(i=0; i<receive_count_mocount; i++)
    {
        QList<QVariant> rows;
        rows.append(Motor3Count[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_14(vars);
    RangeStr = "O2:O" + QString::number(receive_count_mocount);
    QAxObject *user_range14 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range14->setProperty("Value",res_14);
    vars.clear();

    for(i=0; i<receive_count_mocount; i++)
    {
        QList<QVariant> rows;
        rows.append(Motor4Count[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_15(vars);
    RangeStr = "P2:P" + QString::number(receive_count_mocount);
    QAxObject *user_range15 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range15->setProperty("Value",res_15);
    vars.clear();

    for(i=0; i<receive_count_mocount; i++)
    {
        QList<QVariant> rows;
        rows.append(Motor5Count[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_16(vars);
    RangeStr = "Q2:Q" + QString::number(receive_count_mocount);
    QAxObject *user_range16 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range16->setProperty("Value",res_16);
    vars.clear();

    for(i=0; i<receive_count_mocount; i++)
    {
        QList<QVariant> rows;
        rows.append(Motor6Count[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_17(vars);
    RangeStr = "R2:R" + QString::number(receive_count_mocount);
    QAxObject *user_range17 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range17->setProperty("Value",res_17);
    vars.clear();

    for(i=0; i<receive_count_pressure; i++)
    {
        QList<QVariant> rows;
        rows.append(surpressure_elbow[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_18(vars);
    RangeStr = "S2:S" + QString::number(receive_count_pressure);
    QAxObject *user_range18 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range18->setProperty("Value",res_18);
    vars.clear();

    for(i=0; i<receive_count_pressure; i++)
    {
        QList<QVariant> rows;
        rows.append(surpressure_shou1[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_19(vars);
    RangeStr = "T2:T" + QString::number(receive_count_pressure);
    QAxObject *user_range19 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range19->setProperty("Value",res_19);
    vars.clear();

    for(i=0; i<receive_count_pressure; i++)
    {
        QList<QVariant> rows;
        rows.append(surpressure_shou2[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_20(vars);
    RangeStr = "U2:U" + QString::number(receive_count_pressure);
    QAxObject *user_range20 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range20->setProperty("Value",res_20);
    vars.clear();

    // the emg data and the relevant elbow angle
    for(i=0; i<emgsaveLen; i++)
    {
        QList<QVariant> rows;
        rows.append(EmgDataSave[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_21(vars);
    RangeStr = "V2:V" + QString::number(emgsaveLen);
    QAxObject *user_range21 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range21->setProperty("Value",res_21);
    vars.clear();

    for(i=0; i<emgsaveLen; i++)
    {
        QList<QVariant> rows;
        rows.append(AngleElbow_emg[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_22(vars);
    RangeStr = "W2:W" + QString::number(emgsaveLen);
    QAxObject *user_range22 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range22->setProperty("Value",res_22);
    vars.clear();

    for(i=0; i<fiteffRecord.size(); i++)
    {
        QList<QVariant> rows;
        rows.append(fiteffRecord[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_23(vars);
    RangeStr = "W2:W" + QString::number(fiteffRecord.length());
    QAxObject *user_range23 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range23->setProperty("Value",res_23);
    vars.clear();


    //excel
    pWorkBook->dynamicCall("SaveAs(const QString &)", QDir::toNativeSeparators(filename));
    pApplication->dynamicCall("Quit()");
    delete pApplication;
}


void MainWindow::on_actionClean_triggered()
{
    plot_timer->stop();

    ui->qCustomPlot->clearGraphs();
    ui->qCustomPlot->replot();

    ui->qCustomPlot2->clearGraphs();
    ui->qCustomPlot2->replot();

    ui->qCustomPlot3->clearGraphs();
    ui->qCustomPlot3->replot();

    ui->qCustomPlot4->clearGraphs();
    ui->qCustomPlot4->replot();

    ui->qCustomPlot5->clearGraphs();
    ui->qCustomPlot5->replot();

    ui->qCustomPlot6->clearGraphs();
    ui->qCustomPlot6->replot();

    ui->qCustomPlot7->clearGraphs();
    ui->qCustomPlot7->replot();

    ui->qCustomPlot8->clearGraphs();
    ui->qCustomPlot8->replot();

    ui->qCustomPlot9->clearGraphs();
    ui->qCustomPlot9->replot();

    ui->qCustomPlot10->clearGraphs();
    ui->qCustomPlot10->replot();

    ui->qCustomPlot11->clearGraphs();
    ui->qCustomPlot11->replot();

    ui->qCustomPlot12->clearGraphs();
    ui->qCustomPlot12->replot();

    ui->Motor1Plot->clearGraphs();
    ui->Motor1Plot->replot();

    ui->Motor2Plot->clearGraphs();
    ui->Motor2Plot->replot();

    ui->Motor3Plot->clearGraphs();
    ui->Motor3Plot->replot();

    ui->Motor4Plot->clearGraphs();
    ui->Motor4Plot->replot();

    ui->Motor5Plot->clearGraphs();
    ui->Motor5Plot->replot();

    ui->Motor6Plot->clearGraphs();
    ui->Motor6Plot->replot();

    ui->emgFitPlot->clearGraphs();
    ui->emgFitPlot->replot();

    Plot_Init();

    receive_count_tension = 0;
    receive_count_angle = 0;
    receive_count_pressure = 0;
    receive_count_mocount = 0;

    for(int i=0;i<6;i++)
        max_tension[i]=0;
    ui->statusBar->showMessage(tr("data cleaned"));

}

void MainWindow::plot()
{

    linkdisplay->adjustPos(shoulder_x[receive_count_angle]*3.14/180,
            shoulder_y[receive_count_angle]*3.14/180,
            shoulder_z[receive_count_angle]*3.14/180,
            elbow_x[receive_count_angle]*3.14/180,
            elbow_y[receive_count_angle]*3.14/180,
            elbow_z[receive_count_angle]*3.14/180);

    QPen pen;
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(3);
    pen.setBrush(Qt::blue);

    // test, only paint the latest one hundred data;
    unsigned int plotlength = 100;
    QVector<double> time_x_plot(plotlength),tensionPlot(plotlength),tension2Plot(plotlength),tension3Plot(plotlength),tension4Plot(plotlength),tension5Plot(100),tension6Plot(plotlength);
    QVector<double> elbowXPlot(plotlength),elbowYPlot(plotlength),elbowZPlot(plotlength),shoulXPlot(plotlength),shoulYPlot(plotlength),shoulZPlot(plotlength);
    QVector<double> Mot1CntPlot(plotlength),Mot2CntPlot(plotlength),Mot3CntPlot(plotlength),Mot4CntPlot(plotlength),Mot5CntPlot(plotlength),Mot6CntPlot(plotlength);
    unsigned int k = 0;
    unsigned int p = 0;
    unsigned int s = 0;
    if(time_x_tension[receive_count_tension]>plotlength)
    {
        for(int i=time_x_tension[receive_count_tension]-plotlength; i<time_x_tension[receive_count_tension]; i++)
        {
            time_x_plot[k] = i;
            tensionPlot[k] = tension_y[i];
            tension2Plot[k] = tension_y2[i];
            tension3Plot[k] = tension_y3[i];
            tension4Plot[k] = tension_y4[i];
            tension5Plot[k] = tension_y5[i];
            tension6Plot[k] = tension_y6[i];
            k++;
        }

        for(int i=time_x_angle[receive_count_angle]-plotlength; i<time_x_angle[receive_count_angle]; i++)
        {
            elbowXPlot[p] = elbow_x[i];
            elbowYPlot[p] = elbow_y[i];
            elbowZPlot[p] = elbow_z[i];
            shoulXPlot[p] = shoulder_x[i];
            shoulYPlot[p] = shoulder_y[i];
            shoulZPlot[p] = shoulder_z[i];
            p++;
        }     

        for(int i=time_x_mocount[receive_count_mocount]-plotlength; i<time_x_mocount[receive_count_mocount]; i++)
        {
            Mot1CntPlot[s] = Motor1Count[i];
            Mot2CntPlot[s] = Motor2Count[i];
            Mot3CntPlot[s] = Motor3Count[i];
            Mot4CntPlot[s] = Motor4Count[i];
            Mot5CntPlot[s] = Motor5Count[i];
            Mot6CntPlot[s] = Motor6Count[i];
            s++;
        }

        // tension data plot
        ui->qCustomPlot->graph(0)->setData(time_x_plot,tensionPlot);
        ui->qCustomPlot->graph(0)->setPen(pen);
        ui->qCustomPlot->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot->yAxis->setRange(0,max_tension[0]*1.1);
        ui->qCustomPlot->replot();

        ui->qCustomPlot2->graph(0)->setData(time_x_plot,tension2Plot);
        ui->qCustomPlot2->graph(0)->setPen(pen);
        ui->qCustomPlot2->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot2->yAxis->setRange(0,max_tension[1]*1.1);
        ui->qCustomPlot2->replot();

        ui->qCustomPlot3->graph(0)->setData(time_x_plot,tension3Plot);
        ui->qCustomPlot3->graph(0)->setPen(pen);
        ui->qCustomPlot3->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot3->yAxis->setRange(0,max_tension[2]*1.1);
        ui->qCustomPlot3->replot();

        ui->qCustomPlot4->graph(0)->setData(time_x_plot,tension4Plot);
        ui->qCustomPlot4->graph(0)->setPen(pen);
        ui->qCustomPlot4->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot4->yAxis->setRange(0,max_tension[3]*1.1);
        ui->qCustomPlot4->replot();

        ui->qCustomPlot5->graph(0)->setData(time_x_plot,tension5Plot);
        ui->qCustomPlot5->graph(0)->setPen(pen);
        ui->qCustomPlot5->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot5->yAxis->setRange(0,max_tension[4]*1.1);
        ui->qCustomPlot5->replot();

        ui->qCustomPlot6->graph(0)->setData(time_x_plot,tension6Plot);
        ui->qCustomPlot6->graph(0)->setPen(pen);
        ui->qCustomPlot6->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot6->yAxis->setRange(0,max_tension[5]*1.1);
        ui->qCustomPlot6->replot();

        // angle data plot
        ui->qCustomPlot7->graph(0)->setData(time_x_plot,elbowXPlot);
        ui->qCustomPlot7->graph(0)->setPen(pen);
        ui->qCustomPlot7->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot7->yAxis->setRange(-120,120);
        ui->qCustomPlot7->replot();

        ui->qCustomPlot8->graph(0)->setData(time_x_plot,elbowYPlot);
        ui->qCustomPlot8->graph(0)->setPen(pen);
        ui->qCustomPlot8->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot8->yAxis->setRange(-120,120);
        ui->qCustomPlot8->replot();

        ui->qCustomPlot9->graph(0)->setData(time_x_plot,elbowZPlot);
        ui->qCustomPlot9->graph(0)->setPen(pen);
        ui->qCustomPlot9->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot9->yAxis->setRange(-120,120);
        ui->qCustomPlot9->replot();

        ui->qCustomPlot10->graph(0)->setData(time_x_plot,shoulXPlot);
        ui->qCustomPlot10->graph(0)->setPen(pen);
        ui->qCustomPlot10->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot10->yAxis->setRange(-120,120);
        ui->qCustomPlot10->replot();

        ui->qCustomPlot11->graph(0)->setData(time_x_plot,shoulYPlot);
        ui->qCustomPlot11->graph(0)->setPen(pen);
        ui->qCustomPlot11->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot11->yAxis->setRange(-120,120);
        ui->qCustomPlot11->replot();

        ui->qCustomPlot12->graph(0)->setData(time_x_plot,shoulZPlot);
        ui->qCustomPlot12->graph(0)->setPen(pen);
        ui->qCustomPlot12->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->qCustomPlot12->yAxis->setRange(-120,120);
        ui->qCustomPlot12->replot();

        ui->Motor1Plot->graph(0)->setData(time_x_plot,Mot1CntPlot);
        ui->Motor1Plot->graph(0)->setPen(pen);
        ui->Motor1Plot->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->Motor1Plot->yAxis->setRange(min_motor_count[0],max_motor_count[0]*1.1);
        ui->Motor1Plot->replot();

        ui->Motor2Plot->graph(0)->setData(time_x_plot,Mot2CntPlot);
        ui->Motor2Plot->graph(0)->setPen(pen);
        ui->Motor2Plot->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->Motor2Plot->yAxis->setRange(min_motor_count[1],max_motor_count[1]*1.1);
        ui->Motor2Plot->replot();

        ui->Motor3Plot->graph(0)->setData(time_x_plot,Mot3CntPlot);
        ui->Motor3Plot->graph(0)->setPen(pen);
        ui->Motor3Plot->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->Motor3Plot->yAxis->setRange(min_motor_count[2],max_motor_count[2]*1.1);
        ui->Motor3Plot->replot();

        ui->Motor4Plot->graph(0)->setData(time_x_plot,Mot4CntPlot);
        ui->Motor4Plot->graph(0)->setPen(pen);
        ui->Motor4Plot->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->Motor4Plot->yAxis->setRange(min_motor_count[3],max_motor_count[3]*1.1);
        ui->Motor4Plot->replot();

        ui->Motor5Plot->graph(0)->setData(time_x_plot,Mot5CntPlot);
        ui->Motor5Plot->graph(0)->setPen(pen);
        ui->Motor5Plot->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->Motor5Plot->yAxis->setRange(min_motor_count[4],max_motor_count[4]*1.1);
        ui->Motor5Plot->replot(QCustomPlot::rpQueuedReplot);

        ui->Motor6Plot->graph(0)->setData(time_x_plot,Mot6CntPlot);
        ui->Motor6Plot->graph(0)->setPen(pen);
        ui->Motor6Plot->xAxis->setRange(time_x_plot[0],time_x_plot[plotlength-1]);
        ui->Motor6Plot->yAxis->setRange(min_motor_count[5],max_motor_count[5]*1.1);
        ui->Motor6Plot->replot(QCustomPlot::rpQueuedReplot);
    }
}

/*****************************/

void MainWindow::setLine1EditValue()
{
    int pos=ui->horizontalSlider->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit->setText(str);
}

void MainWindow::setLine2EditValue()
{
    int pos=ui->horizontalSlider2->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit2->setText(str);
}

void MainWindow::setLine3EditValue()
{
    int pos=ui->horizontalSlider3->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit3->setText(str);
}

void MainWindow::setLine4EditValue()
{
    int pos=ui->horizontalSlider4->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit4->setText(str);
}

void MainWindow::setLine5EditValue()
{
    int pos=ui->horizontalSlider5->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit5->setText(str);
}

void MainWindow::setLine6EditValue()
{
    int pos=ui->horizontalSlider6->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit6->setText(str);
}

void MainWindow::setLine7EditValue()
{
    int pos=ui->horizontalSlider7->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit7->setText(str);
}

void MainWindow::setLine8EditValue()
{
    int pos=ui->horizontalSlider8->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit8->setText(str);
}

void MainWindow::setLine9EditValue()
{
    int pos=ui->horizontalSlider9->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit9->setText(str);
}

void MainWindow::setLine10EditValue()
{
    int pos=ui->horizontalSlider10->value();
    QString str=QString("%1").arg(pos);
    ui->sendMsgLineEdit10->setText(str);
}

void MainWindow::on_VRDisplay_clicked()
{
    emit sigVRSerialOpen();
}

void MainWindow::on_sendmsgButton_clicked()
{
    uint TensionOrAngle=0;
    int sendData[10];
    if(ui->actionOpen->isEnabled())
    {
        QMessageBox::critical(this,tr("wrong operation"),tr("open com first!!!"),QMessageBox::Ok);
        return;
    }
    // JOINNT CONTROL MODE
    if(ui->joint_RadioButton->isChecked())
    {
        TensionOrAngle = 1;
        sendData[0] = ui->sendMsgLineEdit7->text().toInt();
        sendData[1] = ui->sendMsgLineEdit8->text().toInt();
        sendData[2] = ui->sendMsgLineEdit9->text().toInt();
        sendData[3] = ui->sendMsgLineEdit10->text().toUInt();
        sendData[4] = ui->circlelineEdit->text().toUInt();
        if(sendData[4] == 0)
        {
            QMessageBox::critical(this,tr("wrong operation"),tr("circle can not be zero"),QMessageBox::Ok);
            return;
        }
        else
            emit sigMdSerialCtrl(TensionOrAngle, sendData);
    }
    // CABEL TENSION MODE
    else if(ui->cabel_RadioButton->isChecked())
    {
        TensionOrAngle = 0;
        sendData[0] = ui->sendMsgLineEdit->text().toInt();
        sendData[1] = ui->sendMsgLineEdit2->text().toInt();
        sendData[2] = ui->sendMsgLineEdit3->text().toInt();
        sendData[3] = ui->sendMsgLineEdit4->text().toInt();
        sendData[4] = ui->sendMsgLineEdit5->text().toInt();
        sendData[5] = ui->sendMsgLineEdit6->text().toInt();
        sendData[6] = ui->TensionP->text().toUInt(); // 100*kp
        sendData[7] = ui->TensionD->text().toUInt(); // 100*kd
        sendData[8] = ui->TensionI->text().toUInt(); // 100*ki
        emit sigMdSerialCtrl(TensionOrAngle, sendData);
    }
    // PTP CONTROL MODE
    else if(ui->Position_RadioButton->isChecked())
    {
        TensionOrAngle = 2;
        sendData[0] = ui->PosXlineEdit->text().toInt();
        sendData[1] = ui->PosYlineEdit->text().toInt();
        sendData[2] = ui->PosZlineEdit->text().toInt();
        emit sigMdSerialCtrl(TensionOrAngle, sendData);
    }
    // LINEAR CONTROL MODE
    else if(ui->Linear_RadioButton->isChecked())
    {
        TensionOrAngle = 3;
        sendData[0] = ui->circlelineEdit->text().toUInt();
        emit sigMdSerialCtrl(TensionOrAngle, sendData);
    }

    // Position PD control
    else if(ui->torque_RadioButton->isChecked())
    {
        TensionOrAngle = 4;
        sendData[0] = ui->sendMsgLineEdit7->text().toInt();
        sendData[1] = ui->sendMsgLineEdit8->text().toInt();
        sendData[2] = ui->sendMsgLineEdit9->text().toInt();
        sendData[3] = ui->sendMsgLineEdit10->text().toUInt();
        sendData[4] = ui->TensionP->text().toUInt(); // 100*kp
        sendData[5] = ui->TensionD->text().toUInt(); // 100*kd
        sendData[6] = ui->TensionI->text().toUInt(); // 1000*ki
        emit sigMdSerialCtrl(TensionOrAngle, sendData);
    }
    else
    {
        QMessageBox::critical(this,tr("wrong operation"),tr("choose one category first!!!"),QMessageBox::Ok);
        return;
    }
}

void MainWindow::on_actionStartMeasure_triggered()
{
    getsensordata->start();
    //plot_timer->start(plot_timerdly);
    ui->actionStartMeasure->setEnabled(false);
    ui->actionStopMeasure->setEnabled(true);
}

void MainWindow::on_actionStopMeasure_triggered()
{
    getsensordata->terminate();
    getsensordata->wait();
    ui->actionSave->setEnabled(true);
    ui->actionStopMeasure->setEnabled(false);
    ui->actionStartMeasure->setEnabled(true);
}

void MainWindow::on_BeforeTightenButton_clicked()
{
    if(ui->actionOpen->isEnabled())
    {
        QMessageBox::critical(this,tr("wrong operation"),tr("open com first!!!"),QMessageBox::Ok);
        return;
    }
    else
    {
        // 预紧信号发送
        unsigned int TensionData[6];
        for(int i=0; i<6; i++)
            TensionData[i] = 200;
        emit sigMdBeforeTigh(TensionData);
    }

}

void MainWindow::on_TeachButton_clicked()
{
    emit sigMdTeachStart();
}

void MainWindow::on_StopteachButton_clicked()
{
    emit sigMdTeachStop();
}

void MainWindow::on_ReplayButton_clicked()
{
    emit sigMdReplayTeach();
}

void MainWindow::on_pushButton_Listen_clicked()
{
    if(ui->pushButton_Listen->text() == tr("LISTEN"))
    {
        qDebug() << "Try to Listen!";
        emg_server = new EMG_server();
        ui->pushButton_Trigger->setEnabled(true);
        ui->pushButton_Listen->setText("LISTENING...");
    }
    else
    {
        qDebug() << "Stop listening!";
        emg_server->~EMG_server();
        ui->pushButton_Listen->setText("Listen");
        // disable the button
        ui->pushButton_Trigger->setEnabled(false);
    }
}

void MainWindow::on_pushButton_Start_clicked()
{
    qDebug() << "Start EMG!";
    emg_server->Send_Data("start");
    emit sigEmgStart();
}

void MainWindow::on_pushButton_Trigger_clicked()
{
    qDebug() << "Send Trigger!";
    emg_server->Send_Data("end");
    emit sigEmgTrigger();
}

void MainWindow::slotEmgThetaFit(double *fiteff, double *bufferX, double *bufferY, unsigned int dimension, int sizenum)
{
    //plot_timer->stop();
    //QVector<double> x_dot(200),y_dot(200);
    for(int i=0; i<10; i++)
        fiteffRecord.append(fiteff[i]); // record the fit value
    QVector<double> x_dot;
    QVector<double> y_dot;
    for(int i=0; i<sizenum; i++)
    {
        x_dot.append(bufferX[i]);
        y_dot.append(bufferY[i]);
        qDebug()<<"x_dot"<<i<<"is:"<<x_dot[i];
        qDebug()<<"y_dot"<<i<<"is:"<<y_dot[i];
    }

    QPen pen;
    ui->emgFitPlot->graph(0)->setPen(QPen(Qt::blue));
    ui->emgFitPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->emgFitPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->emgFitPlot->graph(0)->setData(x_dot,y_dot);
    ui->emgFitPlot->xAxis->setRange(-10,60);
    ui->emgFitPlot->yAxis->setRange(-20,50);

    double max,min;
    max = bufferX[0];
    min = bufferX[0];
    for(int i=0; i<sizenum; i++)
    {
        if(min > bufferX[i])
            min = bufferX[i];
        if(max < bufferX[i])
            max = bufferX[i];
    }
    qDebug()<<"min is:"<<min;
    qDebug()<<"max is:"<<max;
    QVector<double> x(1000),y(1000);
    int k=0;
    for(int j=10*min; j<10*max; j++)
    {    
        x[k] = 0.1*j;
        for(unsigned int i=0; i<dimension+1; i++)
        {
            y[k] += fiteff[i]*qPow(x[k],dimension-i);
        }
        k++;
        //qDebug()<<"y"<<k<<"is:"<<y[k];
    }
    //qDebug()<<"y size is:"<<y.size();

    pen.setStyle(Qt::DashLine);
    pen.setWidth(2);
    pen.setColor(Qt::red);
    ui->emgFitPlot->graph(1)->setPen(pen);
    ui->emgFitPlot->graph(1)->setData(x,y);

    ui->emgFitPlot->replot();
}

void MainWindow::on_StopMotorButton_clicked()
{
    emit sigDisableMotor();
}

void MainWindow::modMessage(QString kind, QString message)
{

    if(kind == "noerror")
        QMessageBox::information(this, kind, message);
    if(kind == "error")
        QMessageBox::critical(this, kind, message, QMessageBox::Ok);
}
