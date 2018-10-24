#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "myglwidget.h"
#include <QDebug>
#include <QtGui>
#include <QVector>
#include <QList>
#include <QVariant>
#include <ActiveQt/QAxObject>
#include <qdebug.h>
#include "motorcontrol.h"
#include "vrdisplay.h"
#include "tensioncontrol.h"


unsigned int shift_x_tension = 0;
unsigned int shift_x_angle = 0;
unsigned int shift_x_pressure = 0;

#define RcvBufSize 380
unsigned int RcvBufNum;
unsigned char RcvBuf[RcvBufSize];
bool new_flag=0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    startInit();
    setWindowTitle(QString("Upper Limb Console"));
    ui->statusBar->showMessage(QString("welcome to use upper limb console"));

    getsensordata = new GetSensordata;
    motorcontrol = new MotorControl;
    vrdisplay = new VRDisplay;
    tensioncontrol = new TensionControl;


    // 3D surface
    //Q3DScatter *graph = new Q3DScatter();
    //QWidget *container = QWidget::createWindowContainer(graph);

    /*
    if (!graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
        return -1;
    }
    */


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

    // Before connect object must be instantiation!!!
    connect(this, SIGNAL(sigSerialInit()), tensioncontrol, SLOT(slotSerialInit()));
    connect(this, SIGNAL(sigSerialClose()), tensioncontrol, SLOT(slotSerialClose()));
    connect(this, SIGNAL(sigSerialCtrl(uint, int*)), tensioncontrol, SLOT(slotSerialCtrl(uint, int*)));
    connect(this, SIGNAL(sigBeforeTigh()), motorcontrol, SLOT(slotBeforeTigh()));
    connect(this,SIGNAL(sigVRSerialOpen()), vrdisplay, SLOT(slotVRSerialOpen()));
}

MainWindow::~MainWindow()
{
    getsensordata->terminate();
    getsensordata->wait();
    delete getsensordata;
    getsensordata = 0;

    motorcontrol->terminate();
    motorcontrol->wait();
    delete motorcontrol;
    motorcontrol = 0;

    tensioncontrol->terminate();
    tensioncontrol->wait();
    delete tensioncontrol;
    tensioncontrol = 0;

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

    QFont fontTick("Times New Roman", 10, 75);
    QFont fontLabel("Times New Roman", 10, 50);

    ui->qCustomPlot->addGraph();
    ui->qCustomPlot->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot->xAxis->setLabel("Time");
    ui->qCustomPlot->yAxis->setLabel("Tenison1");

    ui->qCustomPlot2->addGraph();
    ui->qCustomPlot2->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot2->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot2->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot2->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot2->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot2->xAxis->setLabel("Time");
    ui->qCustomPlot2->yAxis->setLabel("Tenison2");

    ui->qCustomPlot3->addGraph();
    ui->qCustomPlot3->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot3->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot3->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot3->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot3->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot3->xAxis->setLabel("Time");
    ui->qCustomPlot3->yAxis->setLabel("Tenison3");

    ui->qCustomPlot4->addGraph();
    ui->qCustomPlot4->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot4->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot4->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot4->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot4->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot4->xAxis->setLabel("Time");
    ui->qCustomPlot4->yAxis->setLabel("Tenison4");

    ui->qCustomPlot5->addGraph();
    ui->qCustomPlot5->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot5->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot5->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot5->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot5->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot5->xAxis->setLabel("Time");
    ui->qCustomPlot5->yAxis->setLabel("Tenison5");

    ui->qCustomPlot6->addGraph();
    ui->qCustomPlot6->graph(0)->setPen(QPen(Qt::red));
    ui->qCustomPlot6->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot6->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot6->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot6->yAxis->setLabelFont(fontLabel);
    ui->qCustomPlot6->xAxis->setLabel("Time");
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
    ui->qCustomPlot12->xAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot12->yAxis->setTickLabelFont(fontTick);
    ui->qCustomPlot12->xAxis->setLabelFont(fontLabel);
    ui->qCustomPlot12->yAxis->setLabelFont(fontLabel);
    ui->Motor1Plot->xAxis->setLabel("Time");
    ui->Motor1Plot->yAxis->setLabel("angle");

    ui->Motor2Plot->addGraph();
    ui->Motor2Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor2Plot->xAxis->setLabel("Time");
    ui->Motor2Plot->yAxis->setLabel("angle");

    ui->Motor3Plot->addGraph();
    ui->Motor3Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor3Plot->xAxis->setLabel("Time");
    ui->Motor3Plot->yAxis->setLabel("angle");

    ui->Motor4Plot->addGraph();
    ui->Motor4Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor4Plot->xAxis->setLabel("Time");
    ui->Motor4Plot->yAxis->setLabel("angle");

    ui->Motor5Plot->addGraph();
    ui->Motor5Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor5Plot->xAxis->setLabel("Time");
    ui->Motor5Plot->yAxis->setLabel("angle");

    ui->Motor6Plot->addGraph();
    ui->Motor6Plot->graph(0)->setPen(QPen(Qt::red));
    ui->Motor6Plot->xAxis->setLabel("Time");
    ui->Motor6Plot->yAxis->setLabel("angle");

    ui->elbpresPlot->addGraph();
    ui->elbpresPlot->graph(0)->setPen(QPen(Qt::red));
    ui->elbpresPlot->xAxis->setLabel("Time");
    ui->elbpresPlot->yAxis->setLabel("elbow_press");

    ui->shopresPlot->legend->setVisible(true);
    ui->shopresPlot->xAxis->setLabel("time");
    ui->shopresPlot->yAxis->setLabel("shou_press()");

    ui->shopresPlot->addGraph();
    ui->shopresPlot->graph(0)->setPen(QPen(Qt::red));
    ui->shopresPlot->graph(0)->setName("shou_press_z");

    ui->shopresPlot->addGraph();
    ui->shopresPlot->graph(1)->setPen(QPen(Qt::green));
    ui->shopresPlot->graph(1)->setName("shou_press_x");

    plot_timer=new QTimer(this);
    plot_timerdly = TIME_PLOT_INTERVAL;
    connect(plot_timer,SIGNAL(timeout()),this,SLOT(plot()));
    //plot_timer->start(plot_timerdly);

}

// Initialize mainwindow
void MainWindow::startInit()
{
    setActionsEnable(false);
    ui->actionExit->setEnabled(true);

    ui->horizontalSlider->setMinimum(100);
    ui->horizontalSlider->setMaximum(2000);

    ui->horizontalSlider2->setMinimum(100);
    ui->horizontalSlider2->setMaximum(2000);

    ui->horizontalSlider3->setMinimum(100);
    ui->horizontalSlider3->setMaximum(2000);

    ui->horizontalSlider4->setMinimum(100);
    ui->horizontalSlider4->setMaximum(2000);

    ui->horizontalSlider5->setMinimum(100);
    ui->horizontalSlider5->setMaximum(2000);

    ui->horizontalSlider6->setMinimum(100);
    ui->horizontalSlider6->setMaximum(2000);

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

    ui->sendMsgLineEdit->setText("100");
    ui->sendMsgLineEdit2->setText("100");
    ui->sendMsgLineEdit3->setText("100");
    ui->sendMsgLineEdit4->setText("100");
    ui->sendMsgLineEdit5->setText("100");
    ui->sendMsgLineEdit6->setText("100");
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
    emit sigSerialInit();
    //tensioncontrol->start();
    ui->actionOpen->setEnabled(false);

    setActionsEnable(true);
}

//
void MainWindow::on_actionClose_triggered()
{
    //plot_timer->stop();
    //serial.close();

    ui->actionOpen->setEnabled(true);

    setActionsEnable(false);

    ui->actionSave->setEnabled(true);
    ui->actionExit->setEnabled(true);

    ui->statusBar->showMessage(tr(""));

    emit sigSerialClose();
}


void MainWindow::on_actionExit_triggered()
{
    this->close();
}

//save to excel file
void MainWindow::on_actionSave_triggered()
{
    //rece_timer->stop();
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

    //
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

    QAxObject *cell_22 = pSheet->querySubObject("Cells(int,int)", 1, 22);
    cell_22->dynamicCall("SetValue(const QVariant)", QVariant("el_x_acc"));

    QAxObject *cell_23 = pSheet->querySubObject("Cells(int,int)", 1, 23);
    cell_23->dynamicCall("SetValue(const QVariant)", QVariant("el_y_acc"));

    QAxObject *cell_24 = pSheet->querySubObject("Cells(int,int)", 1, 24);
    cell_24->dynamicCall("SetValue(const QVariant)", QVariant("el_z_acc"));

    QAxObject *cell_25 = pSheet->querySubObject("Cells(int,int)", 1, 25);
    cell_25->dynamicCall("SetValue(const QVariant)", QVariant("sh_x_acc"));

    QAxObject *cell_26 = pSheet->querySubObject("Cells(int,int)", 1, 26);
    cell_26->dynamicCall("SetValue(const QVariant)", QVariant("sh_y_acc"));

    QAxObject *cell_27 = pSheet->querySubObject("Cells(int,int)", 1, 27);
    cell_27->dynamicCall("SetValue(const QVariant)", QVariant("sh_z_acc"));

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

    for(i=0; i<receive_count_pressure; i++)
    {
        QList<QVariant> rows;
        rows.append(surpressure_elbow[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_12(vars);
    RangeStr = "S2:S" + QString::number(receive_count_pressure);
    QAxObject *user_range12 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range12->setProperty("Value",res_12);
    vars.clear();

    for(i=0; i<receive_count_pressure; i++)
    {
        QList<QVariant> rows;
        rows.append(surpressure_shou1[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_13(vars);
    RangeStr = "T2:T" + QString::number(receive_count_pressure);
    QAxObject *user_range13 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range13->setProperty("Value",res_13);
    vars.clear();

    for(i=0; i<receive_count_pressure; i++)
    {
        QList<QVariant> rows;
        rows.append(surpressure_shou2[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_14(vars);
    RangeStr = "U2:U" + QString::number(receive_count_pressure);
    QAxObject *user_range14 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range14->setProperty("Value",res_14);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(elbow_x_acc[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_15(vars);
    RangeStr = "V2:V" + QString::number(receive_count_angle);
    QAxObject *user_range15 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range15->setProperty("Value",res_15);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(elbow_y_acc[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_16(vars);
    RangeStr = "W2:W" + QString::number(receive_count_angle);
    QAxObject *user_range16 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range16->setProperty("Value",res_16);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(elbow_z_acc[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_17(vars);
    RangeStr = "X2:X" + QString::number(receive_count_angle);
    QAxObject *user_range17 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range17->setProperty("Value",res_17);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(shoulder_x_acc[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_18(vars);
    RangeStr = "Y2:Y" + QString::number(receive_count_angle);
    QAxObject *user_range18 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range18->setProperty("Value",res_18);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(shoulder_y_acc[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_19(vars);
    RangeStr = "Z2:Z" + QString::number(receive_count_angle);
    QAxObject *user_range19 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range19->setProperty("Value",res_19);
    vars.clear();

    for(i=0; i<receive_count_angle; i++)
    {
        QList<QVariant> rows;
        rows.append(shoulder_z_acc[i]);
        vars.append(QVariant(rows));
    }
    QVariant res_20(vars);
    RangeStr = "AA2:AA" + QString::number(receive_count_angle);
    QAxObject *user_range20 = pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range20->setProperty("Value",res_20);
    vars.clear();

    //excel
    pWorkBook->dynamicCall("SaveAs(const QString &)", QDir::toNativeSeparators(filename));
    pApplication->dynamicCall("Quit()");
    delete pApplication;
}


void MainWindow::on_actionClean_triggered()
{

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

    Plot_Init();

    receive_count_tension = 0;
    receive_count_angle = 0;
    receive_count_pressure = 0;

    for(int i=0;i<6;i++)
        max_tension[i]=0;
    ui->statusBar->showMessage(tr("data cleaned"));
}

void MainWindow::plot()
{
    linkdisplay->adjustPos(shoulder_x[receive_count_angle-1]*3.14/180,
            shoulder_y[receive_count_angle-1]*3.14/180,
            shoulder_z[receive_count_angle-1]*3.14/180,
            elbow_x[receive_count_angle-1]*3.14/180,
            elbow_y[receive_count_angle-1]*3.14/180,
            elbow_z[receive_count_angle-1]*3.14/180);

    //  Six tension value figure
    if(receive_count_tension > 1)
        shift_x_tension = receive_count_tension*0.05 - 0.5;
    if(receive_count_angle > 1)
        shift_x_angle = receive_count_angle*0.05 - 0.5;
    if(receive_count_pressure > 1)
        shift_x_pressure = receive_count_pressure*0.05 - 0.5;

    QPen pen;
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(3);
    pen.setBrush(Qt::red);

    ui->qCustomPlot->graph(0)->setData(time_x_tension,tension_y);
    ui->qCustomPlot->graph(0)->setPen(pen);
    ui->qCustomPlot->xAxis->setRange(shift_x_tension,(receive_count_tension-1)*0.05);
    ui->qCustomPlot->yAxis->setRange(0,max_tension[0]*1.1);
    ui->qCustomPlot->replot();

    ui->qCustomPlot2->graph(0)->setData(time_x_tension,tension_y2);
    ui->qCustomPlot2->graph(0)->setPen(pen);
    ui->qCustomPlot2->xAxis->setRange(shift_x_tension,receive_count_tension*0.05);
    ui->qCustomPlot2->yAxis->setRange(0,max_tension[1]*1.1);
    ui->qCustomPlot2->replot();

    ui->qCustomPlot3->graph(0)->setData(time_x_tension,tension_y3);
    ui->qCustomPlot3->graph(0)->setPen(pen);
    ui->qCustomPlot3->xAxis->setRange(shift_x_tension,receive_count_tension*0.05);
    ui->qCustomPlot3->yAxis->setRange(0,max_tension[2]*1.1);
    ui->qCustomPlot3->replot();

    ui->qCustomPlot4->graph(0)->setData(time_x_tension,tension_y4);
    ui->qCustomPlot4->graph(0)->setPen(pen);
    ui->qCustomPlot4->xAxis->setRange(shift_x_tension,receive_count_tension*0.05);
    ui->qCustomPlot4->yAxis->setRange(0,max_tension[3]*1.1);
    ui->qCustomPlot4->replot();

    ui->qCustomPlot5->graph(0)->setData(time_x_tension,tension_y5);
    ui->qCustomPlot5->graph(0)->setPen(pen);
    ui->qCustomPlot5->xAxis->setRange(0,receive_count_tension*0.05);
    ui->qCustomPlot5->yAxis->setRange(0,max_tension[4]*1.1);
    ui->qCustomPlot5->replot();

    ui->qCustomPlot6->graph(0)->setData(time_x_tension,tension_y6);
    ui->qCustomPlot6->graph(0)->setPen(pen);
    ui->qCustomPlot6->xAxis->setRange(0,receive_count_tension*0.05);
    ui->qCustomPlot6->yAxis->setRange(0,max_tension[5]*1.1);
    ui->qCustomPlot6->replot();

    // Elbow and Shoulder angle figure
    ui->qCustomPlot7->graph(0)->setData(time_x_angle,elbow_x);
    ui->qCustomPlot7->graph(0)->setPen(pen);
    ui->qCustomPlot7->xAxis->setRange(shift_x_angle,(receive_count_angle-1)*0.05);
    ui->qCustomPlot7->yAxis->setRange(-120,120);
    ui->qCustomPlot7->replot();

    ui->qCustomPlot8->graph(0)->setData(time_x_angle,elbow_y);
    ui->qCustomPlot8->graph(0)->setPen(pen);
    ui->qCustomPlot8->xAxis->setRange(shift_x_angle,(receive_count_angle-1)*0.05);
    ui->qCustomPlot8->yAxis->setRange(-120,120);
    ui->qCustomPlot8->replot();

    ui->qCustomPlot9->graph(0)->setData(time_x_angle,elbow_z);
    ui->qCustomPlot9->graph(0)->setPen(pen);
    ui->qCustomPlot9->xAxis->setRange(shift_x_angle,(receive_count_angle-1)*0.05);
    ui->qCustomPlot9->yAxis->setRange(-120,120);
    ui->qCustomPlot9->replot();

    ui->qCustomPlot10->graph(0)->setData(time_x_angle,shoulder_x);
    ui->qCustomPlot10->graph(0)->setPen(pen);
    ui->qCustomPlot10->xAxis->setRange(shift_x_angle,(receive_count_angle-1)*0.05);
    ui->qCustomPlot10->yAxis->setRange(-120,120);
    ui->qCustomPlot10->replot();

    ui->qCustomPlot11->graph(0)->setData(time_x_angle,shoulder_y);
    ui->qCustomPlot11->graph(0)->setPen(pen);
    ui->qCustomPlot11->xAxis->setRange(shift_x_angle,(receive_count_angle-1)*0.05);
    ui->qCustomPlot11->yAxis->setRange(-120,120);
    ui->qCustomPlot11->replot();

    ui->qCustomPlot12->graph(0)->setData(time_x_angle,shoulder_z);
    ui->qCustomPlot12->graph(0)->setPen(pen);
    ui->qCustomPlot12->xAxis->setRange(shift_x_angle,(receive_count_angle-1)*0.05);
    ui->qCustomPlot12->yAxis->setRange(-120,120);
    ui->qCustomPlot12->replot();

    // Elbow surface pressure figure
    ui->elbpresPlot->graph(0)->setData(time_x_surpressure,surpressure_elbow);
    ui->elbpresPlot->xAxis->setRange(shift_x_pressure,receive_count_pressure*0.05);
    ui->elbpresPlot->yAxis->setRange(0,5);
    ui->elbpresPlot->replot();

    // Shoulder surface pressure figure
    ui->shopresPlot->graph(0)->setData(time_x_surpressure,surpressure_shou1);
    ui->shopresPlot->graph(1)->setData(time_x_surpressure,surpressure_shou2);
    ui->shopresPlot->xAxis->setRange(shift_x_pressure,receive_count_pressure*0.05);
    ui->shopresPlot->yAxis->setRange(0,5);
    ui->shopresPlot->replot();


//    /********1**********/
//    ui->Motor1Plot->graph(0)->setData(time_x,motoangle_1);
//    ui->Motor1Plot->xAxis->setRange(0,time_x[receive_count_angle]);
//    ui->Motor1Plot->yAxis->setRange(0,max_motor_angle[0]*1.1);
//    ui->Motor1Plot->replot(QCustomPlot::rpQueuedReplot);

//    /********2**********/
//    ui->Motor2Plot->graph(0)->setData(time_x,motoangle_2);
//    ui->Motor2Plot->xAxis->setRange(0,time_x[receive_count_angle]);
//    ui->Motor2Plot->yAxis->setRange(0,max_motor_angle[1]*1.1);
//    ui->Motor2Plot->replot(QCustomPlot::rpQueuedReplot);

//    /********3**********/
//    ui->Motor3Plot->graph(0)->setData(time_x,motoangle_3);
//    ui->Motor3Plot->xAxis->setRange(0,time_x[receive_count_angle]);
//    ui->Motor3Plot->yAxis->setRange(0,max_motor_angle[2]*1.1);
//    ui->Motor3Plot->replot(QCustomPlot::rpQueuedReplot);

//    /********4**********/
//    ui->Motor4Plot->graph(0)->setData(time_x,motoangle_4);
//    ui->Motor4Plot->xAxis->setRange(0,time_x[receive_count_angle]);
//    ui->Motor4Plot->yAxis->setRange(0,max_motor_angle[3]*1.1);
//    ui->Motor4Plot->replot(QCustomPlot::rpQueuedReplot);

//    /********5**********/
//    ui->Motor5Plot->graph(0)->setData(time_x,motoangle_5);
//    ui->Motor5Plot->xAxis->setRange(0,time_x[receive_count_angle]);
//    ui->Motor5Plot->yAxis->setRange(0,max_motor_angle[4]*1.1);
//    ui->Motor5Plot->replot(QCustomPlot::rpQueuedReplot);

//    /********6**********/
//    ui->Motor6Plot->graph(0)->setData(time_x,motoangle_6);
//    ui->Motor6Plot->xAxis->setRange(0,time_x[receive_count_angle]);
//    ui->Motor6Plot->yAxis->setRange(0,max_motor_angle[5]*1.1);
//    ui->Motor6Plot->replot(QCustomPlot::rpQueuedReplot);
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


void MainWindow::on_pushButton_clicked()
{
    // 预紧信号发送
    emit sigBeforeTigh();
}

void MainWindow::on_VRDisplay_clicked()
{
    emit sigVRSerialOpen();
}

void MainWindow::on_sendmsgButton_clicked()
{
    uint TensionOrAngle=0;
    int sendData[10];
    // JOINNT CONTROL MODE
    if(ui->joint_RadioButton->isChecked())
    {
        TensionOrAngle = 1;
        sendData[0] = ui->sendMsgLineEdit7->text().toInt();
        sendData[1] = ui->sendMsgLineEdit8->text().toInt();
        sendData[2] = ui->sendMsgLineEdit9->text().toInt();
        sendData[3] = ui->sendMsgLineEdit10->text().toUInt();
        sendData[4] = ui->vellineEdit->text().toUInt();
        sendData[5] = ui->acclineEdit->text().toUInt();
        sendData[6] = ui->circlelineEdit->text().toUInt();
        emit sigSerialCtrl(TensionOrAngle, sendData);
        //motorcontrol->start();
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
        emit sigSerialCtrl(TensionOrAngle, sendData);
        //motorcontrol->start();
    }
    // PTP CONTROL MODE
    else if(ui->Position_RadioButton->isChecked())
    {
        TensionOrAngle = 2;
        sendData[0] = ui->PosXlineEdit->text().toInt();
        sendData[1] = ui->PosYlineEdit->text().toInt();
        sendData[2] = ui->PosZlineEdit->text().toInt();
        sendData[3] = ui->vellineEdit->text().toUInt();
        sendData[4] = ui->acclineEdit->text().toUInt();
        emit sigSerialCtrl(TensionOrAngle, sendData);
    }
    else if(ui->Linear_RadioButton->isChecked())
    {
        TensionOrAngle = 3;
        emit sigSerialCtrl(TensionOrAngle, sendData);
    }
    else
    {
        QMessageBox::critical(this,tr("wrong operation"),tr("choose one category first!!!"),QMessageBox::Ok);
        return;
    }
}

void MainWindow::on_actionStartMeasure_triggered()
{
    receive_count_tension = 0;
    receive_count_angle = 0;
    receive_count_pressure = 0;
    getsensordata->start();
    plot_timer->start(plot_timerdly);
}

void MainWindow::on_actionStopMeasure_triggered()
{
    getsensordata->terminate();
    getsensordata->wait();
    ui->actionSave->setEnabled(true);
    plot_timer->stop();
}
