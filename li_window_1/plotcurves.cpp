#include "plotcurves.h"

PlotCurves::PlotCurves(QObject *parent):QThread(parent)
{

}

void PlotCurves::run()
{
    qDebug()<<"GetSensordata Thread is running"<<endl;


    /*
    //  Six tension value figure
    ui->qCustomPlot->graph(0)->setData(time_x,tension_y);
    ui->qCustomPlot->xAxis->setRange(0,time_x[receive_count_tension-1]);
    ui->qCustomPlot->yAxis->setRange(0,max_tension[0]*1.1);
    ui->qCustomPlot->replot();

    ui->qCustomPlot2->graph(0)->setData(time_x,tension_y2);
    ui->qCustomPlot2->xAxis->setRange(0,time_x[receive_count_tension-1]);
    ui->qCustomPlot2->yAxis->setRange(0,max_tension[1]*1.1);
    ui->qCustomPlot2->replot();

    ui->qCustomPlot3->graph(0)->setData(time_x,tension_y3);
    ui->qCustomPlot3->xAxis->setRange(0,time_x[receive_count_tension-1]);
    ui->qCustomPlot3->yAxis->setRange(0,max_tension[2]*1.1);
    ui->qCustomPlot3->replot();

    ui->qCustomPlot4->graph(0)->setData(time_x,tension_y4);
    ui->qCustomPlot4->xAxis->setRange(0,time_x[receive_count_tension-1]);
    ui->qCustomPlot4->yAxis->setRange(0,max_tension[3]*1.1);
    ui->qCustomPlot4->replot();

    ui->qCustomPlot5->graph(0)->setData(time_x,tension_y5);
    ui->qCustomPlot5->xAxis->setRange(0,time_x[receive_count_tension-1]);
    ui->qCustomPlot5->yAxis->setRange(0,max_tension[4]*1.1);
    ui->qCustomPlot5->replot();

    ui->qCustomPlot6->graph(0)->setData(time_x,tension_y6);
    ui->qCustomPlot6->xAxis->setRange(0,time_x[receive_count_tension-1]);
    ui->qCustomPlot6->yAxis->setRange(0,max_tension[5]*1.1);
    ui->qCustomPlot6->replot();

    // Elbow and Shoulder angle figure
    ui->qCustomPlot7->graph(0)->setData(time_x,elbow_x);
    ui->qCustomPlot7->graph(1)->setData(time_x,elbow_y);
    ui->qCustomPlot7->graph(2)->setData(time_x,elbow_z);
    ui->qCustomPlot7->xAxis->setRange(0,time_x[receive_count_angle]);
    ui->qCustomPlot7->yAxis->setRange(-360,360);
    ui->qCustomPlot7->replot();

    ui->qCustomPlot8->graph(0)->setData(time_x,shoulder_x);
    ui->qCustomPlot8->graph(1)->setData(time_x,shoulder_y);
    ui->qCustomPlot8->graph(2)->setData(time_x,shoulder_z);
    ui->qCustomPlot8->xAxis->setRange(0,time_x[receive_count_angle]);
    ui->qCustomPlot8->yAxis->setRange(-360,360);
    ui->qCustomPlot8->replot();
    */


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
