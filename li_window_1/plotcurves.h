#ifndef PLOTCURVES_H
#define PLOTCURVES_H


#include <QThread>
#include "ui_mainwindow.h"
#include <QMainWindow>


class PlotCurves:public QThread
{
    Q_OBJECT

public:
    explicit PlotCurves(QObject *parent = 0);
private:
    void run();
};

#endif // PLOTCURVES_H
