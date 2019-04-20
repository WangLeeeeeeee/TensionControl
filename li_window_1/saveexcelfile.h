#ifndef SAVEEXCELFILE_H
#define SAVEEXCELFILE_H

#include <QThread>
#include <QFile>
#include <QInputDialog>
#include <QStandardItemModel>
#include <QFileDialog>
#include <QInputDialog>
#include <QtCore>
#include <ActiveQt/QAxObject>
#include "imuread.h"
#include "tensionread.h"
#include "encoderread.h"
#include "emg_tcp.h"

class saveExcelFile: public QObject
{
    Q_OBJECT
public:
    saveExcelFile();
    void setPath(QString path);
public slots:
    void slotInitExcel();
    void slotSaveFile();
signals:
    void sigSavingProcess(int n, QString message);
private:
    QString savePath;
};

#endif // SAVEEXCELFILE_H
