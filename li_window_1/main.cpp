#include <QApplication>

#include "mainwindow.h"
#include "myglwidget.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //QTextCodec::setCodecForTr(QTextCodec::codecForLocale());//处理中文
    MainWindow w;
    w.show();
    
    return a.exec();

}
