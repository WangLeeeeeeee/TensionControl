#ifndef ABOUTDIALOG_H
#define ABOUTDIALOG_H

#include <QDialog>

namespace Ui {
class aboutdialog;
}

class aboutdialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit aboutdialog(QWidget *parent = 0);
    ~aboutdialog();
    
private slots:
    void on_close_pushButton_clicked();

private:
    Ui::aboutdialog *ui;
};

#endif // ABOUTDIALOG_H
