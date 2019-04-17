#ifndef MODBUS_H
#define MODBUS_H

#include <QModbusDataUnit>
#include <QSerialPort>
#include <QThread>
#include <QDebug>

class QModbusClient;
class QModbusReply;


class modbus: public QObject
{
    Q_OBJECT

public:
    modbus();
    ~modbus();

private:
    QModbusDataUnit readRequest() const;
    QModbusDataUnit writeRequest() const;
    QModbusReply *lastRequest;
    QModbusClient *modbusDevice;

public slots:
    // about modbus read and write
    void readModbus(uint Seraddress, int Startaddress, uint number);
    void readReady();
    void writeModbus(uint Seraddress, int Startaddress, qint32 Data, bool writeLength);
    void slotMdSerialInit();
    void slotMdSerialClose();

signals:
    // send the read data
    void sigModReadData(QString data);
    void sigModMessage(QString kind, QString message);
};

#endif // MODBUS_H
