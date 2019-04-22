#include "modbus.h"
#include <QModbusRtuSerialMaster>

modbus::modbus()
{
}

modbus::~modbus()
{

}

//---------------------------------------
// Function: initial the com and connect the com with Modbus protocol
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotMdSerialInit()
{
    qDebug()<<"modbusInit"<<QThread::currentThreadId();
    modbusDevice = new QModbusRtuSerialMaster(this);
    if(!modbusDevice){
        qDebug()<<"Could not creat Modbus master!!!";
        emit sigModMessage("error","Could not create Modbus master");
    }
    if(!modbusDevice)
        return;
    if(modbusDevice->state() != QModbusDevice::ConnectedState){
        modbusDevice->setConnectionParameter(QModbusDevice::SerialPortNameParameter,"COM24");
        modbusDevice->setConnectionParameter(QModbusDevice::SerialBaudRateParameter,QSerialPort::Baud57600);
        modbusDevice->setConnectionParameter(QModbusDevice::SerialDataBitsParameter,QSerialPort::Data8);
        modbusDevice->setConnectionParameter(QModbusDevice::SerialParityParameter,QSerialPort::NoParity);
        modbusDevice->setConnectionParameter(QModbusDevice::SerialStopBitsParameter,QSerialPort::TwoStop);
        modbusDevice->setTimeout(100);
        modbusDevice->setNumberOfRetries(3);
        if(!modbusDevice->connectDevice()){
            qDebug()<<"Connect failed: "<<modbusDevice->errorString();
            emit sigModMessage("error","Connect failed: "+ modbusDevice->errorString());
        } else {
            qDebug()<<"Connect successful: "<<modbusDevice->errorString();
            emit sigModMessage("noerror","Connect successful: ");
        }
    } else{
        modbusDevice->disconnectDevice();
    }
}

//---------------------------------------
// Function: close modbus com
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::slotMdSerialClose()
{
    qDebug()<<"modbusClose"<<QThread::currentThreadId();
    if(modbusDevice){
    modbusDevice->disconnectDevice();
    delete modbusDevice;
    modbusDevice = nullptr;
    }
}

//---------------------------------------
// Function: read data of server address with Modbus protocol
// Input parameter: Seraddress: server address
//                  Startaddress: start address
//                  number: register number
// Output parameter: no
//---------------------------------------
void modbus::readModbus(uint Seraddress, int Startaddress, uint number) //定义读modbus函数
{
    if(!modbusDevice)
        return;

    QModbusDataUnit readUnit(QModbusDataUnit::HoldingRegisters,Startaddress,number);//发送的数据信息（数据类型 ，起始地址，个数）
    if(auto *reply = modbusDevice->sendReadRequest(readUnit, Seraddress)){
        if(!reply->isFinished())
            connect(reply, &QModbusReply::finished, this, &modbus::readReady);
        else
            delete reply;
    } else {
        emit sigModMessage("error","Read error: "+modbusDevice->errorString());
    }
}

//---------------------------------------
// Function: enter the function automatically after the read request is finished
// Input parameter: no
// Output parameter: no
//---------------------------------------
void modbus::readReady()
{
    auto reply = qobject_cast<QModbusReply *>(sender());
    if(!reply)
        return;

    if(reply->error() == QModbusDevice::NoError) {
        const QModbusDataUnit unit = reply->result();   //unit就是Modbus读的数据
        for(uint i = 0; i < unit.valueCount(); i++) {
            const QString entry = tr("Address: %1, Value: %2").arg(unit.startAddress() + i)
                                     .arg(QString::number(unit.value(i),
                                          unit.registerType() <= QModbusDataUnit::Coils ? 10 : 16));
            const QString Data = tr("%1").arg(QString::number(unit.value(i),
                                                                       unit.registerType() <= QModbusDataUnit::Coils ? 10 : 16));
            //qDebug()<<"read data is:"<<entry;   //显示数据entry
            emit sigModReadData(Data);
        }
    } else if(reply->error() == QModbusDevice::ProtocolError) {
        emit sigModMessage("error","Read response error: "+reply->errorString()+" Mobus exception:"+reply->rawResult().exceptionCode());
    } else {
        emit sigModMessage("error","Read response error: "+reply->errorString()+" code:"+reply->rawResult().exceptionCode());
    }

    reply->deleteLater();
}

//---------------------------------------
// Function: write data into server address with Modbus protocol
// Input parameter: Seraddress: server address
//                  Startaddress: start address
//                  Data: data(int type)
//                  writeLength: 0: write 16 function code(0x06), 1: write 32 function code(0x10)
//---------------------------------------
void modbus::writeModbus(uint Seraddress, int Startaddress, qint32 Data, bool writeLength)
{
    if(!modbusDevice)
        return;

    QModbusDataUnit writeUnit(QModbusDataUnit::HoldingRegisters,Startaddress,writeLength+1); //发送的数据信息（数据类型 ，起始地址，个数）0x030b

    int hex;
    if(writeLength == 0)
    {
        hex = Data;
        writeUnit.setValue(0,Data); // 相当于写单个寄存器
    }
    else {
        hex = Data>>16;
        writeUnit.setValue(1,hex); // 相当于写单个寄存器
        hex = Data;
        writeUnit.setValue(0,hex); // 相当于写单个寄存器
    }
    if(auto *reply = modbusDevice->sendWriteRequest(writeUnit, Seraddress)){// server address   sendWriteRequest是向服务器写数据
        if(!reply->isFinished()){ //reply Returns true when the reply has finished or was aborted.
            //if(reply->error() == QModbusDevice::NoError)
                //qDebug()<<"no error";

        } else {
            reply->deleteLater();
        }
    } else {
        emit sigModMessage("error","Write error: "+modbusDevice->errorString());
    }
}



