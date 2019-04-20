#include "saveexcelfile.h"
#include <windows.h>

QStringList NumToLetter = {"A2:A","B2:B","C2:C","D2:D","E2:E","F2:F","G2:G","H2:H","I2:I","J2:J","K2:K","L2:L",
                          "M2:M","N2:N","O2:O","P2:P","Q2:Q","R2:R","S2:S","T2:T","U2:U","V2:V","W2:W","X2:X","Y2:Y","Z2:Z"};

saveExcelFile::saveExcelFile()
{
}

void saveExcelFile::setPath(QString path)
{
    savePath = path;
}

void saveExcelFile::slotInitExcel()
{
    qDebug()<<"saveExcelFile slotInitExcel"<<QThread::currentThreadId();
    HRESULT r = OleInitialize(0);
    if (r != S_OK && r != S_FALSE)
    {
        qDebug()<<"Qt:初始化Ole失败";
        //qWarning("Qt:初始化Ole失败（error %x）",(unsigned int)r);
    }
}

void saveExcelFile::slotSaveFile()
{
    qDebug()<<"saveExcelFile slotSaveFile"<<QThread::currentThreadId();

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

    pWorkBooks->dynamicCall("Add");
    pWorkBook = pApplication->querySubObject("ActiveWorkBook");

    pSheets = pWorkBook->querySubObject("Sheets");
    pSheet = pSheets->querySubObject("Item(int)", 1);

    QAxObject *ten[6];
    for(int i=0; i<6; i++)
    {
        ten[i] = pSheet->querySubObject("Cells(int, int)", 1, i+1);
        ten[i]->dynamicCall("SetValue(const QVariant)", QVariant("ten"+QString::number(long(i+1))+"g"));
    }
    QAxObject *angle[6];
    for(int i=0; i<6; i++)
    {
        angle[i] = pSheet->querySubObject("Cells(int, int)", 1, i+7);
        switch(i){
        case 0:
            angle[i]->dynamicCall("SetValue(const QVariant&)", QVariant("elbow_z"));
            break;
        case 1:
            angle[i]->dynamicCall("SetValue(const QVariant&)", QVariant("elb_x"));
            break;
        case 2:
            angle[i]->dynamicCall("SetValue(const QVariant&)", QVariant("elb_y"));
            break;
        case 3:
            angle[i]->dynamicCall("SetValue(const QVariant&)", QVariant("sho_z"));
            break;
        case 4:
            angle[i]->dynamicCall("SetValue(const QVariant&)", QVariant("sho_x"));
            break;
        case 5:
            angle[i]->dynamicCall("SetValue(const QVariant&)", QVariant("sho_y"));
            break;
        }
    }
    QAxObject *encoder[6];
    for(int i=0; i<6; i++)
    {
        encoder[i] = pSheet->querySubObject("Cells(int, int)", 1, i+13);
        encoder[i]->dynamicCall("SetValue(const QVariant&)", QVariant("ten"+QString::number(long(i+1))+"g"));
    }
    QAxObject *emg[3];
    for(int i=0; i<3; i++)
    {
        emg[i] = pSheet->querySubObject("Cells(int, int)", 1, i+19);
        switch(i){
        case 0:
            emg[i]->dynamicCall("SetValue(const QVariant&)", QVariant("Emgvalue"));
            break;
        case 1:
            emg[i]->dynamicCall("SetValue(const QVariant&)", QVariant("ElbowAngle_emg"));
            break;
        case 2:
            emg[i]->dynamicCall("SetValue(const QVariant&)", QVariant("ElbowAngle_fit"));
            break;
        }
    }

    // Change one-dimension to two-dimension array
    // Using QVarient in excel
    QVariantList vars;
    QString RangeStr;
    unsigned int numAll = receive_count_tension*6+receive_count_angle*6+receive_count_mocount*6+emgsaveLen*2+fiteffRecord.size();
    unsigned int numNow = 0;

    qDebug()<<"numAll is:"<<numAll;
    // save tension data
    for(int i=0; i<6; i++)
    {
        for(unsigned int j=0; j<receive_count_tension; j++)
        {
            numNow++;
            qDebug()<<"numNow is:"<<numNow;
            emit sigSavingProcess(numNow*100/numAll,"saving");
            QList<QVariant> rows;
            switch(i){
            case 0:
                rows.append(tension_y[j]);
                break;
            case 1:
                rows.append(tension_y2[j]);
                break;
            case 2:
                rows.append(tension_y3[j]);
                break;
            case 3:
                rows.append(tension_y4[j]);
                break;
            case 4:
                rows.append(tension_y5[j]);
                break;
            case 5:
                rows.append(tension_y6[j]);
                break;
            default:
                break;
            }
            vars.append(QVariant(rows));
        }
        QVariant res(vars);
        RangeStr = NumToLetter[i] + QString::number(receive_count_tension+1);
        QAxObject *user_range = pSheet->querySubObject("Range(const QString&)",RangeStr);
        user_range->setProperty("Value",res);
        vars.clear();
    }

    qDebug()<<"angle data number is:"<<receive_count_angle;
    // save angle data
    for(int i=0; i<6; i++)
    {
        for(unsigned int j=0; j<receive_count_angle; j++)
        {
            numNow++;
            qDebug()<<"numNow is:"<<numNow;
            emit sigSavingProcess(numNow*100/numAll,"saving");
            QList<QVariant> rows;
            switch(i){
            case 0:
                rows.append(elbow_z[j]);
                break;
            case 1:
                rows.append(elbow_x[j]);
                break;
            case 2:
                rows.append(elbow_y[j]);
                break;
            case 3:
                rows.append(shoulder_z[j]);
                break;
            case 4:
                rows.append(shoulder_x[j]);
                break;
            case 5:
                rows.append(shoulder_y[j]);
                break;
            default:
                break;
            }
            vars.append(QVariant(rows));
        }
        QVariant res(vars);
        RangeStr = NumToLetter[i+6] + QString::number(receive_count_angle+1);
        QAxObject *user_range = pSheet->querySubObject("Range(const QString&)",RangeStr);
        user_range->setProperty("Value",res);
        vars.clear();
    }

    qDebug()<<"encoder data number is:"<<receive_count_mocount;
    // save encoder data
    for(int i=0; i<6; i++)
    {
        for(unsigned int j=0; j<receive_count_mocount; j++)
        {
            numNow++;
            qDebug()<<"numNow is:"<<numNow;
            emit sigSavingProcess(numNow*100/numAll,"saving");
            QList<QVariant> rows;
            switch(i){
            case 0:
                rows.append(Motor1Count[j]);
                break;
            case 1:
                rows.append(Motor2Count[j]);
                break;
            case 2:
                rows.append(Motor3Count[j]);
                break;
            case 3:
                rows.append(Motor4Count[j]);
                break;
            case 4:
                rows.append(Motor5Count[j]);
                break;
            case 5:
                rows.append(Motor6Count[j]);
                break;
            default:
                break;
            }
            vars.append(QVariant(rows));
        }
        QVariant res(vars);
        RangeStr = NumToLetter[i+12] + QString::number(receive_count_mocount+1);
        QAxObject *user_range = pSheet->querySubObject("Range(const QString&)",RangeStr);
        user_range->setProperty("Value",res);
        vars.clear();
    }
    // save emg data
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<emgsaveLen; j++)
        {
            numNow++;
            emit sigSavingProcess(numNow*100/numAll,"saving");
            QList<QVariant> rows;
            switch(i){
            case 0:
                rows.append(EmgDataSave[i]);
                break;
            case 1:
                rows.append(AngleElbow_emg[i]);
                break;
            default:
                break;
            }
            vars.append(QVariant(rows));
        }
        QVariant res(vars);
        RangeStr = NumToLetter[i+18] + QString::number(emgsaveLen+1);
        QAxObject *user_range = pSheet->querySubObject("Range(const QString&)",RangeStr);
        user_range->setProperty("Value",res);
        vars.clear();
    }
    for(int i=0; i<fiteffRecord.size(); i++)
    {
        numNow++;
        emit sigSavingProcess(numNow*100/numAll,"saving");
        QList<QVariant> rows;
        rows.append(fiteffRecord[i]);
        vars.append(QVariant(rows));
    }
    QVariant res(vars);
    RangeStr = NumToLetter[20] + QString::number(fiteffRecord.length()+1);
    QAxObject *user_range= pSheet->querySubObject("Range(const QString&)",RangeStr);
    user_range->setProperty("Value",res);
    vars.clear();

    //excel
    pWorkBook->dynamicCall("SaveAs(const QString &)", QDir::toNativeSeparators(savePath));
    pApplication->dynamicCall("Quit()");
    delete pApplication;

    emit sigSavingProcess(100,"finish");
}
