#include "emg_tcp.h"


unsigned int len = 131072;
QVector<double> elbXRecord_emg(len),elbYRecord_emg(len),elbZRecord_emg(len);
QVector<double> shoXRecord_emg(len),shoYRecord_emg(len),shoZRecord_emg(len);
QVector<double> emgValue(len);
unsigned int emgRecordCout = 0;
unsigned int emgTension[6] = {0,0,0,0,0,0};
double fitEfficient[10];
unsigned int emgdataLength=50;
unsigned int polyRank=6;

#define ParaBuffer(Buffer,Row,Col) (*(Buffer + (Row) * (SizeSrc + 1) + (Col)))



EMG_server::EMG_server(QObject* parent):QObject(parent)
{
    server = new QTcpServer(this);
    server->listen(QHostAddress::Any, PORT);
    connect(server,SIGNAL(newConnection()),this, SLOT(server_New_Connect()));
    recordAngleTimer = new QTimer(this);
    connect(recordAngleTimer,SIGNAL(timeout()),this,SLOT(slotRecord()));
    tenctrl = new TensionControl();
    connect(this, SIGNAL(sigEmgTensionctrl(uint*)), tenctrl, SLOT(slotEmgTenctrl(uint*)));
}


EMG_server::~EMG_server()
{
    server->close();
    server->deleteLater();
}


void EMG_server::Send_Data(const char* data)
{
    socket->write(data);
    socket->flush();
}


void EMG_server::server_New_Connect()
{
    qDebug() << "A Client connect!";
    //根据客户端连接获取服务端socket对象
    socket = server->nextPendingConnection();
    //连接QTcpSocket的信号槽，以读取新数据
    connect(socket, &QTcpSocket::readyRead, this, & EMG_server::Read_Data);
    connect(socket, &QTcpSocket::disconnected, this, &EMG_server::socket_Disconnected);
}


void EMG_server::Read_Data()
{
    QByteArray buffer;
    //读取缓冲区数据
    buffer = socket->readAll();
    if(!buffer.isEmpty())
    {
        qDebug() << buffer;
    }
}


void EMG_server::socket_Disconnected()
{
    qDebug() << "Disconnected!";
}

void EMG_server::slotEmgStart()
{
    recordAngleTimer->start(100);

}

void EMG_server::slotEmgTrigger()
{
    qDebug()<<"starting the fit";
    // STOP RECORD
    //recordAngleTimer->stop();
    // 多项式拟合 构建emg = c0 + c1*theta + c2*theta^2 + c3*theta^3
    int Amount;
    //double BufferX[1024],BufferY[1024],ParaK[6];
    double ParaK[6];
    double BufferX[20] = {0.99,2.0,2.99,4.0,4.99,6.0,6.99,7.99,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0};
    double BufferY[20] = {-7.6,-2.4,108.7,625.0,2170.5,5814.5,13191.8,26622.0,49230.2,85066.5,139226.2,217970.1,328843.86,480798.4,684310.0,951500.9,1296255.9,1734346.6,2283552.12,2965776.5};
    Amount = 20;
    Cal((const double*)BufferX, (const double*)BufferY, Amount, sizeof(ParaK) / sizeof(double), (double*)ParaK);
    for(int i=0; i<sizeof(ParaK) / sizeof(double); i++)
        fitEfficient[i] = ParaK[i];
    for (Amount = 0; Amount < sizeof(ParaK) / sizeof(double); Amount++)
    {
        printf("ParaK[%d] = %lf\r\n", Amount, ParaK[Amount]);
        qDebug()<<"the par is:"<<ParaK[Amount];
    }
    /*
    int i,n;
    double xi,yi, chisq;
    gsl_matrix *X,*cov;
    gsl_vector *y,*c;
    n=200;
    double a = gsl_sf_bessel_J0(n);
    X = gsl_matrix_alloc(n,4);
    y = gsl_vector_alloc(n);

    c = gsl_vector_alloc(4);
    cov = gsl_matrix_alloc(4, 4);

    // test with some emg data
    for(int i=0; i<200; i++)
    {
        emgValue[i] = i*i/100+i*0.3+3;
        elbYRecord_emg[i] = i*0.3;
    }

    for(i=0; i<n; i++) // 读入N行数据（x,y,ei）,ei为y的方差
    {
        // y为emg, x为theta
        xi = elbYRecord_emg[i];
        yi = emgValue[i];
        gsl_matrix_set(X, i, 0, 1.0);
        gsl_matrix_set(X, i, 1, xi);
        gsl_matrix_set(X, i, 2, xi*xi);
        gsl_matrix_set(X, i, 3, xi*xi*xi);
        gsl_vector_set(y, i, yi);
    }
    gsl_multifit_linear_workspace* work = gsl_multifit_linear_alloc(n, 4);//使用拟合函数的第一步，申请运行函数的空间
    gsl_multifit_linear(X, y, c, cov, &chisq, work);
    gsl_multifit_linear_free(work);

#define C(i) (gsl_vector_get(c,(i)))
    qDebug()<<"the final equation is:"<<"emg ="<<C(0)<<"+"<<C(1)<<"theta +"<<C(2)<<"theta^2 +"<<C(3)<<"theta^3";

    for(int i=0; i<4; i++)
        fitEfficient[i] = C(i);// save the efficient for next run.

    gsl_matrix_free(X);
    gsl_vector_free(y);
    gsl_vector_free(c);
    gsl_matrix_free(cov);
    */
}

void EMG_server::slotRecord()
{
    // RECORD THE ANGLE
    elbXRecord_emg[emgRecordCout] = elbow_x[receive_count_angle];
    elbYRecord_emg[emgRecordCout] = -elbow_y[receive_count_angle];
    elbZRecord_emg[emgRecordCout] = elbow_z[receive_count_angle];
    shoXRecord_emg[emgRecordCout] = shoulder_x[receive_count_angle];
    shoYRecord_emg[emgRecordCout] = shoulder_y[receive_count_angle];
    shoZRecord_emg[emgRecordCout] = shoulder_z[receive_count_angle];
    // SET THE TENSION VALUE
    emit sigEmgTensionctrl(emgTension);
}

int EMG_server::PrintPara(double* Para, int SizeSrc)
{
    int i, j;
    for (i = 0; i < SizeSrc; i++)
    {
        for (j = 0; j <= SizeSrc; j++)
            printf("%10.6lf ", ParaBuffer(Para, i, j));
        printf("\r\n");
    }
    printf("\r\n");
    return 0;
}

int EMG_server::Cal(const double *BufferX, const double *BufferY, int Amount, int SizeSrc, double *ParaResK)
{
    double* ParaK = (double*)malloc(SizeSrc * (SizeSrc + 1) * sizeof(double));
    GetParaBuffer(ParaK, BufferX, BufferY, Amount, SizeSrc);
    ParaDeal(ParaK, SizeSrc);
    for (Amount = 0; Amount < SizeSrc; Amount++, ParaResK++)
        *ParaResK = ParaBuffer(ParaK, Amount, SizeSrc);
    free(ParaK);
    return 0;
}

int EMG_server::GetParaBuffer(double *Para, const double *X, const double *Y, int Amount, int SizeSrc)
{
    int i, j;
    for (i = 0; i < SizeSrc; i++)
        for (ParaBuffer(Para, 0, i) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, 0, i) += pow(*(X + j), 2 * (SizeSrc - 1) - i);
    for (i = 1; i < SizeSrc; i++)
        for (ParaBuffer(Para, i, SizeSrc - 1) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, i, SizeSrc - 1) += pow(*(X + j), SizeSrc - 1 - i);
    for (i = 0; i < SizeSrc; i++)
        for (ParaBuffer(Para, i, SizeSrc) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, i, SizeSrc) += (*(Y + j)) * pow(*(X + j), SizeSrc - 1 - i);
    for (i = 1; i < SizeSrc; i++)
        for (j = 0; j < SizeSrc - 1; j++)
            ParaBuffer(Para, i, j) = ParaBuffer(Para, i - 1, j + 1);
    return 0;
}

int EMG_server::ParaDeal(double *Para, int SizeSrc)
{
    PrintPara(Para, SizeSrc);
    Paralimit(Para, SizeSrc);
    PrintPara(Para, SizeSrc);
    if (ParaDealA(Para, SizeSrc))
        return -1;
    PrintPara(Para, SizeSrc);
    if (ParaDealB(Para, SizeSrc))
        return -1;
    return 0;
}

int EMG_server::ParaDealB(double *Para, int SizeSrc)
{
    int i;
    for (i = 0; i < SizeSrc; i++)
        if (ParaPreDealB(Para, SizeSrc, i))
            return -1;
    for (i = 0; i < SizeSrc; i++)
    {
        if (ParaBuffer(Para, i, i))
        {
            ParaBuffer(Para, i, SizeSrc) /= ParaBuffer(Para, i, i);
            ParaBuffer(Para, i, i) = 1.0;
        }
    }
    return 0;
}

int EMG_server::ParaPreDealB(double *Para, int SizeSrc, int OffSet)
{
    int i, j;
    for (i = OffSet + 1; i < SizeSrc; i++)
    {
        for (j = OffSet + 1; j <= i; j++)
            ParaBuffer(Para, i, j) *= ParaBuffer(Para, OffSet, OffSet);
        ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, OffSet, OffSet) - ParaBuffer(Para, i, OffSet) * ParaBuffer(Para, OffSet, SizeSrc);
        ParaBuffer(Para, i, OffSet) = 0;
        ParalimitRow(Para, SizeSrc, i);
    }
    return 0;
}

int EMG_server::ParaDealA(double* Para, int SizeSrc)
{
    int i;
    for (i = SizeSrc; i; i--)
        if (ParaPreDealA(Para, SizeSrc, i))
            return -1;
    return 0;
}

int EMG_server::ParaPreDealA(double* Para, int SizeSrc, int Size)
{
    int i, j;
    for (Size -= 1, i = 0; i < Size; i++)
    {
        for (j = 0; j < Size; j++)
            ParaBuffer(Para, i, j) = ParaBuffer(Para, i, j) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, j) * ParaBuffer(Para, i, Size);
        ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, SizeSrc) * ParaBuffer(Para, i, Size);
        ParaBuffer(Para, i, Size) = 0;
        ParalimitRow(Para, SizeSrc, i);
    }
    return 0;
}

int EMG_server::Paralimit(double* Para, int SizeSrc)
{
    int i;
    for (i = 0; i < SizeSrc; i++)
        if (ParalimitRow(Para, SizeSrc, i))
            return -1;
    return 0;
}

int EMG_server::ParalimitRow(double* Para, int SizeSrc, int Row)
{
    int i;
    double Max, Min, Temp;
    for (Max = abs(ParaBuffer(Para, Row, 0)), Min = Max, i = SizeSrc; i; i--)
    {
        Temp = abs(ParaBuffer(Para, Row, i));
        if (Max < Temp)
            Max = Temp;
        if (Min > Temp)
            Min = Temp;
    }
    Max = (Max + Min) * 0.000005;
    for (i = SizeSrc; i >= 0; i--)
        ParaBuffer(Para, Row, i) /= Max;
    return 0;
}

