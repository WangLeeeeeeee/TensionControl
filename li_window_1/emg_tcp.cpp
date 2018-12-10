#include "emg_tcp.h"
#include "tensioncontrol.h"

unsigned int len = 131072;

// 运动轨迹改成三次,拟合曲线外平滑不跳动
double theta0_aim = 0;
double thetaf_aim = 60.0;
double tf_aim = 100.0; // aim time is 5 second(50 100 millisecond)

// 保存拟合曲线数据
QVector<double> EmgDataSave(len),AngleElbow_emg(len);
unsigned int emgsaveLen=0;

// 自动触发end

// 给定阻力(三次曲线)
double resisforce0 = 0;
double resisforcef = 3000;
double resist_tf = 20.0;



double elbXRecord_emg[131072];
double elbYRecord_emg[131072];
double elbZRecord_emg[131072];
double shoXRecord_emg[131072];
double shoYRecord_emg[131072];
double shoZRecord_emg[131072];

//QVector<double> emgValue(len);
QVector<float> EMG_array;
int emgRecordCout = 0;
unsigned int imuRecordCout = 0;
unsigned int emgTension[6] = {0,0,0,0,0,0};
unsigned int emgdataLength=50;
unsigned int polyRank=6;
bool receiveFlag=0;
#define Dimension 6
double fitEfficient[20];
double prefitEfficient[20];
double elbowLastMax = 0;
double elbowLastMin = 0;

unsigned int triggerTimes = 1;

double testEmg[] = {0.344731, 0.159619, 0.306073, 0.286254, 0.228736, 0.235155, 0.458003, 0.298763, 0.235687, 0.192039, 0.12551, 0.473019, 0.236293, 0.374413, 0.393713, 0.197504,
                    0.282371, 0.313201, 0.326407, 0.235456, 0.271036, 0.249283, 0.192322, 0.530665, 0.290591, 0.377142, 0.411442, 0.217107, 0.269897, 0.227633, 0.191032, 0.216026,
                    0.146853, 0.301339, 0.146471, 0.204349, 0.274863, 0.217329, 0.262068, 0.24579, 0.20721, 0.190976, 0.296504, 0.259974, 0.181215, 0.189336, 0.203391, 0.279267,
                    0.187418, 0.382311, 0.258136, 0.220126, 0.141941, 0.140481, 0.29291, 0.153883, 0.245818, 0.233847, 0.222912, 0.246096, 0.223803, 0.28088, 0.244484, 0.212023,
                    0.234415, 0.244092, 0.357438, 0.31736, 0.408802, 0.246563, 0.318511, 0.0987493, 0.322341, 0.396747, 0.280388, 0.228273, 0.181881, 0.176906, 0.213039, 0.273367,
                    0.169204, 0.281492, 0.374456, 0.244883, 0.192444, 0.204607, 0.130491, 0.242702, 0.232386, 0.200275, 0.199444, 0.19295, 0.26489, 0.175278, 0.190291, 0.203473,
                    0.184774, 0.311246, 0.181699, 0.210783, 0.368878, 0.512448, 0.399662, 0.366619, 0.525112, 0.269603, 0.254797, 0.158603, 0.205359, 0.377271, 0.268316, 0.312777,
                    0.229514, 0.161434, 0.181419, 0.243081, 0.219598, 0.210365, 0.395148, 0.28908, 0.294276, 0.247334, 0.188729, 0.179799, 0.249513, 0.148772, 0.113894, 0.146938,
                    0.282617, 0.354944, 0.180716, 0.200501, 0.305416, 0.207999, 0.117714, 0.223987, 0.273832, 0.176523, 0.264365, 0.544943, 0.521902, 0.307544, 0.432358, 0.383538,
                    0.586857, 0.332084, 0.309473, 0.548178, 0.364684, 0.335736, 0.412654, 0.295429, 0.418774, 0.409565, 0.147892, 0.254448, 0.643609, 0.207097, 0.185592, 0.36606,
                    0.306824, 0.284178, 0.26691, 0.236364, 0.300839, 0.224807, 0.274863, 0.351969, 0.376821, 0.984971, 4.03802, 1.74009, 3.90983, 4.24756, 3.5245, 5.70064, 7.55111,
                    2.72675, 5.04777, 2.90977, 3.02484, 4.39166, 8.21854, 5.12621, 3.9313, 11.3895, 4.50738, 2.5739, 3.80595, 1.9274, 4.19134, 6.24308, 4.14435, 2.19956, 2.1014,
                    3.77594, 3.17981, 1.76888, 3.34698, 2.5085, 1.74021, 1.29471, 3.35379, 1.77639, 3.17397, 2.50247, 3.26302, 3.33224, 2.50925, 1.54507, 2.2317, 1.87708, 2.5258,
                    3.00631, 2.51149, 1.43393, 1.39217, 3.35474, 0.417006, 0.515441, 0.795508, 0.845376, 0.931696, 0.721622, 0.718091, 0.922774, 0.628863, 0.835155, 0.780653,
                    0.766671, 0.707866, 0.744509, 0.877445, 2.05472, 0.478301, 0.408316, 0.302332, 0.207425, 0.324557, 0.203646, 0.276764, 0.19385, 0.289339, 0.239589, 0.248389,
                    0.237429, 0.278936, 0.230798, 0.29605, 0.266685, 0.240365, 0.32312, 0.336612, 1.50534, 1.048, 2.11539, 4.74745, 2.54216, 2.74015, 2.57326, 2.5329, 1.69246,
                    3.31158, 1.55245, 2.09442, 5.06939, 1.90543, 0.862759, 0.827877, 1.78819, 0.703319, 0.49521, 0.329304, 0.483372, 0.380696, 0.375164, 0.27366, 0.449815, 0.378803,
                    0.270095, 0.287158, 0.387719, 0.357382, 0.31151, 0.332679, 0.310217, 0.299049, 0.346508, 0.236641, 0.355219, 0.345025, 0.297592, 0.287195, 0.360018, 0.326361,
                    0.250255, 0.309179,0.30647, 0.278444, 0.266897, 0.359687, 0.285739, 0.301662, 0.323365, 0.452064, 0.30958, 0.36651, 0.281447, 0.359799, 0.277404, 0.407353,
                    0.304236, 0.278785, 0.269434, 0.230355,0.586857, 0.332084, 0.309473, 0.548178, 0.364684, 0.335736, 0.412654, 0.295429, 0.418774, 0.409565, 0.147892, 0.254448,
                    0.306824, 0.284178, 0.26691, 0.236364, 0.300839, 0.224807, 0.274863, 0.351969, 0.376821, 0.984971, 4.03802, 1.74009, 3.90983, 4.24756, 3.5245, 5.70064, 7.55111,
                    2.72675, 5.04777, 2.90977, 3.02484, 4.39166, 8.21854, 5.12621, 3.9313, 11.3895, 4.50738, 2.5739, 3.80595, 1.9274, 4.19134, 6.24308, 4.14435, 2.19956, 2.1014,
                    3.77594, 3.17981, 1.76888, 3.34698, 2.5085, 1.74021, 1.29471, 3.35379, 1.77639, 3.17397, 2.50247, 3.26302, 3.33224, 2.50925, 1.54507, 2.2317, 1.87708, 2.5258,
                    3.00631, 2.51149, 1.43393, 1.39217, 3.35474, 0.417006, 0.515441, 0.795508, 0.845376, 0.931696, 0.721622, 0.718091, 0.922774, 0.628863, 0.835155, 0.780653,
                    0.766671, 0.707866, 0.744509, 0.877445, 2.05472, 0.478301, 0.408316, 0.302332, 0.207425, 0.324557, 0.203646, 0.276764, 0.19385, 0.289339, 0.239589, 0.248389,
                    0.237429, 0.278936, 0.230798, 0.29605, 0.266685, 0.240365, 0.32312, 0.336612, 1.50534, 1.048, 2.11539, 4.74745, 2.54216, 2.74015, 2.57326, 2.5329, 1.69246,
                    3.31158, 1.55245, 2.09442, 5.06939, 1.90543, 0.862759, 0.827877, 1.78819, 0.703319, 0.49521, 0.329304, 0.483372, 0.380696, 0.375164, 0.27366, 0.449815, 0.378803,
                    0.270095, 0.287158, 0.387719, 0.357382, 0.31151, 0.332679, 0.310217, 0.299049, 0.346508, 0.236641, 0.355219, 0.345025, 0.297592, 0.287195, 0.360018, 0.326361,
                    0.250255, 0.309179,0.30647, 0.278444, 0.266897, 0.359687, 0.285739, 0.301662, 0.323365, 0.452064, 0.30958, 0.36651, 0.281447, 0.359799, 0.277404, 0.407353};
double testImu[] = {0,1,2,3,5,6,8,9,10,11,12,11,12,11,13,15,16,17,20,24,26,27,28,29,28,27,26,29,30,34,35,36,37,38,39,40,36,38,39,56,54,46,47,48,49,50,51,54,55,56,57,58,59,62,63,64,62,61,58,65,67,68};


#define ParaBuffer(Buffer,Row,Col) (*(Buffer + (Row) * (SizeSrc + 1) + (Col)))



EMG_server::EMG_server(QObject* parent):QObject(parent)
{
    server = new QTcpServer(this);
    server->listen(QHostAddress::Any, PORT);   
    connect(server,SIGNAL(newConnection()),this, SLOT(server_New_Connect()));
    recordAngleTimer = new QTimer(this);
    connect(recordAngleTimer,SIGNAL(timeout()),this,SLOT(slotRecord()));
    posbackTimer = new QTimer(this);
    connect(posbackTimer,SIGNAL(timeout()),this,SLOT(slotposBack()));
    tenctrl = new TensionControl;
    connect(this, SIGNAL(sigEmgTensionctrl(uint*)), tenctrl, SLOT(slotEmgTenctrl(uint*)));
}


EMG_server::~EMG_server()
{
    server->close();
    server->deleteLater();
}


void EMG_server::Send_Data(const char *data)
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
    qDebug()<<"enter the read dat";
    QByteArray buffer;
    //QVector<float> EMG_array;
    char* buf_string;
    const char delim[] = " ,[]";
    char *p;
    //读取缓冲区数据
    buffer = socket->readAll();
    buf_string = buffer.data();//QByteArray to char
    p = strtok(buf_string, delim);//Segmentate the string
    if(EMG_array.size() != 0) // clear the last data
        EMG_array.clear();
    while(p) {
        EMG_array.append(atof(p));//Add float into Qvecto
        p = strtok(NULL, delim);
        //emgRecordCout++;
    }

    if(!buffer.isEmpty())
    {
        qDebug() <<  EMG_array;
    }

    //recordAngleTimer->stop();
    //posbackTimer->start(100);
    // find the max angle and the min angle of last record
    for(unsigned int i=0; i<imuRecordCout; i++)
    {
        if(elbYRecord_emg[i]>elbowLastMax)
            elbowLastMax = elbYRecord_emg[i];
        if(elbYRecord_emg[i]<elbowLastMin)
            elbowLastMin = elbYRecord_emg[i];
    }
    receiveFlag = 1;

    qDebug()<<"starting the fit";
    // 多项式拟合 构建emg = c0 + c1*theta + c2*theta^2 + c3*theta^3
    double *BufferY,*BufferX;
    BufferX = (double *)calloc(imuRecordCout , sizeof(double));
    BufferY = (double *)calloc(imuRecordCout , sizeof(double));
    qDebug()<<"imu record count is:"<<imuRecordCout;
    int sizenum;
    double P[Dimension+1];

    if(receiveFlag == 1)
    {
        emgRecordCout = EMG_array.size();
        qDebug()<<"emgRecordCout is"<<emgRecordCout;
        if(imuRecordCout < emgRecordCout)
        {
            // 从后向前截取一段肌电信号和角度记录值
            for(unsigned int i=0; i<imuRecordCout; i++)
            {
                BufferX[i] = elbYRecord_emg[i];
                BufferY[i] = 100*EMG_array[emgRecordCout-imuRecordCout+i];

                // save the emg and the angle data
                EmgDataSave[emgsaveLen] = BufferY[i];
                AngleElbow_emg[emgsaveLen] = BufferX[i];
                emgsaveLen = emgsaveLen + 1;
            }
            // using 1000 as a sepreate flag
            EmgDataSave[emgsaveLen] = 1000;
            AngleElbow_emg[emgsaveLen] = 1000;
            emgsaveLen = emgsaveLen + 1;

            sizenum = imuRecordCout; // data count
        }
        else
        {
            // 从后向前截取一段角度信号和角度记录值
            for(unsigned int i=0; i<emgRecordCout; i++)
            {
                BufferX[i] = elbYRecord_emg[imuRecordCout-emgRecordCout+i];
                BufferY[i] = 100*EMG_array[i];

                // save the emg and the angle data
                EmgDataSave[emgsaveLen] = BufferY[i];
                AngleElbow_emg[emgsaveLen] = BufferX[i];
                emgsaveLen = emgsaveLen + 1;
            }
            // using 1000 as a sepreate flag
            EmgDataSave[emgsaveLen] = 1000;
            AngleElbow_emg[emgsaveLen] = 1000;
            emgsaveLen = emgsaveLen + 1;

            sizenum = emgRecordCout; // data count
        }
        Cal((const double*)BufferX, (const double*)BufferY, sizenum, sizeof(P) / sizeof(double), (double*)P);
        for (int i=0;i<Dimension+1;i++)				//这里是升序排列，Matlab是降序排列
        {
            fitEfficient[i] = P[i];
            qDebug()<<"P"<<i<<"is"<<P[i];
        }
        for(int i=0; i<Dimension+1; i++)
        {
            fitEfficient[i] +=  prefitEfficient[i];
            prefitEfficient[i] = fitEfficient[i];
        }
        // test the fit data
        double *sum;
        sum = (double *)calloc(sizenum , sizeof(double));
        for(int j=0; j<sizenum; j++)
        {
            for(int i=0; i<Dimension+1; i++)
            {
                sum[j] += fitEfficient[i]*qPow(BufferX[j],Dimension-i);
            }
            qDebug()<<"sum"<<j<<"is:"<<sum[j];
        }
        unsigned int dimension = Dimension;
        emit sigEmgThetaFit(fitEfficient, BufferX, BufferY, dimension, sizenum);
    }
}


void EMG_server::socket_Disconnected()
{
    qDebug() << "Disconnected!";
}

void EMG_server::slotEmgStart()
{
    for(unsigned int i=0; i<imuRecordCout; i++)
    {
        elbYRecord_emg[i] = 0;
    }
    imuRecordCout = 0;
    recordAngleTimer->start(100);
    posbackTimer->stop();
}

void EMG_server::slotEmgTrigger()
{
    /*
    triggerTimes++;
    recordAngleTimer->stop();
    posbackTimer->start(100);
    // find the max angle and the min angle of last record
    for(unsigned int i=0; i<imuRecordCout; i++)
    {
        if(elbYRecord_emg[i]>elbowLastMax)
            elbowLastMax = elbYRecord_emg[i];
        if(elbYRecord_emg[i]<elbowLastMin)
            elbowLastMin = elbYRecord_emg[i];
    }
    receiveFlag = 1;
//    imuRecordCout = 46;
    qDebug()<<"starting the fit";
    // 多项式拟合 构建emg = c0 + c1*theta + c2*theta^2 + c3*theta^3
    double *BufferY,*BufferX;
    BufferX = (double *)calloc(imuRecordCout , sizeof(double));
    BufferY = (double *)calloc(imuRecordCout , sizeof(double));
    qDebug()<<"imu record count is:"<<imuRecordCout;
    qDebug()<<"the testEMG size is:"<<sizeof(testEmg) / sizeof(double);
    int i, sizenum;
    double P[Dimension+1];

    if(EMG_array.size() != 0)
        EMG_array.clear();
    for(int i=0; i<sizeof(testEmg) / sizeof(double); i++)
    {
        EMG_array.append(testEmg[i]);
        //EMG_array[i] = testEmg[i];
    }

    if(receiveFlag == 1)
    {
        emgRecordCout = EMG_array.size();
        qDebug()<<"emgRecordCout is"<<emgRecordCout;
        // 从后向前截取一段肌电信号和角度记录值
        /*
        for(int i=emgRecordCout-Datanum; i<emgRecordCout; i++)
        {
            BufferX[i] = elbYRecord_emg[i];
            BufferY[a] = EMG_array[i];
            a++;
        }
        */

    /*
        for(unsigned int i=0; i<imuRecordCout; i++)
        {
            BufferX[i] = elbYRecord_emg[i]; // just for test
            //BufferX[i] = testImu[i];
            BufferY[i] = triggerTimes*EMG_array[emgRecordCout-imuRecordCout+i];

            // save the emg and the angle data
            EmgDataSave[emgsaveLen] = BufferY[i];
            AngleElbow_emg[emgsaveLen] = BufferX[i];
            emgsaveLen = emgsaveLen + 1;
        }
        // using 1000 as a sepreate flag
        EmgDataSave[emgsaveLen] = 1000;
        AngleElbow_emg[emgsaveLen] = 1000;
        emgsaveLen = emgsaveLen + 1;

        //sizenum = sizeof(BufferX)/ sizeof(BufferX[0]);	//	拟合数据的维数
        sizenum = imuRecordCout;
         qDebug()<<"sizenum is"<<sizenum;
        Cal((const double*)BufferX, (const double*)BufferY, sizenum, sizeof(P) / sizeof(double), (double*)P);
        for (i=0;i<Dimension+1;i++)				//这里是升序排列，Matlab是降序排列
        {
            fitEfficient[i] = P[i];
            qDebug()<<"P"<<i<<"is"<<P[i];
        }

        // test the fit function
        //QVector<double> sum;
        for(int i=0; i<Dimension+1; i++)
        {
            fitEfficient[i] +=  prefitEfficient[i];
            prefitEfficient[i] = fitEfficient[i];
        }
        double *sum;
        sum = (double *)calloc(imuRecordCout , sizeof(double));
        for(int j=0; j<sizenum; j++)
        {
            for(int i=0; i<Dimension+1; i++)
            {
                // use the first and second data test
                sum[j] += fitEfficient[i]*qPow(BufferX[j],Dimension-i);
            }
            qDebug()<<"sum"<<j<<"is:"<<sum[j];
        }
        unsigned int dimension = Dimension;
        emit sigEmgThetaFit(fitEfficient, BufferX, BufferY, dimension, sizenum);
    }
    */
}

void EMG_server::slotRecord()
{
    qDebug()<<"imuRecord count is:"<<imuRecordCout;
    // RECORD THE ANGLE
    double a;
    a = -elbow_y[receive_count_angle];
    elbYRecord_emg[imuRecordCout] = a;
    elbXRecord_emg[imuRecordCout] = elbow_x[receive_count_angle];
    elbZRecord_emg[imuRecordCout] = elbow_z[receive_count_angle];
    shoXRecord_emg[imuRecordCout] = shoulder_x[receive_count_angle];
    shoYRecord_emg[imuRecordCout] = shoulder_y[receive_count_angle];
    shoZRecord_emg[imuRecordCout] = shoulder_z[receive_count_angle];

    // According the theta collected to calculate the emg value(based on the fit function)
    double fitTension=0;
    double theta;
    theta = elbYRecord_emg[imuRecordCout];
    //theta = testImu[imuRecordCout] + 1.11;
    if(theta == 0) // the result of qPow(0,n) seems infinite
        theta = 1;
    qDebug()<<"theta is"<<theta;

    // judge the angle if exceed the aim angle auto trigger
    if(theta > 59)
    {
        slotEmgTrigger();
        Send_Data("end");
        recordAngleTimer->stop();
        posbackTimer->start(100);
    }

    // when the theta is out of the range of last record, the fit result maybe strange, so set it to zero
    // the strategy mentioned up is not very well, so changed it to below(let the result be the same as the margin)
    if(theta>elbowLastMax)
        theta = elbowLastMax;
    if(theta<elbowLastMin)
        theta = elbowLastMin;
    for(int i=0; i<Dimension+1; i++)
    {
        // use the first and second data test
        fitTension += fitEfficient[i]*qPow(theta,Dimension-i);
        qDebug()<<"the fitEfficiet"<<i<<"is:"<<fitEfficient[i];
    }
    for(int i=0; i<4; i++)
        emgTension[i] = 300;
    if((theta>30)&&(theta<60))
    {
        emgTension[4] = 600+(theta-30)*150;
    }
    else
        emgTension[4] = 300;
    qDebug()<<"emgTension 4 is:"<<emgTension[4];
    /*
    // give a resist force
    double aimResistForce;
    double a0_re,a1_re,a2_re,a3_re;
    unsigned int t_re;
    a0_re = resisforce0;
    a1_re = 0.0;
    a2_re = 3*(resisforcef - resisforce0)/qPow(resist_tf,2);
    a3_re = -2*(resisforcef - resisforce0)/qPow(resist_tf,3);
    t_re = imuRecordCout;
    if(t_re < resist_tf)
    {
       aimResistForce = a0_re + a1_re*t_re + a2_re*qPow(t_re,2) + a3_re*qPow(t_re,3);
    }
    else
    {
        aimResistForce = resisforcef;
    }
    emgTension[4] = aimResistForce;
    */
    double aimElbowAngle; // cubic poly nomial theta
    double a0,a1,a2,a3;
    unsigned int t;
    a0 = theta0_aim;
    a1 = 0.0;
    a2 = 3*(thetaf_aim - theta0_aim)/qPow(tf_aim,2);
    a3 = -2*(thetaf_aim - theta0_aim)/qPow(tf_aim,3);
    t = imuRecordCout;
    if(t < tf_aim)
    {
        aimElbowAngle = a0 + a1*t + a2*qPow(t,2) + a3*qPow(t,3);
    }
    else
    {
        aimElbowAngle = thetaf_aim;
    }
    qDebug()<<"aim theta is:"<<aimElbowAngle;
    double detaTheta = 0;
    detaTheta = aimElbowAngle - theta;
    if(detaTheta < 0)
        detaTheta = -detaTheta;
    fitTension = 300+10*fitTension*(detaTheta); // asist tension equal emg multiply deta theta
    qDebug()<<"fitTension is:"<<fitTension;
    if(fitTension < 0)
    {
        qDebug()<<"the asist tension is less 0";
        emgTension[5] = 200;
    }
    else
    {
        emgTension[5] = fitTension;
    }
    if(emgTension[5] > 6500)
    {
        qDebug()<<"the asist tension is exceed 3000";
        emgTension[5] = 6500;
    }
    imuRecordCout++;

    // SET THE TENSION VALUE
    emit sigEmgTensionctrl(emgTension);

}

void EMG_server::slotposBack()
{
    // set all the tension to 100
    for(int i=0; i<6; i++)
        emgTension[i] = 300;

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
            ParaBuffer(Para, 0, i) += qPow(*(X + j), 2 * (SizeSrc - 1) - i);
    for (i = 1; i < SizeSrc; i++)
        for (ParaBuffer(Para, i, SizeSrc - 1) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, i, SizeSrc - 1) += qPow(*(X + j), SizeSrc - 1 - i);
    for (i = 0; i < SizeSrc; i++)
        for (ParaBuffer(Para, i, SizeSrc) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, i, SizeSrc) += (*(Y + j)) * qPow(*(X + j), SizeSrc - 1 - i);
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

void EMG_server::polyfit(int n,double x[],double y[],int poly_n,double p[])
{
    int i,j;
    double *tempx,*tempy,*sumxx,*sumxy,*ata;

    tempx = (double *)calloc(n , sizeof(double));
    sumxx = (double *)calloc((poly_n*2+1) , sizeof(double));
    tempy = (double *)calloc(n , sizeof(double));
    sumxy = (double *)calloc((poly_n+1) , sizeof(double));
    ata = (double *)calloc( (poly_n+1)*(poly_n+1) , sizeof(double) );
    for (i=0;i<n;i++)
    {
        tempx[i]=1;
        tempy[i]=y[i];
    }
    for (i=0;i<2*poly_n+1;i++)
    {
        for (sumxx[i]=0,j=0;j<n;j++)
        {
            sumxx[i]+=tempx[j];
            tempx[j]*=x[j];
        }
    }
    for (i=0;i<poly_n+1;i++)
    {
        for (sumxy[i]=0,j=0;j<n;j++)
        {
            sumxy[i]+=tempy[j];
            tempy[j]*=x[j];
        }
    }
    for (i=0;i<poly_n+1;i++)
    {
        for (j=0;j<poly_n+1;j++)
        {
            ata[i*(poly_n+1)+j]=sumxx[i+j];
        }
    }
    gauss_solve(poly_n+1,ata,p,sumxy);

    free(tempx);
    free(sumxx);
    free(tempy);
    free(sumxy);
    free(ata);
}

void EMG_server::gauss_solve(int n,double A[],double x[],double b[])
{
    int i,j,k,r;
    double max;
    for (k=0;k<n-1;k++)
    {
        max=fabs(A[k*n+k]);					// find maxmum
        r=k;
        for (i=k+1;i<n-1;i++)
        {
            if (max<fabs(A[i*n+i]))
            {
                max=fabs(A[i*n+i]);
                r=i;
            }
        }
        if (r!=k)
        {
            for (i=0;i<n;i++)		//change array:A[k]&A[r]
            {
                max=A[k*n+i];
                A[k*n+i]=A[r*n+i];
                A[r*n+i]=max;
            }
            max=b[k];                    //change array:b[k]&b[r]
            b[k]=b[r];
            b[r]=max;
        }

        for (i=k+1;i<n;i++)
        {
            for (j=k+1;j<n;j++)
                A[i*n+j]-=A[i*n+k]*A[k*n+j]/A[k*n+k];
            b[i]-=A[i*n+k]*b[k]/A[k*n+k];
        }
    }

    for (i=n-1;i>=0;x[i]/=A[i*n+i],i--)
    {
        for (j=i+1,x[i]=b[i];j<n;j++)
            x[i]-=A[i*n+j]*x[j];
    }
}
