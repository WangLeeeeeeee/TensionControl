#include "linkdisplay.h"

#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/q3dtheme.h>
#include <QtCore/qmath.h>

using namespace QtDataVisualization;

const int numberOfItems = 3600;
const float curveDivider = 3.0f;
const int lowerNumberOfItems = 900;
const float lowerCurveDivider = 0.75f;
const float pi=3.1415926;

const float x_shou = 10.0f;
const float y_shou = 10.0f;
const float z_shou = 10.0f;
const float len_upp = 5.0f;
const float len_for = 5.0f;
float x_elbo = 0.0f;
float y_elbo = 0.0f;
float z_elbo = 0.0f;
float x_wris = 0.0f;
float y_wris = 0.0f;
float z_wris = 0.0f;


LinkDisplay::LinkDisplay(Q3DScatter *link)
      : m_graph(link),
        m_fontSize(40.0f),
        m_style(QAbstract3DSeries::MeshSphere),
        m_smooth(true),
        m_itemCount(lowerNumberOfItems),
        m_curveDivider(lowerCurveDivider)
{
    m_graph->activeTheme()->setType(Q3DTheme::ThemeQt);
    QFont font = m_graph->activeTheme()->font();
    font.setPointSize(m_fontSize);
    m_graph->activeTheme()->setFont(font);
    m_graph->setShadowQuality(QAbstract3DGraph::ShadowQualitySoftLow);
    m_graph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);

    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);
    series->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    series->setMeshSmooth(m_smooth);
    m_graph->addSeries(series);

    buildLink();
}

LinkDisplay::~LinkDisplay()
{
    delete m_graph;
}

void LinkDisplay::buildLink()
{
    m_graph->axisX()->setTitle("X");
    m_graph->axisY()->setTitle("Y");
    m_graph->axisZ()->setTitle("Z");

    m_graph->axisY()->setRange(-600.0f,100.0f);
    m_graph->axisZ()->setRange(-200.0f,200.0f);
    m_graph->axisX()->setRange(-200.0f,200.0f);

    /*
    QScatterDataArray *dataArray = new QScatterDataArray;
    dataArray->resize(m_itemCount);
    QScatterDataItem *ptrToDataArray = &dataArray->first();
    ptrToDataArray->setPosition(QVector3D(2.0f,6.0f,2.0f));
    ptrToDataArray++;
    ptrToDataArray->setPosition(QVector3D(2.0f,3.0f,2.0f));
    ptrToDataArray++;
    ptrToDataArray->setPosition(QVector3D(4.0f,2.0f,2.0f));
    ptrToDataArray++;

    m_graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);
    */
}

void LinkDisplay::adjustPos(double sh_x, double sh_y, double sh_z, double el_x, double el_y, double el_z)
{
    QScatterDataArray *dataArray = new QScatterDataArray;
    dataArray->resize(m_itemCount);
    QScatterDataItem *ptrToDataArray = &dataArray->first();
    float lenUpper = 0.3035*1000;
    float lenFore = 0.135*1000;
    float posLenUpp[10],posLenFor[10];

    for(int i=0; i<10; i++)
    {
        posLenUpp[i] = i*lenUpper/10;
        posLenFor[i] = i*lenFore/10;

        float posUppX = -posLenUpp[i]*(sin(sh_z)*sin(sh_x) - cos(sh_z)*cos(sh_x)*cos(sh_y - pi/2));
        float posUppY = posLenUpp[i]*(cos(sh_z)*sin(sh_x) + cos(sh_x)*cos(sh_y - pi/2)*sin(sh_z));
        float posUppZ = -posLenUpp[i]*cos(sh_x)*sin(sh_y - pi/2);
        ptrToDataArray->setPosition(QVector3D(-posUppY,-posUppZ,-posUppX));
        ptrToDataArray++;

        float posForX = (607*cos(sh_z)*cos(sh_x)*cos(sh_y - pi/2))/2 - (607*sin(sh_z)*sin(sh_x))/2 - posLenFor[i]*(cos(el_y)*(sin(sh_z)*sin(sh_x) - cos(sh_z)*cos(sh_x)*cos(sh_y - pi/2)) + cos(sh_z)*sin(el_y)*sin(sh_y - pi/2));
        float posForY = posLenFor[i]*(cos(el_y)*(cos(sh_z)*sin(sh_x) + cos(sh_x)*cos(sh_y - pi/2)*sin(sh_z)) - sin(sh_z)*sin(el_y)*sin(sh_y - pi/2)) + (607*cos(sh_z)*sin(sh_x))/2 + (607*cos(sh_x)*cos(sh_y - pi/2)*sin(sh_z))/2;
        float posForZ = - (607*cos(sh_x)*sin(sh_y - pi/2))/2 - posLenFor[i]*(cos(sh_y - pi/2)*sin(el_y) + cos(sh_x)*cos(el_y)*sin(sh_y - pi/2));
        ptrToDataArray->setPosition(QVector3D(-posForY,-posForZ,-posForX));
        ptrToDataArray++;
    }
    m_graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);
}


