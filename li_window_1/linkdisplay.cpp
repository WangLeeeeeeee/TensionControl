#include "linkdisplay.h"


#include <QtDataVisualization/QValue3DAxis>
#include <QtDataVisualization/Q3DTheme>
#include <QtCore/qmath.h>

#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/q3dtheme.h>

using namespace QtDataVisualization;

const int numberOfItems = 3600;
const float curveDivider = 3.0f;
const int lowerNumberOfItems = 900;
const float lowerCurveDivider = 0.75f;

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

    QScatterDataArray *dataArray = new QScatterDataArray;
    dataArray->resize(m_itemCount);
    QScatterDataItem *ptrToDataArray = &dataArray->first();

    ptrToDataArray->setPosition(QVector3D(1.0f,1.0f,6.0f));
    ptrToDataArray++;
    ptrToDataArray->setPosition(QVector3D(1.0f,1.0f,3.0f));
    ptrToDataArray++;
    ptrToDataArray->setPosition(QVector3D(1.0f,2.0f,2.0f));
    ptrToDataArray++;

    m_graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);
}


