#ifndef LINKDISPLAY_H
#define LINKDISPLAY_H


#include <QtDataVisualization/Q3DSurface>
#include <QtDataVisualization/QSurfaceDataProxy>
#include <QtDataVisualization/QHeightMapSurfaceDataProxy>
#include <QtDataVisualization/QSurface3DSeries>

#include <QtDataVisualization/q3dscatter.h>
#include <QtDataVisualization/qabstract3dseries.h>
//#include "getsensordata.h"

using namespace QtDataVisualization;


class LinkDisplay : public QObject
{

    Q_OBJECT
public:
    explicit LinkDisplay(Q3DScatter *link);
    ~LinkDisplay();

    void buildLink();

private:
    QVector3D randVector();
    Q3DScatter *m_graph;
    int m_fontSize;
    QAbstract3DSeries::Mesh m_style;
    bool m_smooth;
    int m_itemCount;
    float m_curveDivider;

};

#endif // LINKDISPLAY_H
