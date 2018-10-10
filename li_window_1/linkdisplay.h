#ifndef LINKDISPLAY_H
#define LINKDISPLAY_H

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
    void adjustPos(double sh_x, double sh_y, double sh_z, double el_x, double el_y, double el_z);

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
