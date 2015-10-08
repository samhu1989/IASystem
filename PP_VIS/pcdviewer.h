#ifndef PCDVIEWER_H
#define PCDVIEWER_H
#include "pipe.h"
#include <pcl/visualization/cloud_viewer.h>
#include <QWidget>
#include <QVTKWidget.h>

namespace Ui {
class PCDViewer;
}

class PCDViewer : public QWidget
{
    Q_OBJECT

public:
    explicit PCDViewer(QWidget *parent = 0);
    ~PCDViewer();
    bool addClouds(const QList<quint32>&hashKeys,bool multiView);
    bool addClouds(const QStringList& pcdfile);
    pcl::visualization::PCLVisualizer v;
private:
    void addCloudsInMultiView(const QList<FullPointCloud::Ptr>&pcdList,const QStringList& nameList);
    void addCloudsInOneView(const QList<FullPointCloud::Ptr>&pcdList,const QStringList& nameList);
    Ui::PCDViewer *ui;
    QVTKWidget widget;
};

#endif // PCDVIEWER_H
