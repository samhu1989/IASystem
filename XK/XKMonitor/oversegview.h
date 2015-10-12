#ifndef OVERSEGVIEW_H
#define OVERSEGVIEW_H

#include <QWidget>
#include "pipe.h"
#include "xkcommon.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
namespace Ui {
class OverSegView;
}
using namespace XKCommon;
class OverSegView : public QWidget
{
    Q_OBJECT

public:
    explicit OverSegView(QWidget *parent = 0);
    ~OverSegView();
protected:
    void showSuperVoxel(
            SuperVoxelClusters&,
            SuperVoxelAdjacency&
            );
protected slots:
    void showFromFile(void);
    void showFromProc(void);
private:
    Ui::OverSegView *ui;
    QVTKWidget widget;
    pcl::visualization::PCLVisualizer v;
};

#endif // OVERSEGVIEW_H
