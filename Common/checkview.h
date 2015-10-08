#ifndef CHECKVIEW_H
#define CHECKVIEW_H
#include "pipe.h"
#include <QFrame>
#include <QVTKWidget.h>
#include <pcl/visualization/cloud_viewer.h>
#include <QTimer>
namespace Ui {
class CheckView;
}

class CheckView : public QFrame
{
    Q_OBJECT

public:
    explicit CheckView(QWidget *parent = 0);
    ~CheckView();
    void addPointCloud(FullPointCloud::Ptr& a,FullPointCloud::Ptr& b);
    void initT(const cv::Mat&_T);
    void getT(cv::Mat&_T);
    void setResult(bool*);

    void picking(const pcl::visualization::PointPickingEvent & event, void *);
    void key(const pcl::visualization::KeyboardEvent & event, void *);

protected slots:
    void reject(void);
    void accept(void);
    void load(void);
private:
    std::vector<int>_PickedIndex;
    FullPointCloud::Ptr _a;
    FullPointCloud::Ptr _b;
    bool loaded;
    Ui::CheckView *ui;
    QVTKWidget widget;
    bool* result;
    pcl::visualization::PCLVisualizer v;
    QTimer timer;
};

#endif // CHECKVIEW_H
