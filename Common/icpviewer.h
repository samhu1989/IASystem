#ifndef ICPVIEWER_H
#define ICPVIEWER_H
#include <QWidget>
#include "common_global.h"
#include <QThread>
#include "icp.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <QTimer>
namespace Ui {
class ICPViewer;
}

class SParseComputor:public QThread
{
    Q_OBJECT
public:
    SParseComputor():QThread(),isInitialized(false){;}
    ICP::ICPSparse icp;
    FullPointCloud::Ptr _A;
    FullPointCloud::Ptr _B;
protected:
    void run(void);
private:
    bool isInitialized;
};

class COMMONSHARED_EXPORT ICPViewer : public QWidget
{
    Q_OBJECT
public:
    explicit ICPViewer(
            FullPointCloud::Ptr A,
            FullPointCloud::Ptr B,
             QWidget *parent = 0
            );
    ~ICPViewer();
    void run(void);
protected slots:
    void compute(void);
    void show(void);
private:
    QTimer* timer;
    SParseComputor* computor;
    Ui::ICPViewer *ui;
    QVTKWidget widget;
    pcl::visualization::PCLVisualizer v;
    int _Counter;
};

#endif // ICPVIEWER_H
