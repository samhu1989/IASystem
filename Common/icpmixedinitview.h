#ifndef ICPMIXEDINITVIEW_H
#define ICPMIXEDINITVIEW_H
#include <QFrame>
#include <QThread>
#include <QKeyEvent>
#include "common_global.h"
#include "icp.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <QTime>
namespace Ui {
class ICPMixedInitView;
}

class Computation:public QThread
{
    Q_OBJECT
public:
    ~Computation();
    void setA(FullPointCloud::Ptr cloud){_A = cloud;}
    void setB(FullPointCloud::Ptr cloud){_B = cloud;}
    int getTimeMS(void);
    ICP::ICPMixRandInit init;
    ICP::ICPGeneral icp;
protected:
    void run(void);
private:
    FullPointCloud::Ptr _A;
    FullPointCloud::Ptr _B;
    QTime t;
    int t_ms;
};

class COMMONSHARED_EXPORT ICPMixedInitView : public QFrame
{
    Q_OBJECT
public:
    explicit ICPMixedInitView(QWidget *parent = 0);
    void setA(FullPointCloud::Ptr cloud);
    void setB(FullPointCloud::Ptr cloud);
    void run(void);
    ~ICPMixedInitView();
public slots:
    void keyPressEvent(QKeyEvent*);
    void keyReleaseEvent(QKeyEvent*);
protected slots:
    void finishing(void);
    void terminating(void);
    void showPointCloud(bool updateCam=false);
    void showPlane(void);
    void showPlaneCorrepondence(void);
    void showInitTransform(void);
    void showFinalTransform(void);
private:
    Ui::ICPMixedInitView *ui;
    Computation compute;
    bool computed;
    QVTKWidget widget;
    pcl::visualization::PCLVisualizer v;
    FullPointCloud::Ptr _A;
    FullPointCloud::Ptr _B;
};

#endif // ICPMIXEDINITVIEW_H
