#ifndef ICPDIALOG_H
#define ICPDIALOG_H

#include <QDialog>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include "pipe.h"
#include "icpcomputer.h"
namespace Ui {
class ICPDialog;
}

class ICPDialog : public QDialog
{
    Q_OBJECT
public:
    typedef std::shared_ptr<pcl::visualization::PCLVisualizer> VisualizerPtr;
    typedef pcl::visualization::PCLVisualizer Visualizer;
    typedef std::shared_ptr<QVTKWidget> QVTKWidgetPtr;
    class ViewPort{
    public:
        FullPointCloud::Ptr cloud;
        QVTKWidgetPtr w;
        VisualizerPtr v;
        std::vector<int> _PickedIndex;
    };
    explicit ICPDialog(QWidget *parent = 0);
    void setA(FullPointCloud::Ptr a);
    void setB(FullPointCloud::Ptr b);
    void getT(Eigen::Matrix4f& T);
    void pick(const pcl::visualization::PointPickingEvent & event, void*);
    void key(const pcl::visualization::KeyboardEvent& event, void*);
    ~ICPDialog();
public slots:
    void reset();
protected slots:
    void startICP(bool);
    void finishICP();
protected:
    void showInput(void);
    void showResult(void);
private:
    Ui::ICPDialog *ui;
    ViewPort vA;
    ViewPort vB;
    ICPComputer compute;
    float _T[16];
};

#endif // ICPDIALOG_H
