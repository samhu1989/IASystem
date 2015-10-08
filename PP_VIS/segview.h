#ifndef SEGVIEW_H
#define SEGVIEW_H

#include <QWidget>
#include "pipe.h"
#include <QVTKWidget.h>
#include <pcl/visualization/cloud_viewer.h>
namespace Ui {
class SegView;
}

class SegView : public QWidget
{
    Q_OBJECT

public:
    explicit SegView(QWidget *parent = 0);
    ~SegView();
    void updatePointCloud(bool updateCam=false);

    void picking(const pcl::visualization::PointPickingEvent & event, void *);
    void key(const pcl::visualization::KeyboardEvent & event,void*);

public slots:
    void next(void);
    void last(void);
    void reSeg(void);

    void reset(void);
    void reLoad(void);
    void save(void);
protected:

    void updateSeg(void);
    void reSegPlane(void);
    void mergeSeg(void);
    void segToOutlier(void);
    void initPlane(pcl::ModelCoefficients::Ptr coeff,Eigen::Vector3f& center);
    FullPointCloud::Ptr extractPatchCloud(unsigned int segidx);
    FullPointCloud::Ptr extractPlaneCloud(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,Eigen::Vector3f&center);
    void reSegByPlane(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,std::vector<int>& reseg);
    void biCut(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,std::vector<int>& reseg);
    void regionCut(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,std::vector<int>& reseg);
    void updateIdMap(unsigned int idx,std::vector<int>& reseg);

private:
    std::vector<int> _PickedIndex;

    Ui::SegView *ui;
    QList<quint32> _FrameKeyList;
    QList<quint32> _IdMapKeyList;
    FullPointCloud::Ptr cloud;
    QString info;
    QString idinfo;
    cv::Mat idmap;
    std::vector<std::vector<int>> segmap;
    unsigned int currentFrame;

    QVTKWidget widget;
    pcl::visualization::PCLVisualizer v;
};

#endif // SEGVIEW_H
