#ifndef FEATUREVIEW_H
#define FEATUREVIEW_H

#include <QFrame>
#include "pipe.h"
#include <QVTKWidget.h>
#include <pcl/visualization/cloud_viewer.h>
namespace Ui {
class FeatureView;
}

class FeatureView : public QFrame
{
    Q_OBJECT
public:
    explicit FeatureView(QWidget *parent = 0);
    ~FeatureView();

    void updatePointCloud(bool updateCam=false);
    void reLoad(void);

signals:
    void passToMdi(QWidget*);

public slots:
    void passChildToMdi(QWidget*);

protected slots:
    void nextR(void);
    void lastR(void);
    void nextL(void);
    void lastL(void);
    void compare(void);


protected:
    void reLoadR(void);
    void reLoadL(void);
    void updatePointCloudR(bool updateCam=false);
    void updatePointCloudL(bool updateCam=false);

private:
    Ui::FeatureView *ui;

    QList<quint32> _FrameKeyList;
    QList<quint32> _IdMapKeyList;

    FullPointCloud::Ptr cloudL;

    FullPointCloud::Ptr cloudR;

    QString infoL;
    QString idinfoL;
    cv::Mat idmapL;
    std::vector<std::vector<int>> segmapL;
    unsigned int currentFrameL;

    QString infoR;
    QString idinfoR;
    cv::Mat idmapR;
    std::vector<std::vector<int>> segmapR;
    unsigned int currentFrameR;

    QVTKWidget widgetL;
    pcl::visualization::PCLVisualizer vL;

    QVTKWidget widgetR;
    pcl::visualization::PCLVisualizer vR;
};

#endif // FEATUREVIEW_H
