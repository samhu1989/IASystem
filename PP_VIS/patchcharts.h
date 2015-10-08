#ifndef PATCHCHARTS_H
#define PATCHCHARTS_H

#include <QFrame>
#include "pipe.h"
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
namespace Ui {
class PatchCharts;
}

class PatchCharts : public QFrame
{
    Q_OBJECT
public:
    explicit PatchCharts(QWidget *parent = 0);

    void setFrames(FullPointCloud::Ptr&,cv::Mat&,FullPointCloud::Ptr&,cv::Mat&);

    static void idmap2segmap(cv::Mat& idmap,std::vector<std::vector<int>>&segmap);

    void reLoadPatch(void);

    void updatePatch(void);

    ~PatchCharts();
signals:
    void passToMdi(QWidget*);

protected:

    void reLoadPatchL(void);
    void reLoadPatchR(void);

    void updatePatchL(void);
    void updatePatchR(void);
    void updatePair(void);

protected slots:
    void lastL(void);
    void lastR(void);
    void nextL(void);
    void nextR(void);
    void compare(void);
    void icp(void);
    void sicp(void);
    void changePair(bool);

private:
    Ui::PatchCharts *ui;

    FullPointCloud::Ptr cloudL;
    FullPointCloud::Ptr patchL;

    FullPointCloud::Ptr cloudR;
    FullPointCloud::Ptr patchR;

    cv::Mat idmapL;
    std::vector<std::vector<int>> segmapL;
    unsigned int currentPatchL;

    cv::Mat idmapR;
    std::vector<std::vector<int>> segmapR;
    unsigned int currentPatchR;

    std::vector<std::pair<int,int>>patchPair;

    QVTKWidget widgetL;
    pcl::visualization::PCLVisualizer vL;

    QVTKWidget widgetR;
    pcl::visualization::PCLVisualizer vR;
};

#endif // PATCHCHARTS_H
