#ifndef ICPWIDGET_H
#define ICPWIDGET_H

#include <QFrame>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <opencv2/core/core.hpp>
#include "pipe.h"
#include "geoobj.h"
namespace Ui {
class ICPWidget;
}

class ICPWidget : public QFrame
{
    Q_OBJECT
    typedef enum{
        PICK_FRAME,
        PICK_PATCH,
    }State;
public:

    explicit ICPWidget(QWidget *parent = 0);
    ~ICPWidget();

    void init(void);
    void pick(const pcl::visualization::PointPickingEvent & event, void*);
    void key(const pcl::visualization::KeyboardEvent& event, void*);

signals:
    void currentObj(int);
    void deleteObj(int);

protected:
    void generateSeg(void);

protected slots:
    void reLoadFrameWithView(void);
    void reLoadFrame(void);
    void nextFrame(void);
    void lastFrame(void);
    void updateShow(bool updateCam=false);
    void changeState(int);

    void addObj(void);
    void delObj(void);

    void selectObj(int);
    void deselectObj(int);

    void icpObj(void);

    void outputObj(void);
    void outputObjBox(void);
    void outputObjModel(void);

    cv::Mat transformLayout(cv::Mat&layout,Eigen::Matrix4f&);

    void outputFrame(void);
    void outputFrame(
            unsigned int FrameIdx,
            const std::vector<cv::Mat>&layouts,
            const std::vector<unsigned int>&objIdx
         );

private:
    Ui::ICPWidget *ui;
    QVTKWidget widget;
    pcl::visualization::PCLVisualizer v;

    QList<quint32> _FrameKeyList;
    QList<quint32> _IdMapKeyList;

    FullPointCloud::Ptr frameCloud;
    FullPointCloud::Ptr segCloud;
    FullPointCloud::Ptr showCloud;

    QString info;
    QString idinfo;
    cv::Mat idmap;

    std::vector<GeoObj::Ptr> _ObjList;
    int currentObjIndex;
    std::vector<std::vector<int>> segmap;
    unsigned int currentFrame;

    std::vector<int> _PickedIndex;
    State currentState;
    std::string currentPath;

    std::vector<unsigned int> alignedObjIndex;
    std::vector<cv::Mat> alignedObjLayout;
};

#endif // ICPWIDGET_H
