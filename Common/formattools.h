#ifndef FORMATTOOLS_H
#define FORMATTOOLS_H

#include <QFrame>
#include "common_global.h"
#include "pipe.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <memory>
#include <QFileInfo>
namespace Ui {
class FormatTools;
}

class COMMONSHARED_EXPORT FormatTools : public QFrame
{
    Q_OBJECT

public:
    typedef enum{
        REF,
        TARGET
    }State;
    explicit FormatTools(QWidget *parent = 0);
    ~FormatTools();
    void pick(const pcl::visualization::PointPickingEvent & event, void*);
    void key(const pcl::visualization::KeyboardEvent& event, void*);

protected slots:
    void load(void);
    void loadRef(void);
    void loadPCD( const std::string&fname,FullPointCloud::Ptr cloud,QFileInfo& info);
    void loadPLY( const std::string&fname,FullPointCloud::Ptr cloud,QFileInfo& info);
    void loadOBJ( const std::string&fname,FullPointCloud::Ptr cloud,QFileInfo& info);
    void showCloud(bool updateCam=false);
    void showAlign(void);
    void alignRef(void);
    void setZtoUp(void);
    void save(void);
    void remove(void);

private:
    Ui::FormatTools *ui;
    FullPointCloud::Ptr cloud;
    FullPointCloud::Ptr ref;
    QFileInfo cloudInfo;
    QFileInfo refInfo;
    std::shared_ptr<pcl::visualization::PCLVisualizer> v;
    std::shared_ptr<QVTKWidget> w;
    std::vector<int> _PickedIndex;
    std::vector<int> _PickedRef;
    std::vector<int> _Selected;
    State _State;
};

#endif // FORMATTOOLS_H
