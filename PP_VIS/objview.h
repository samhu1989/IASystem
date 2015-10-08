#ifndef OBJVIEW_H
#define OBJVIEW_H

#include <QWidget>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <memory>
#include "geoobj.h"
namespace Ui {
class ObjView;
}

class ObjView : public QWidget
{
    Q_OBJECT
public:
    explicit ObjView(QWidget *parent = 0);
    explicit ObjView(GeoObj::Ptr ptr,QWidget *parent = 0);
    ~ObjView();
public slots:
    void acceptSelect(int);
    void acceptDelete(int);
signals:
    void select(int);
    void deSelect(int);
protected slots:
    void changeName(QString);
    void informSelect(bool);
private:
    Ui::ObjView *ui;
    std::shared_ptr<pcl::visualization::PCLVisualizer> v;
    GeoObj::Ptr _ObjPtr;
    QVTKWidget w;
};

#endif // OBJVIEW_H
