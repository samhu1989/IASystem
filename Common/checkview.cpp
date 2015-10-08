#include "checkview.h"
#include "ui_checkview.h"

CheckView::CheckView(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::CheckView)
{
    loaded = false;
    std::cerr<<"checking"<<std::endl;
    ui->setupUi(this);
    ui->widget->setMinimumHeight(240);
    ui->widget->setMinimumWidth(320);
    v.initCameraParameters();
    widget.SetRenderWindow(v.getRenderWindow());
    ui->gridLayout_2->addWidget(&widget);
    connect(ui->buttonBox,SIGNAL(accepted()),this,SLOT(accept()));
    connect(ui->buttonBox,SIGNAL(rejected()),this,SLOT(reject()));
    timer.setSingleShot(false);
    timer.start(30);
    connect(&timer,SIGNAL(timeout()),this,SLOT(load()));
    v.registerPointPickingCallback<CheckView>(&CheckView::picking,*this,NULL);
    v.registerKeyboardCallback<CheckView>(&CheckView::key,*this,NULL);
}

void CheckView::setResult(bool* merge)
{
    result = merge;
}

void CheckView::addPointCloud(FullPointCloud::Ptr &a, FullPointCloud::Ptr &b)
{
    _a = a;
    _b = b;
}

void CheckView::load(void)
{
    if(isVisible())
    {
        timer.stop();
        v.setBackgroundColor(1.0,1.0,1.0);
        Pipe::inform("a");
        pcl::visualization::PointCloudColorHandlerCustom<FullPoint> ac(_a,0,255,0);
        pcl::visualization::PointCloudColorHandlerCustom<FullPoint> bc(_b,255,0,0);
        v.addPointCloud<FullPoint> (_a,ac,"a");
        v.addPointCloud<FullPoint> (_b,bc,"b");
        Pipe::inform("b");
        v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"a");
        v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"b");
        v.addCoordinateSystem(0.3);
        Pipe::inform("c");
    }
}

void CheckView::reject(void)
{
    *result = false;
    close();
}

void CheckView::accept(void)
{
    *result = true;
    close();
}

void CheckView::picking(const pcl::visualization::PointPickingEvent & event, void *)
{
    unsigned int idx = _PickedIndex.size();
    if(idx>=Pipe::FalseColorNum)return;
    _PickedIndex.push_back(event.getPointIndex());
    FullPoint p = _a->at(_PickedIndex[idx]);
    QString name;
    name = name.sprintf("%d",idx);
    float r,g,b;
    r = float(Pipe::FalseColor[idx][0])/255.0;
    g = float(Pipe::FalseColor[idx][1])/255.0;
    b = float(Pipe::FalseColor[idx][2])/255.0;
    v.addSphere (p,0.05,r,g,b,name.toStdString());
}

void CheckView::key(const pcl::visualization::KeyboardEvent & event,void*)
{
    if(_PickedIndex.empty())return;
    if("Delete"==event.getKeySym()&&event.keyDown())
    {
        unsigned int idx = _PickedIndex.size()-1;
        QString name;
        name = name.sprintf("%d",idx);
        v.removeShape(name.toStdString());
        _PickedIndex.pop_back();
    }
}

CheckView::~CheckView()
{
    delete ui;
}
