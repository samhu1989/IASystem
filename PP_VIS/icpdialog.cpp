#include "icpdialog.h"
#include "ui_icpdialog.h"
#include <Eigen/Core>
ICPDialog::ICPDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ICPDialog)
{
    ui->setupUi(this);
    vA.w = QVTKWidgetPtr(new QVTKWidget());
    vB.w = QVTKWidgetPtr(new QVTKWidget());
    ui->LeftLayout->addWidget(vA.w.get());
    ui->RightLayout->addWidget(vB.w.get());
    connect(ui->reset,SIGNAL(clicked()),this,SLOT(reset()));
    connect(ui->icp,SIGNAL(clicked(bool)),this,SLOT(startICP(bool)));
    vA.v = VisualizerPtr(NULL);
    vB.v = VisualizerPtr(NULL);
}

void ICPDialog::setA(FullPointCloud::Ptr a)
{
    vA.cloud = a;
}

void ICPDialog::setB(FullPointCloud::Ptr b)
{
    vB.cloud = b;
}

void ICPDialog::getT(Eigen::Matrix4f& T)
{
    Eigen::Map<Eigen::Matrix4f> transform(_T);
    T = transform;
}

void ICPDialog::reset()
{
    Eigen::Map<Eigen::Matrix4f> transform(_T);
    transform = Eigen::Matrix4f::Identity();
    if(vA.v.get()==NULL)
    {
        vA.v = VisualizerPtr(new Visualizer());
        vA.v->registerPointPickingCallback<ICPDialog>(&ICPDialog::pick,*this,&vA);
        vA.v->registerKeyboardCallback<ICPDialog>(&ICPDialog::key,*this,&vA);
    }
    if(vB.v.get()==NULL)
    {
        vB.v = VisualizerPtr(new Visualizer());
        vB.v->registerPointPickingCallback<ICPDialog>(&ICPDialog::pick,*this,&vB);
        vB.v->registerKeyboardCallback<ICPDialog>(&ICPDialog::key,*this,&vB);
    }
    showInput();
}

void ICPDialog::startICP(bool checked)
{
    if(compute.isRunning())
    {
        if(!checked)
        {
            disconnect(ui->icp,SIGNAL(clicked(bool)),this,SLOT(startICP(bool)));
            ui->icp->setChecked(true);
            connect(ui->icp,SIGNAL(clicked(bool)),this,SLOT(startICP(bool)));
        }
    }else{
        if(checked)
        {
            compute.setA(vA.cloud);
            compute.setB(vB.cloud);
            compute.setCorr(vA._PickedIndex,vB._PickedIndex);
            vA.v->removeAllShapes();
            vB.v->removeAllShapes();
            vA._PickedIndex.clear();
            vB._PickedIndex.clear();
            connect(&compute,SIGNAL(finished()),this,SLOT(finishICP()));
            compute.start();
        }
    }
}

void ICPDialog::finishICP()
{
    Eigen::Map<Eigen::Matrix4f> T(_T);
    connect(ui->icp,SIGNAL(clicked(bool)),this,SLOT(startICP(bool)));
    Eigen::Matrix4f t;
    compute.getT(t);
    T = t;
    ui->icp->setChecked(false);
    showResult();
}

void ICPDialog::pick(const pcl::visualization::PointPickingEvent & event, void* ptr)
{
    ViewPort &port = *(reinterpret_cast<ViewPort*>(ptr));
    if(port.cloud->empty())return;
    unsigned int idx = port._PickedIndex.size();
    idx%=Pipe::FalseColorNum;
    port._PickedIndex.push_back(event.getPointIndex());
    FullPoint p = port.cloud->at(port._PickedIndex[idx]);
    QString name;
    name = name.sprintf("%d",idx);
    float r,g,b;
    r = float(Pipe::FalseColor[idx][0])/255.0;
    g = float(Pipe::FalseColor[idx][1])/255.0;
    b = float(Pipe::FalseColor[idx][2])/255.0;
    port.v->addSphere (p,0.05,r,g,b,name.toStdString());
    port.w->update();
}

void ICPDialog::key(const pcl::visualization::KeyboardEvent& event, void* ptr)
{
    ViewPort &port = *(reinterpret_cast<ViewPort*>(ptr));
    if(port._PickedIndex.empty())return;
    if("Delete"==event.getKeySym()&&event.keyDown())
    {
        unsigned int idx = port._PickedIndex.size()-1;
        QString name;
        name = name.sprintf("%d",idx);
        port.v->removeShape(name.toStdString());
        port._PickedIndex.pop_back();
        port.w->update();
    }
}

void ICPDialog::showInput(void)
{
    FullPoint center;
    //show A
    Pipe::getPCDCenter( vA.cloud , center );
    vA.v->setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    vA.v->removeAllPointClouds();
    vA.v->addCoordinateSystem(0.3);
    vA.v->setBackgroundColor(1.0,1.0,1.0);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbA(vA.cloud);
    vA.v->addPointCloud<FullPoint>( vA.cloud , rgbA , "A" );
    vA.v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "A");

    vA.w->SetRenderWindow(vA.v->getRenderWindow());
    vA.w->update();
    //show B
    Pipe::getPCDCenter( vB.cloud , center );
    vB.v->setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    vB.v->removeAllPointClouds();
    vB.v->addCoordinateSystem(0.3);
    vB.v->setBackgroundColor(1.0,1.0,1.0);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbB(vB.cloud);
    vB.v->addPointCloud<FullPoint>( vB.cloud , rgbB , "B" );
    vB.v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B");

    vB.w->SetRenderWindow(vB.v->getRenderWindow());
    vB.w->update();
}

void ICPDialog::showResult(void)
{
    Eigen::Map<Eigen::Matrix4f> transform(_T);
    FullPointCloud::Ptr r(new FullPointCloud);
    pcl::transformPointCloudWithNormals<FullPoint>(*vA.cloud,*r,transform);
    pcl::visualization::PointCloudColorHandlerCustom<FullPoint> blue(r,0,0,255);
    vB.v->addPointCloud<FullPoint>(r,blue,"blueA");
    vB.v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "blueA");
    vB.w->SetRenderWindow(vB.v->getRenderWindow());
    vB.w->update();
}

ICPDialog::~ICPDialog()
{
    delete ui;
}
