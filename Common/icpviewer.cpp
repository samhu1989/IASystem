#include "icpviewer.h"
#include "ui_icpviewer.h"
#include "pipe.h"
ICPViewer::ICPViewer(
        FullPointCloud::Ptr A,
        FullPointCloud::Ptr B,
        QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ICPViewer)
{
    ui->setupUi(this);
    setMinimumSize(320,240);
    v.initCameraParameters();
    v.addCoordinateSystem(0.3);
    v.setBackgroundColor(1.0,1.0,1.0);
    computor = new SParseComputor();
    computor->_A = A;
    computor->_B = B;
    timer = new QTimer();
    connect(timer,SIGNAL(timeout()),this,SLOT(compute()));
    ui->gridLayout->addWidget(&widget);
    widget.SetRenderWindow(v.getRenderWindow());
    widget.show();
    _Counter = 0;
    timer->setSingleShot(true);

}

ICPViewer::~ICPViewer()
{
    timer->stop();
    timer->deleteLater();
    computor->disconnect(this,SLOT(show()));
    while(computor->isRunning())computor->exit();
    computor->deleteLater();
    delete ui;
}

void ICPViewer::run()
{
    timer->start(1);
}

void ICPViewer::compute(void)
{
    disconnect(computor,SIGNAL(finished()),this,SLOT(show()));
    computor->start();
    connect(computor,SIGNAL(finished()),this,SLOT(show()));
}

void ICPViewer::show(void)
{
    FullPoint center;
    Pipe::getPCDCenter( computor->_B , center );
    if(_Counter==0)v.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    FullPointCloud::Ptr transA(new FullPointCloud);
    FullPointCloud::Ptr alignedA(new FullPointCloud);
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    t(0,3) = 1.5;
    t(1,3) = 1.5;
    pcl::transformPointCloud<FullPoint>(*computor->_A,*transA,t);
    pcl::transformPointCloud<FullPoint>(*computor->_A,*alignedA,computor->icp._T);

    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbA(transA);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbB(computor->_B);
    pcl::visualization::PointCloudColorHandlerCustom<FullPoint>rgbC(alignedA,0,0,255);

    v.removeAllShapes();
    v.removeAllPointClouds();
    Eigen::Affine3f coordt;
    coordt = t;
    v.removeCoordinateSystem();
    v.addCoordinateSystem(0.3,coordt,0);

    v.addPointCloud<FullPoint>( transA , rgbA , "A" );
    v.addPointCloud<FullPoint>( computor->_B , rgbB , "B" );
    v.addPointCloud<FullPoint>( alignedA , rgbC , "C" );

    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "A");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "B");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "C");
    widget.SetRenderWindow(v.getRenderWindow());
    widget.update();
    if( _Counter < 30 )
    {
        run();
        ++_Counter;
    }else{
        Pipe::inform("Result");
        std::cerr<<computor->icp._T<<std::endl;
    }
}

void SParseComputor::run()
{
    if(!isInitialized)
    {
        ICP::RotateAloneZ rotate;
        FullPoint cA,cB;
        Pipe::getPCDCenter(_A,cA);
        Pipe::getPCDCenter(_B,cB);
        rotate._T = Eigen::Matrix4f::Identity();
        rotate(_A,cA,_B,cB);
        icp._T = rotate._T;
        isInitialized = true;
    }
    icp(_A,_B);
}
