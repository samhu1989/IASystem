#include "icpmixedinitview.h"
#include "ui_icpmixedinitview.h"
#include "icp.h"
#include "pipe.h"

Computation::~Computation()
{
    terminate();
}

ICPMixedInitView::ICPMixedInitView(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::ICPMixedInitView)
{
    ui->setupUi(this);
    v.initCameraParameters();
    v.addCoordinateSystem(0.3);
    v.setBackgroundColor(1.0,1.0,1.0);
    ui->gridLayout_2->addWidget(&widget);
    connect(&compute,SIGNAL(finished()),this,SLOT(finishing()));
    connect(&compute,SIGNAL(terminated()),this,SLOT(terminating()));
    computed = false;
    widget.SetRenderWindow(v.getRenderWindow());
    widget.show();
    connect(ui->plane,SIGNAL(clicked()),this,SLOT(showPlane()));
    connect(ui->corr,SIGNAL(clicked()),this,SLOT(showPlaneCorrepondence()));
    connect(ui->cloud,SIGNAL(clicked()),this,SLOT(showPointCloud()));
    connect(ui->initT,SIGNAL(clicked()),this,SLOT(showInitTransform()));
    connect(ui->finalT,SIGNAL(clicked()),this,SLOT(showFinalTransform()));
}

ICPMixedInitView::~ICPMixedInitView()
{
    while(compute.isRunning())compute.exit();
    compute.deleteLater();
    delete ui;
}

void ICPMixedInitView::setA(FullPointCloud::Ptr cloud)
{
    _A = cloud;
    compute.setA(_A);
}

void ICPMixedInitView::setB(FullPointCloud::Ptr cloud)
{
    _B = cloud;
    compute.setB(_B);
}

void ICPMixedInitView::run(void)
{
    compute.start();
    ui->label->setText(tr("Running"));
}

void ICPMixedInitView::finishing(void)
{
    computed = true;
    QString time;
    time = time.sprintf("Used:%d(ms)",compute.getTimeMS());
    ui->label->setText(time);
    Pipe::inform("showing");
    showPointCloud(true);
}

void ICPMixedInitView::terminating(void)
{
    ui->label->setText(tr("Broken"));
}

void ICPMixedInitView::keyPressEvent(QKeyEvent *e)
{
    Pipe::inform("press");
    int key = e->key();
    switch(key)
    {
    case Qt::Key_1:
        showPointCloud();break;
    case Qt::Key_2:
        showPlane();break;
    case Qt::Key_3:
        showPlaneCorrepondence();break;
    default:
        showPointCloud();break;
    }
    e->accept();
}

void ICPMixedInitView::keyReleaseEvent(QKeyEvent *e)
{
    Pipe::inform("release");
    e->accept();
}

void ICPMixedInitView::showPointCloud(bool updateCam)
{
    if(!computed)return;
    FullPoint center;
    Pipe::getPCDCenter( _B , center );

    if(updateCam)v.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    FullPointCloud::Ptr transA(new FullPointCloud);
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    t(0,3) = 1.5;
    t(1,3) = 1.5;
    pcl::transformPointCloud<FullPoint>(*_A,*transA,t);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbA(transA);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbB(_B);

    v.removeAllShapes();
    v.removeAllPointClouds();
    Eigen::Affine3f coordt;
    coordt = t;
    v.addCoordinateSystem(0.3,coordt,0);

    v.addPointCloud<FullPoint>( transA , rgbA , "A" );
    v.addPointCloud<FullPoint>( _B , rgbB , "B" );

    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "A");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B");


}

void ICPMixedInitView::showPlane(void)
{
    if(!computed)return;
    Pipe::inform("showing plane");
    cv::Mat idA;
    cv::Mat idB;
    compute.init.getPlanesA(idA);
    compute.init.getPlanesB(idB);

    FullPointCloud::Ptr transA(new FullPointCloud);
    FullPointCloud::Ptr transB(new FullPointCloud);

    *transB = *_B;

    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    t(0,3) = 1.5;
    t(1,3) = 1.5;
    pcl::transformPointCloud<FullPoint>(*_A,*transA,t);

    Pipe::doFalseColorOnPCD(idA,transA);
    Pipe::doFalseColorOnPCD(idB,transB);

    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbA(transA);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbB(transB);

    v.removeAllShapes();
    v.removeAllPointClouds();
    Eigen::Affine3f coordt;
    coordt = t;
    v.addCoordinateSystem(0.3,coordt,0);


    v.addPointCloud<FullPoint>( transA , rgbA , "A" );
    v.addPointCloud<FullPoint>( transB , rgbB , "B" );

    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "A");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B");
}

void ICPMixedInitView::showPlaneCorrepondence(void)
{
    if(!computed)return;
    Pipe::inform("showing plane correspondence");
    cv::Mat idA;
    cv::Mat idB;

    FullPointCloud::Ptr pairA;
    FullPointCloud::Ptr pairB;

    FullPointCloud::Ptr pairAD;
    FullPointCloud::Ptr pairBD;

    compute.init.getMatchedPlanesA(idA);
    compute.init.getMatchedPlanesB(idB);
    compute.init.getPointsCorr(pairA,pairB);
    compute.init.getDerivedPointsCorr(pairAD,pairBD);

    FullPointCloud::Ptr transA(new FullPointCloud);
    FullPointCloud::Ptr transB(new FullPointCloud);

    *transB = *_B;

    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    t(0,3) = 1.5;
    t(1,3) = 1.5;
    pcl::transformPointCloud<FullPoint>(*_A,*transA,t);
    pcl::transformPointCloud<FullPoint>(*pairA,*pairA,t);
    pcl::transformPointCloud<FullPoint>(*pairAD,*pairAD,t);

    Pipe::doFalseColorOnPCD(idA,transA);
    Pipe::doFalseColorOnPCD(idB,transB);

    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbA(transA);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbB(transB);

    v.removeAllShapes();
    v.removeAllPointClouds();
    Eigen::Affine3f coordt;
    coordt = t;
    v.addCoordinateSystem(0.3,coordt,0);

    v.addPointCloud<FullPoint>( transA , rgbA , "A" );
    v.addPointCloud<FullPoint>( transB , rgbB , "B" );

    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "A");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B");

    if(pairA->size()!=pairB->size())
    {
        Pipe::error("The paired match points have different number !!!");
    }
    unsigned int idx;
    for(idx=0;idx<pairA->size();++idx)
    {
        std::stringstream ss_line;
        ss_line<<"line("<<idx<<")";
        v.addLine<FullPoint,FullPoint>(pairA->at(idx),pairB->at(idx),0,255,0,ss_line.str());
    }
    for(idx=0;idx<pairAD->size();++idx)
    {
        std::stringstream ss_line;
        ss_line<<"cline("<<idx<<")";
        v.addLine<FullPoint,FullPoint>(pairAD->at(idx),pairBD->at(idx),0,0,255,ss_line.str());
    }
}

void ICPMixedInitView::showInitTransform(void)
{
    if(!computed)return;
    Pipe::inform("showing transformation");

    FullPointCloud::Ptr transA(new FullPointCloud);
    FullPointCloud::Ptr transB(new FullPointCloud);

    pcl::transformPointCloud<FullPoint>(*_A,*transA,compute.init._T);
    *transB = *_B;

    Pipe::doFalseColorOnPCD(0,transA);
    Pipe::doFalseColorOnPCD(1,transB);

    v.removeCoordinateSystem();
    v.addCoordinateSystem(0.3);

    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbA(transA);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbB(transB);

    v.removeAllShapes();
    v.removeAllPointClouds();

    v.addPointCloud<FullPoint>( transA , rgbA , "A" );
    v.addPointCloud<FullPoint>( transB , rgbB , "B" );

    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "A");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B");

    Pipe::inform("transform:");
    std::cerr<<compute.init._T<<std::endl;
}

void ICPMixedInitView::showFinalTransform(void)
{
    if(!computed)return;
    Pipe::inform("showing transformation");

    FullPointCloud::Ptr transA(new FullPointCloud);
    FullPointCloud::Ptr transB(new FullPointCloud);

    pcl::transformPointCloud<FullPoint>(*_A,*transA,compute.icp._T);
    *transB = *_B;

    Pipe::doFalseColorOnPCD(0,transA);
    Pipe::doFalseColorOnPCD(1,transB);

    v.removeCoordinateSystem();
    v.addCoordinateSystem(0.3);

    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbA(transA);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbB(transB);

    v.removeAllShapes();
    v.removeAllPointClouds();

    v.addPointCloud<FullPoint>( transA , rgbA , "A" );
    v.addPointCloud<FullPoint>( transB , rgbB , "B" );

    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "A");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B");

    Pipe::inform("transform:");
    std::cerr<<compute.icp._T<<std::endl;
}

void Computation::run(void)
{
    t.setHMS(0,0,0);
    t.restart();
    init.setMaxPlaneNum(8);
    init(_A,_B);
    t_ms = t.elapsed();
    ICP::MAX_CORRESPONDENCE_DISTANCE_FOR_INIT = 0.075;
    icp._T = init._T;
    icp(_A,_B);
}

int Computation::getTimeMS(void)
{
    return t_ms;
}

