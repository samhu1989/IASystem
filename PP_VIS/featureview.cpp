#include "featureview.h"
#include "ui_featureview.h"
#include "patchcharts.h"

FeatureView::FeatureView(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::FeatureView)
{
    ui->setupUi(this);
    ui->widget->setMinimumSize(320,240);
    ui->gridLayout_2->addWidget((QWidget*)(&widgetL));
    vL.initCameraParameters();
    vL.addCoordinateSystem(0.3);
    vL.setBackgroundColor(1.0,1.0,1.0);

    ui->widget_2->setMinimumSize(320,240);
    ui->gridLayout_3->addWidget((QWidget*)(&widgetR));
    vR.initCameraParameters();
    vR.addCoordinateSystem(0.3);
    vR.setBackgroundColor(1.0,1.0,1.0);

    QString info;
    Pipe::loadData(_FrameKeyList,info,Pipe::_FrameListKey);
    Pipe::loadData(_IdMapKeyList,info,Pipe::_IdMapListKey);

    cloudL = FullPointCloud::Ptr(new FullPointCloud);
    cloudR = FullPointCloud::Ptr(new FullPointCloud);

    currentFrameL = 0;
    currentFrameR = 1;

    connect(ui->lastL,SIGNAL(clicked()),this,SLOT(lastL()));
    connect(ui->lastR,SIGNAL(clicked()),this,SLOT(lastR()));
    connect(ui->nextL,SIGNAL(clicked()),this,SLOT(nextL()));
    connect(ui->nextR,SIGNAL(clicked()),this,SLOT(nextR()));
    connect(ui->toolButton_2,SIGNAL(clicked()),this,SLOT(compare()));
}

void FeatureView::updatePointCloud(bool updateCam)
{
    updatePointCloudL(updateCam);
    updatePointCloudR(updateCam);
}

void FeatureView::updatePointCloudR(bool updateCam)
{
    FullPoint center;
    Pipe::getPCDCenter( cloudR , center );

    if(updateCam)vR.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbR(cloudR);
    vR.removeAllPointClouds();
    vR.addPointCloud<FullPoint>( cloudR , rgbR , infoR.toStdString() );
    vR.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, infoR.toStdString());

    widgetR.SetRenderWindow(vR.getRenderWindow());
    widgetR.update();
}

void FeatureView::updatePointCloudL(bool updateCam)
{
    FullPoint center;
    Pipe::getPCDCenter( cloudL , center );

    if(updateCam)vL.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbL(cloudL);
    vL.removeAllPointClouds();
    vL.addPointCloud<FullPoint>( cloudL , rgbL , infoL.toStdString() );
    vL.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, infoL.toStdString());

    widgetL.SetRenderWindow(vL.getRenderWindow());
    widgetL.update();
}

void FeatureView::reLoad(void)
{
    reLoadR();
    reLoadL();
}


void FeatureView::reLoadR(void)
{
    if(Pipe::_PipeData.empty())return;
    currentFrameR %= _FrameKeyList.size();
    cloudR->clear();
    Pipe::loadData( cloudR , infoR , _FrameKeyList[currentFrameR] );
    Pipe::loadData( idmapR,idinfoR,_IdMapKeyList[currentFrameR]);
    ui->lineEdit_2->setText(infoR);
}

void FeatureView::reLoadL(void)
{
    if(Pipe::_PipeData.empty())return;
    currentFrameL %= _FrameKeyList.size();
    cloudL->clear();
    Pipe::loadData( cloudL , infoL , _FrameKeyList[currentFrameL] );
    Pipe::loadData( idmapL,idinfoL,_IdMapKeyList[currentFrameL]);
    ui->lineEdit->setText(infoL);
}

void FeatureView::nextR(void)
{
    currentFrameR++;
    reLoadR();
    updatePointCloudR();
}

void FeatureView::lastR(void)
{
    if(currentFrameR==0)currentFrameR=_FrameKeyList.size();
    currentFrameR--;
    reLoadR();
    updatePointCloudR();
}

void FeatureView::nextL(void)
{
    currentFrameL++;
    reLoadL();
    updatePointCloudL();
}

void FeatureView::lastL(void)
{
    if(currentFrameL==0)currentFrameL=_FrameKeyList.size();
    currentFrameL--;
    reLoadL();
    updatePointCloudL();
}

void FeatureView::compare(void)
{
    PatchCharts* w = new PatchCharts();
    connect(w,SIGNAL(passToMdi(QWidget*)),this,SLOT(passChildToMdi(QWidget*)));
    w->setFrames(cloudL,idmapL,cloudR,idmapR);
    w->reLoadPatch();
    w->updatePatch();
    emit passToMdi((QWidget*)w);
}

void FeatureView::passChildToMdi(QWidget *w)
{
    emit passToMdi(w);
}

FeatureView::~FeatureView()
{
    delete ui;
}
