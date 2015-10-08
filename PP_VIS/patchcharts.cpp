#include "patchcharts.h"
#include "ui_patchcharts.h"
#include "histogram.h"
#include "featuredialog.h"
#include "icpmixedinitview.h"
#include "icpviewer.h"
PatchCharts::PatchCharts(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::PatchCharts)
{
    ui->setupUi(this);
    currentPatchL = 0;
    currentPatchR = 0;
    patchL = FullPointCloud::Ptr(new FullPointCloud);
    patchR = FullPointCloud::Ptr(new FullPointCloud);
    ui->gridLayout_2->addWidget(&widgetL);
    ui->gridLayout_3->addWidget(&widgetR);
    ui->widget->setMinimumSize(320,240);
    ui->widget_2->setMinimumSize(320,240);

    vL.initCameraParameters();
    vL.addCoordinateSystem(0.3);
    vL.setBackgroundColor(1.0,1.0,1.0);

    vR.initCameraParameters();
    vR.addCoordinateSystem(0.3);
    vR.setBackgroundColor(1.0,1.0,1.0);

    connect(ui->lastL,SIGNAL(clicked()),this,SLOT(lastL()));
    connect(ui->lastR,SIGNAL(clicked()),this,SLOT(lastR()));
    connect(ui->nextL,SIGNAL(clicked()),this,SLOT(nextL()));
    connect(ui->nextR,SIGNAL(clicked()),this,SLOT(nextR()));
    connect(ui->toolButton_3,SIGNAL(clicked()),this,SLOT(compare()));
    connect(ui->toolButton,SIGNAL(clicked()),this,SLOT(icp()));
    connect(ui->toolButton_2,SIGNAL(clicked()),this,SLOT(sicp()));
}


void PatchCharts::setFrames(FullPointCloud::Ptr& fl,cv::Mat&idl,FullPointCloud::Ptr& fr,cv::Mat&idr)
{

    cloudL = fl;
    cloudR = fr;
    idmapL = idl;
    idmapR = idr;
    idmap2segmap(idmapL,segmapL);
    idmap2segmap(idmapR,segmapR);
}

void PatchCharts::idmap2segmap(cv::Mat& idmap,std::vector<std::vector<int>>&segmap)
{
    int segidx;
    int idx;
    for(idx=0;idx<idmap.cols;++idx)
    {
        segidx = idmap.at<int>(0,idx);
        if( segidx != -1  )
        {
            while( segidx > segmap.size() )
            {
                segmap.push_back(std::vector<int>());
            }
            segmap[segidx-1].push_back(idx);
        }
    }
}

void PatchCharts::reLoadPatch(void)
{
    reLoadPatchL();
    reLoadPatchR();
}

void PatchCharts::updatePatch(void)
{
    updatePatchL();
    updatePatchR();
}

void PatchCharts::reLoadPatchL(void)
{
    patchL->clear();
    currentPatchL%=segmapL.size();
    std::vector<int>& idxlst = segmapL[currentPatchL];
    unsigned int idx;
    for(idx=0;idx<idxlst.size();++idx)
    {
        patchL->push_back(cloudL->at(idxlst[idx]));
    }
    patchL->width = patchL->size();
    patchL->height = 1;
    QString infoL;
    infoL = infoL.sprintf("Patch %d",currentPatchL);
    ui->lineEditL->setText(infoL);
}

void PatchCharts::reLoadPatchR(void)
{
    patchR->clear();
    currentPatchR%=segmapR.size();
    std::vector<int>& idxlst = segmapR[currentPatchR];
    unsigned int idx;
    for(idx=0;idx<idxlst.size();++idx)
    {
        patchR->push_back(cloudR->at(idxlst[idx]));
    }
    patchR->width = patchR->size();
    patchR->height = 1;
    QString infoR;
    infoR = infoR.sprintf("Patch %d",currentPatchR);
    ui->lineEditR->setText(infoR);
}

void PatchCharts::updatePatchL(void)
{
    FullPoint center;
    Pipe::getPCDCenter( patchL , center );

    vL.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbL(patchL);
    vL.removeAllPointClouds();
    vL.addPointCloud<FullPoint>( patchL , rgbL , "patch" );
    vL.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "patch");

    widgetL.SetRenderWindow(vL.getRenderWindow());
    widgetL.update();
}

void PatchCharts::updatePatchR(void)
{
    FullPoint center;
    Pipe::getPCDCenter( patchR , center );

    vR.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgbR(patchR);
    vR.removeAllPointClouds();
    vR.addPointCloud<FullPoint>( patchR , rgbR , "patch" );
    vR.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "patch");

    widgetR.SetRenderWindow(vR.getRenderWindow());
    widgetR.update();
}

void PatchCharts::lastL(void)
{
    if(currentPatchL==0)currentPatchL=segmapL.size();
    currentPatchL--;
    reLoadPatchL();
    updatePatchL();
}

void PatchCharts::lastR(void)
{
    if(currentPatchR==0)currentPatchR=segmapR.size();
    currentPatchR--;
    reLoadPatchR();
    updatePatchR();
}

void PatchCharts::nextL(void)
{
    currentPatchL++;
    reLoadPatchL();
    updatePatchL();
}

void PatchCharts::nextR(void)
{
    currentPatchR++;
    reLoadPatchR();
    updatePatchR();
}

void PatchCharts::changePair(bool)
{
    ;
}

void PatchCharts::compare(void)
{
    cv::Mat colorL;
    colorL = Histogram::getColorHist(patchL);
    cv::Mat colorR;
    colorR = Histogram::getColorHist(patchR);

    cv::Mat sizeL;
    sizeL = Histogram::getBoxSize(patchL);
    cv::Mat sizeR;
    sizeR = Histogram::getBoxSize(patchR);

    cv::Mat nL;
    nL = Histogram::getNorm(patchL);
    cv::Mat nR;
    nR = Histogram::getNorm(patchR);

    cv::Mat nDL;
    nDL = Histogram::getAbsCosineNormHist(patchL);
    cv::Mat nDR;
    nDR = Histogram::getAbsCosineNormHist(patchR);

    FeatureDialog dialog(this);
    dialog.addFeature(colorL,colorR,std::string("R+G+B"));
    dialog.addFeature(nL,nR,std::string("Primary Norm"));
    dialog.addFeature(nDL,nDR,std::string("Abs Cosine Norm Hist"));
    dialog.addFeature(sizeL,sizeR,std::string("Box Size"));

    dialog.calcDists();
    dialog.showDists();
    dialog.exec();
}

void PatchCharts::icp(void)
{
    ICPMixedInitView* w = new ICPMixedInitView();

    if(patchL->size()>patchR->size())
    {
        w->setA(patchR);
        w->setB(patchL);
    }else{
        w->setA(patchL);
        w->setB(patchR);
    }
    w->run();
    emit passToMdi((QWidget*)w);
}

void PatchCharts::sicp(void)
{
    ICPViewer* w;
    if(patchL->size()>patchR->size())
    {
        w = new ICPViewer(patchR,patchL);
    }else{
        w = new ICPViewer(patchL,patchR);
    }
    emit passToMdi((QWidget*)w);
    w->run();
}

PatchCharts::~PatchCharts()
{
    delete ui;
}
