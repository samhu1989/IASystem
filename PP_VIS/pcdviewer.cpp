#include "pcdviewer.h"
#include "ui_pcdviewer.h"

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/common.h>

PCDViewer::PCDViewer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PCDViewer)
{
    ui->setupUi(this);
    setMinimumHeight(240);
    setMinimumWidth(320);
    setWindowTitle("PCDViewer");
    v.initCameraParameters();
    widget.SetRenderWindow(v.getRenderWindow());
    ui->gridLayout->addWidget(&widget);
}

bool PCDViewer::addClouds(const QList<quint32>&hashKeys,bool multiView)
{
    FullPointCloud::Ptr cloudPtr;
    QList<FullPointCloud::Ptr> cloudList;
    QStringList nameList;
    foreach(quint32 k,hashKeys)
    {
        cloudPtr = FullPointCloud::Ptr(new FullPointCloud);
        QString name;
        if( Pipe::loadData(cloudPtr,name,k) )
        {
            cloudList.push_back(cloudPtr);

            nameList.push_back(name);
        }
    }
    if(cloudList.empty())return false;
    if(nameList.size()==1)setWindowTitle(nameList[0]);
    if(multiView)
    {
        addCloudsInMultiView(cloudList,nameList);
    }else{
        addCloudsInOneView(cloudList,nameList);
    }
    return true;
}

void PCDViewer::addCloudsInMultiView(const QList<FullPointCloud::Ptr>&pcdList,const QStringList& nameList)
{
    int cMax = std::sqrtf((float)pcdList.size());
    if( cMax*cMax < pcdList.size()) cMax+=1;
    if(cMax > 4){
        cMax = 4;
        Pipe::inform("Impossible to view too much Point Cloud at once");
    }
    int rMax = pcdList.size() / cMax;
    if(rMax*cMax<pcdList.size())rMax+=1;
    float w = 1.0 / cMax;
    float h = 1.0 / rMax;
    int r,c;
    for(r=0;r<rMax;++r)
    {
        for(c=0;c<cMax;++c)
        {
            int idx = r*cMax+c;
            int vId(0);
            if( idx >= pcdList.size() )break;
            FullPointCloud::Ptr cloud = pcdList[idx];
            QString name = nameList[idx];
            v.createViewPort(c*w,r*h,(c+1)*w,(r+1)*h,vId);
            FullPoint center;
            Pipe::getPCDCenter(cloud,center);
            v.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1,vId);
            v.setBackgroundColor(1.0,1.0,1.0,vId);
            pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgb(cloud);
            v.addPointCloud<FullPoint> ( cloud, rgb,name.toStdString(), vId);
            v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name.toStdString());
//            v.addPointCloudNormals<FullPoint,FullPoint> (cloud,cloud,2,0.05,name.toStdString()+"_Norm",vId);
            v.addCoordinateSystem(0.3,vId);
        }
    }
}

void PCDViewer::addCloudsInOneView(const QList<FullPointCloud::Ptr>&pcdList,const QStringList& nameList)
{
    ;
}

bool PCDViewer::addClouds(const QStringList &pcdfile)
{
    QList<FullPointCloud::Ptr> pcdList;
    bool successOnce = false;
    foreach(QString filename,pcdfile)
    {
        FullPointCloud::Ptr ptr(new FullPointCloud);
        if(0>pcl::io::loadPCDFile(filename.toStdString(),*ptr))
        {
            Pipe::inform("Failed to load pcd at "+filename.toStdString());
            Pipe::record("Failed to load pcd at "+filename.toStdString());
        }else{
            successOnce = true;
        }
        pcdList.push_back(ptr);
    }
    addCloudsInMultiView(pcdList,pcdfile);
    return successOnce;
}

PCDViewer::~PCDViewer()
{
    delete ui;
}
