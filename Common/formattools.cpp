#include "formattools.h"
#include "ui_formattools.h"
#include <QFileDialog>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <QMessageBox>
#include <pcl/registration/transformation_estimation_svd.h>
#include "icp.h"
#include <pcl/features/normal_3d_omp.h>
FormatTools::FormatTools(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::FormatTools)
{
    ui->setupUi(this);
    w = std::shared_ptr<QVTKWidget>(new QVTKWidget());
    ui->gridLayout_2->addWidget(w.get());
    v = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer);
    v->setBackgroundColor(0.7,0.7,0.7);
    ui->widget->setMinimumSize(320,240);
    v->addCoordinateSystem(0.3);
    v->registerPointPickingCallback<FormatTools>(&FormatTools::pick,*this,NULL);
    v->registerKeyboardCallback<FormatTools>(&FormatTools::key,*this,NULL);
    w->SetRenderWindow(v->getRenderWindow());
    cloud = FullPointCloud::Ptr(new FullPointCloud);
    ref = FullPointCloud::Ptr(new FullPointCloud);
    connect(ui->load,SIGNAL(clicked()),this,SLOT(load()));
    connect(ui->loadRef,SIGNAL(clicked()),this,SLOT(loadRef()));
    connect(ui->save,SIGNAL(clicked()),this,SLOT(save()));
    connect(ui->setZ,SIGNAL(clicked()),this,SLOT(setZtoUp()));
    connect(ui->align,SIGNAL(clicked()),this,SLOT(alignRef()));
    connect(ui->showBoth,SIGNAL(clicked()),this,SLOT(showAlign()));
    _State = TARGET;
}

void FormatTools::load(void)
{
    QFileDialog fd(this,tr("Load Point Cloud"), "./data/","" );
    fd.setFileMode(QFileDialog::AnyFile);
    fd.setViewMode(QFileDialog::Detail);
    fd.setAcceptMode(QFileDialog::AcceptOpen);
    fd.setNameFilter(QString(tr("Point Cloud(*.pcd *.PCD *.ply *.PLY *.obj *.OBJ)")));
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }
    if(fileNamesList.empty())return;
    QString fname = fileNamesList[0];
    _PickedIndex.clear();
    _State = TARGET;
    v->removeAllShapes();
    if(fname.endsWith(".pcd")||fname.endsWith(".PCD"))
    {
        loadPCD(fname.toStdString(),cloud,cloudInfo);
    }else if(fname.endsWith(".PLY")||fname.endsWith(".ply"))
    {
        loadPLY(fname.toStdString(),cloud,cloudInfo);
    }else if(fname.endsWith(".OBJ")||fname.endsWith(".obj"))
    {
        loadOBJ(fname.toStdString(),cloud,cloudInfo);
    }else return;
}

void FormatTools::loadRef(void)
{
    QFileDialog fd(this,tr("Load Reference Point Cloud"), "./data/","" );
    fd.setFileMode(QFileDialog::AnyFile);
    fd.setViewMode(QFileDialog::Detail);
    fd.setAcceptMode(QFileDialog::AcceptOpen);
    fd.setNameFilter(QString(tr("Point Cloud(*.pcd *.PCD *.ply *.PLY *.obj *.OBJ)")));
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }
    if(fileNamesList.empty())return;
    QString fname = fileNamesList[0];
    _PickedRef.clear();
    _State = REF;
    v->removeAllShapes();
    if(fname.endsWith(".pcd")||fname.endsWith(".PCD"))
    {
        loadPCD(fname.toStdString(),ref,refInfo);
    }else if(fname.endsWith(".PLY")||fname.endsWith(".ply"))
    {
        loadPLY(fname.toStdString(),ref,refInfo);
    }else if(fname.endsWith(".OBJ")||fname.endsWith(".obj"))
    {
        loadOBJ(fname.toStdString(),ref,refInfo);
    }else return;
}

void FormatTools::loadPCD(const std::string&fname,FullPointCloud::Ptr cloud,QFileInfo&info)
{
    Pipe::inform("Loading PCD");
    std::cerr<<fname<<std::endl;
    cloud->clear();
    pcl::io::loadPCDFile(fname,*cloud);
    info = QFileInfo(QString::fromStdString(fname));
    showCloud(true);
}

void FormatTools::loadPLY(const std::string&fname,FullPointCloud::Ptr cloud,QFileInfo&info)
{
    Pipe::inform("Loading PLY");
    std::cerr<<fname<<std::endl;
    cloud->clear();
    pcl::io::loadPLYFile(fname,*cloud);
    info = QFileInfo(QString::fromStdString(fname));
    showCloud(true);
}

void FormatTools::loadOBJ(const std::string&fname,FullPointCloud::Ptr cloud,QFileInfo&info)
{
    Pipe::inform("Loading OBJ");
    std::cerr<<fname<<std::endl;
}

void FormatTools::showCloud(bool updateCam)
{
    FullPointCloud::Ptr showCloud;
    QFileInfo info;
    switch(_State)
    {
    case REF:
        showCloud=ref;info=refInfo;break;
    case TARGET:
    default:
        showCloud=cloud;info=cloudInfo;break;
    }
    FullPoint center;
    Pipe::getPCDCenter( showCloud , center );
    if(updateCam)v->setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    v->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgb(showCloud);
    v->addPointCloud<FullPoint>( showCloud , rgb , info.baseName().toStdString() );
    v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, info.baseName().toStdString());
    w->SetRenderWindow(v->getRenderWindow());
    w->update();
}

void FormatTools::showAlign(void)
{
    FullPoint center;
    Pipe::getPCDCenter( ref , center );
    v->setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    v->removeAllPointClouds();
    v->removeAllShapes();
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgb(cloud);
    v->addPointCloud<FullPoint>( cloud , rgb , cloudInfo.baseName().toStdString() );
    v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudInfo.baseName().toStdString());
    pcl::visualization::PointCloudColorHandlerCustom<FullPoint> blue(ref,0,0,255);
    v->addPointCloud<FullPoint>( ref , blue , refInfo.baseName().toStdString() );
    v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, refInfo.baseName().toStdString());
    w->SetRenderWindow(v->getRenderWindow());
    w->update();
}

void FormatTools::save(void)
{
    if(cloud->empty())return;
    QFileDialog fd(this,tr("Save Point Cloud"), "./data/","" );
    fd.setFileMode(QFileDialog::AnyFile);
    fd.setViewMode(QFileDialog::Detail);
    fd.setAcceptMode(QFileDialog::AcceptSave);
    fd.setNameFilter(QString(tr("Point Cloud(*.pcd *.PCD *.ply *.PLY *.obj *.OBJ)")));
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }
    if(fileNamesList.empty())return;
    QString fname = fileNamesList[0];
    //estimate normal for point cloud before save
    pcl::search::KdTree<FullPoint>::Ptr tree (new pcl::search::KdTree<FullPoint> ());
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(0,0,0);
    est.setInputCloud(cloud);
    est.setKSearch(10);
    est.compute(*cloud);
    if(fname.endsWith(".pcd")||fname.endsWith(".PCD"))
    {
        pcl::io::savePCDFileBinary<FullPoint>(fname.toStdString(),*cloud);
    }else if(fname.endsWith(".PLY")||fname.endsWith(".ply"))
    {
        sensor_msgs::PointCloud2Ptr tmp(new sensor_msgs::PointCloud2);
        std::vector<int> indexes;
        pcl::removeNaNFromPointCloud(*cloud,*cloud,indexes);
        pcl::removeZeroFromPointCloud(*cloud,*cloud,indexes);
        pcl::toROSMsg(*cloud,*tmp);
        pcl::io::savePLYFile(
                    fname.toStdString(),
                    *tmp,
                    Eigen::Vector4f::Zero(),
                    Eigen::Quaternionf::Identity(),
                    false,false
                    );
    }else{
        pcl::io::savePCDFileBinary<FullPoint>(fname.toStdString()+".pcd",*cloud);
    }
}

void FormatTools::setZtoUp(void)
{
    if(TARGET!=_State)return;
    if(_PickedIndex.empty())return;
    pcl::SACSegmentation<FullPoint> seg;
    pcl::ModelCoefficients coeff;
    pcl::PointIndices indices;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/36);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (ICP::DOWN_SAMPLE_GRID_SIZE*1.50);
    seg.setInputCloud(cloud);
    seg.segment(indices,coeff);

    Eigen::Affine3f t;
    Eigen::Vector3f u,tv(0,0,1);
    u(0)=coeff.values[0];
    u(1)=coeff.values[1];
    u(2)=coeff.values[2];
    if(u(2)>0)u*=-1;
    t = pcl::getTransformationFromTwoUnitVectors(tv,u);
    Eigen::Matrix4f R;
    R = t.matrix();

    Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
    T(0,3) = cloud->at(_PickedIndex.back()).x;
    T(1,3) = cloud->at(_PickedIndex.back()).y;
    T(2,3) = cloud->at(_PickedIndex.back()).z;
    pcl::transformPointCloud<FullPoint>(*cloud,*cloud,R*T.inverse());
    _PickedIndex.clear();
    v->removeAllShapes();
    showCloud(true);
}

void FormatTools::alignRef(void)
{

    if(ref->empty())
    {
        QMessageBox::warning(this,tr("Can't Align to Reference"),tr("No Reference"));
        return;
    }else if(cloud->empty())
    {
        QMessageBox::warning(this,tr("Can't Align to Reference"),tr("Only Reference, No Target"));
        return;
    }

    if(_PickedRef.size()<4)
    {
        QMessageBox::warning(this,tr("Can't Align to Reference"),tr("Not Enough Points Picked in Reference"));
        return;
    }
    if(_PickedIndex.size()<4)
    {
        QMessageBox::warning(this,tr("Can't Align to Reference"),tr("Not Enough Points Picked in Target Point Cloud"));
        return;
    }

    pcl::registration::TransformationEstimation<FullPoint,FullPoint>::Ptr est(new pcl::registration::TransformationEstimationSVD<FullPoint,FullPoint>());
    Eigen::Matrix4f T;
    pcl::Correspondences corrs;
    int idx=0;
    while(idx<_PickedRef.size()&&idx<_PickedIndex.size())
    {
        corrs.push_back(pcl::Correspondence());
        corrs.back().index_query = _PickedIndex[idx];
        corrs.back().index_match = _PickedRef[idx];
        ++idx;
    }
    est->estimateRigidTransformation(*cloud,*ref,corrs,T);
    ICP::ICPGeneral icp;
    icp._T = T;
    icp(cloud,ref);
    T = icp._T;
    pcl::transformPointCloud<FullPoint>(*cloud,*cloud,T);
    showAlign();
}

void FormatTools::pick(const pcl::visualization::PointPickingEvent & event, void*)
{
    unsigned int idx=0;
    FullPoint p;
    switch(_State)
    {
    case REF:
        idx = _PickedRef.size();
        _PickedRef.push_back(event.getPointIndex());
        p = ref->at(_PickedRef.back());
        break;
    case TARGET:
    default:
        idx = _PickedIndex.size();
        _PickedIndex.push_back(event.getPointIndex());
        p = cloud->at(_PickedIndex.back());
    }
    idx%=Pipe::FalseColorNum;
    QString name;
    name = name.sprintf("%d",idx);
    float r,g,b;
    r = float(Pipe::FalseColor[idx][0])/255.0;
    g = float(Pipe::FalseColor[idx][1])/255.0;
    b = float(Pipe::FalseColor[idx][2])/255.0;
    v->addSphere (p,0.05,r,g,b,name.toStdString());
    w->update();
}

void FormatTools::key(const pcl::visualization::KeyboardEvent& event, void*)
{
    switch(_State)
    {
    case REF:
        if(_PickedRef.empty())return;
        if("Delete"==event.getKeySym()&&event.keyDown())
        {
            unsigned int idx = _PickedRef.size()-1;
            QString name;
            name = name.sprintf("%d",idx);
            v->removeShape(name.toStdString());
            _PickedRef.pop_back();
            w->update();
        }
    case TARGET:
    default:
        if(_PickedIndex.empty())return;
        if("Delete"==event.getKeySym()&&event.keyDown())
        {
            unsigned int idx = _PickedIndex.size()-1;
            QString name;
            name = name.sprintf("%d",idx);
            v->removeShape(name.toStdString());
            _PickedIndex.pop_back();
            w->update();
        }
    }
}

void FormatTools::remove(void)
{
    pcl::ExtractIndices<FullPoint> extract;
    extract.setNegative(true);
    FullPointCloud tmp;
    tmp = *cloud;
    pcl::IndicesPtr indices(&_Selected);
    extract.setIndices(indices);
    extract.setInputCloud(cloud);
    extract.filter(*cloud);
    showCloud();
}

FormatTools::~FormatTools()
{
    delete ui;
}
