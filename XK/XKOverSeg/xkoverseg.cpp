#include "xkoverseg.h"
#include <QCoreApplication>
#include <QByteArray>
#include <pcl/io/ply_io.h>
#include <QDir>
#include <QFileInfo>
#include <QFileInfoList>
#include <pcl/features/normal_3d_omp.h>
XKOverSeg::XKOverSeg(QObject *parent) : QObject(parent)
{
    voxel_resolution = 0.008f;
    seed_resolution = 0.1f;
    color_importance = 0.2f;
    spatial_importance = 0.4f;
    normal_importance = 1.0f;

    updateToMonitor = false;

    _Socket = NULL;
    timer = new QTimer();
    timer->setSingleShot(true);
    connect(timer,SIGNAL(timeout()),this,SLOT(work()));
    timer->start(5);
}

XKOverSeg::~XKOverSeg()
{
    _Socket->disconnectFromServer();
    _Socket->deleteLater();
    timer->stop();
    timer->deleteLater();
}

void XKOverSeg::init(void)
{
    if( _Socket == NULL )
    {
        _Socket = new QLocalSocket;
        connect(_Socket,SIGNAL(connected()),this,SLOT(connected()));
        connect(_Socket,SIGNAL(readyRead()),this,SLOT(respond()));
        connect(_Socket,SIGNAL(disconnected()),this,SLOT(disconnected()));
        _Socket->connectToServer("XK");
    }
    parse();
}

void XKOverSeg::connected(void)
{
    QByteArray data;
    QDataStream stream(&data,QIODevice::WriteOnly);
    stream<<tr("Name");
    stream<<tr("OverSeg");
    _Socket->write(data);
}

void XKOverSeg::disconnected(void)
{
    Pipe::inform("Disconnected");
}

void XKOverSeg::respond(void)
{
    std::cerr<<"responding"<<std::endl;

}

void XKOverSeg::parse(void)
{
    QStringList arguments = QCoreApplication::arguments();
    _InputPath = arguments[1].toStdString();
    _OutputPath = arguments[2].toStdString();
    if( arguments.size() > 3 )
    {
        _Suffix = arguments[3].toStdString();
    }
    if( arguments.size() > 4 )
    {
        _Prefix = arguments[4].toStdString();
    }
    if( arguments.size() > 5 )
    {
        voxel_resolution = arguments[5].toFloat();
    }
    if( arguments.size() > 6 )
    {
        seed_resolution = arguments[6].toFloat();
    }
    if( arguments.size() > 7 )
    {
        color_importance = arguments[7].toFloat();
    }
    if( arguments.size() > 8 )
    {
        spatial_importance = arguments[8].toFloat();
    }
    if( arguments.size() > 9 )
    {
        normal_importance = arguments[9].toFloat();
    }
}

void XKOverSeg::work(void)
{
    QStringList inputList = getInputFiles();
    QDir dir;
    while(!inputList.isEmpty())
    {
        _CurrentFileName = inputList.takeFirst();
        QString filepath = dir.relativeFilePath(QString::fromStdString(_InputPath)+"/"+_CurrentFileName);
        Pipe::inform("Processing:");
        std::cerr<<filepath.toStdString()<<std::endl;
        FullPointCloud::Ptr cloud(new FullPointCloud);
        cloud->clear();
        if(filepath.endsWith(".ply")||filepath.endsWith(".PLY"))
            pcl::io::loadPLYFile<FullPoint>(filepath.toStdString(),*cloud);
        else
        {
            pcl::PointCloud<pcl::PointXYZRGBA> tmp;
            pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filepath.toStdString(),tmp);
            pcl::copyPointCloud(tmp,*cloud);
        }
        if(cloud->empty()){
            QCoreApplication::exit(-1);
            return;
        }
        getNormal(cloud);
        getSuperVoxel(cloud);
        saveOutputFiles();
        if(updateToMonitor)toMonitor();
        QCoreApplication::processEvents();
    }
    QCoreApplication::exit(0);
}

void XKOverSeg::toMonitor(void)
{
    ;
}

QStringList XKOverSeg::getInputFiles(void)
{
    QDir dir;
    dir.setPath(QString::fromStdString(_InputPath));
    QStringList filter;
    filter<<(QString::fromStdString(_Prefix)+"*"+QString::fromStdString(_Suffix));
    return dir.entryList(filter,QDir::Files|QDir::NoSymLinks|QDir::NoDotAndDotDot,QDir::DirsLast);
}

void XKOverSeg::saveOutputFiles()
{
    QDir dir;
    dir.setPath(QString::fromStdString(_OutputPath));
    QFileInfo info(_CurrentFileName);
    QString filename = info.baseName();
    filename = dir.absoluteFilePath(filename+".svx");
    QFile file;
    file.setFileName(filename);
    file.open(QIODevice::WriteOnly);
    QDataStream stream(&file);
    XKCommon::saveClusters(stream,supervoxel_clusters);
    XKCommon::saveAdjacency(stream,supervoxel_adjacency);
    file.close();
}

void XKOverSeg::getNormal(FullPointCloud::Ptr cloud)
{
    pcl::search::Search<FullPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<FullPoint> > (new pcl::search::KdTree<FullPoint>);
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max()
                );
    est.setKSearch(15);
    est.setInputCloud(cloud);
    est.compute(*cloud);
}

void XKOverSeg::getSuperVoxel(FullPointCloud::Ptr cloud)
{
    super = std::shared_ptr<pcl::SupervoxelClustering<pcl::PointXYZRGBA>>(
                new pcl::SupervoxelClustering<pcl::PointXYZRGBA>(voxel_resolution,seed_resolution)
            );
    std::cerr<<"input cloud size"<<cloud->size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _CloudT(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr _CloudNT(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud,*_CloudT);
    pcl::copyPointCloud(*cloud,*_CloudNT);
    super->setInputCloud(_CloudT);
    super->setNormalCloud(_CloudNT);
    super->setSpatialImportance(spatial_importance);
    super->setColorImportance(color_importance);
    super->setNormalImportance(normal_importance);
    supervoxel_clusters.clear();
    supervoxel_adjacency.clear();
    super->extract(supervoxel_clusters);
    super->getSupervoxelAdjacency(supervoxel_adjacency);
    std::cerr<<"result cluster num:"<<supervoxel_clusters.size()<<std::endl;
    std::cerr<<"result adjacency num:"<<supervoxel_adjacency.size()<<std::endl;
}
