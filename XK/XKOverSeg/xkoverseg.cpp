#include "xkoverseg.h"
#include <QCoreApplication>
#include <QByteArray>
#include <QDataStream>
#include "supervoxel_clustering.hpp"
#include <pcl/io/ply_io.h>
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

void XKOverSeg::work(void)
{
    bool done = false;
    std::string input_file;
    while(!done)
    {
        FullPointCloud::Ptr cloud(new FullPointCloud);
        pcl::io::loadPLYFile<FullPoint>(input_file,*cloud);
        getSuperVoxel(cloud);
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
    return QStringList();
}

void XKOverSeg::getSuperVoxel(FullPointCloud::Ptr cloud)
{
    super = std::shared_ptr<pcl::SupervoxelClustering<FullPoint>>(
                new pcl::SupervoxelClustering<FullPoint>(voxel_resolution,seed_resolution)
            );
    super->setInputCloud(cloud);
    super->setSpatialImportance(spatial_importance);
    super->setColorImportance(color_importance);
    super->setNormalImportance(normal_importance);
    supervoxel_clusters.clear();
    super->extract(supervoxel_clusters);
    super->getSupervoxelAdjacency(supervoxel_adjacency);
}
