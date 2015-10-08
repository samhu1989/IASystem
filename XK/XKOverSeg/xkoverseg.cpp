#include "xkoverseg.h"
#include <QCoreApplication>
#include <QByteArray>
#include "supervoxel_clustering.hpp"
#include <pcl/io/ply_io.h>
#include <QDir>
#include <QFileInfo>
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
    while(!inputList.isEmpty())
    {
        _CurrentFileName = inputList.takeFirst();
        FullPointCloud::Ptr cloud(new FullPointCloud);
        cloud->clear();
        pcl::io::loadPLYFile<FullPoint>(_CurrentFileName.toStdString(),*cloud);
        if(cloud->empty()){
            QCoreApplication::exit(-1);
            return;
        }
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
    saveClusters(stream,supervoxel_clusters);
    saveAdjacency(stream,supervoxel_adjacency);
    file.close();
}

void XKOverSeg::saveClusters(QDataStream&,SuperVoxelClusters&)
{
    ;
}

void XKOverSeg::saveAdjacency(QDataStream&,SuperVoxelAdjacency&)
{
    ;
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
