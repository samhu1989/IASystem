#include "xkcommon.h"
#include <QMultiHash>
#include <QByteArray>
#include "supervoxel_clustering.hpp"
#include "octree/impl/octree_base.hpp"
#include "octree/impl/octree_pointcloud.hpp"

template class pcl::Supervoxel<FullPoint>;
template class pcl::SupervoxelClustering<FullPoint>;

void XKCommon::saveClusters(QDataStream&stream,SuperVoxelClusters&clusters)
{
    QMultiHash<uint32_t,QByteArray> qClusters;
    foreach (SuperVoxelClusters::value_type v, clusters) {
        QByteArray data;
        QDataStream stream(&data,QIODevice::WriteOnly);
        saveSuperVoxel(stream,*v.second);
        qClusters.insert(v.first,data);
    }
    stream<<qClusters;
}

void XKCommon::saveAdjacency(QDataStream&stream,SuperVoxelAdjacency&adjacency)
{
    QMultiHash<uint32_t,uint32_t> qadjacency;
    foreach(SuperVoxelAdjacency::value_type v,adjacency)
    {
        qadjacency.insert(v.first,v.second);
    }
    stream<<qadjacency;
}

void XKCommon::saveSuperVoxel(QDataStream&stream,SuperVoxel&voxel)
{
    saveColorPoint(stream,voxel.centroid_);
    saveColorPointCloud(stream,*voxel.voxels_);
    saveNormal(stream,voxel.normal_);
    saveNormalPointCloud(stream,*voxel.normals_);
}

void XKCommon::saveColorPoint(QDataStream&stream,ColorPoint&point)
{
    stream<<point.x;
    stream<<point.y;
    stream<<point.z;
    stream<<point.rgba;
}

void XKCommon::saveColorPointCloud(QDataStream&stream,ColorPointCloud&cloud)
{
    stream<<cloud.width;
    stream<<cloud.height;
    stream<<cloud.size();
    foreach(ColorPoint p,cloud)
    {
        saveColorPoint(stream,p);
    }
}

void XKCommon::saveNormal(QDataStream&stream,pcl::Normal&norm)
{
    stream<<norm.normal_x;
    stream<<norm.normal_y;
    stream<<norm.normal_z;
    stream<<norm.curvature;
}

void XKCommon::saveNormalPointCloud(QDataStream&stream,pcl::PointCloud<pcl::Normal>&cloud)
{
    stream<<cloud.width;
    stream<<cloud.height;
    stream<<cloud.size();
    foreach(pcl::Normal n,cloud)
    {
        saveNormal(stream,n);
    }
}

void XKCommon::loadClusters(QDataStream&stream,SuperVoxelClusters&clusters)
{
    QMultiHash<uint32_t,QByteArray> qClusters;
    QMultiHash<uint32_t,QByteArray>::iterator iter;
    stream>>qClusters;
    for(iter=qClusters.begin();iter!=qClusters.end();++iter)
    {
        uint32_t key = iter.key();
        SuperVoxel::Ptr voxel(new SuperVoxel);
        QDataStream stream(&iter.value(),QIODevice::ReadWrite);
        loadSuperVoxel(stream,*voxel);
        clusters.insert(std::make_pair(key,voxel));
    }
}

void XKCommon::loadAdjacency(QDataStream&stream,SuperVoxelAdjacency&adjacency)
{
    QMultiHash<uint32_t,uint32_t> qadjacency;
    QMultiHash<uint32_t,uint32_t>::iterator iter;
    stream>>qadjacency;
    for(iter=qadjacency.begin();iter!=qadjacency.end();++iter)
    {
        uint32_t key = iter.key();
        uint32_t v = iter.value();
        adjacency.insert(std::make_pair(key,v));
    }
}

void XKCommon::loadSuperVoxel(QDataStream&stream,SuperVoxel&voxel)
{
    loadColorPoint(stream,voxel.centroid_);
    loadColorPointCloud(stream,*voxel.voxels_);
    loadNormal(stream,voxel.normal_);
    loadNormalPointCloud(stream,*voxel.normals_);
}

void XKCommon::loadColorPoint(QDataStream&stream,pcl::PointXYZRGBA&point)
{
    stream>>point.x;
    stream>>point.y;
    stream>>point.z;
    stream>>point.rgba;
}

void XKCommon::loadColorPointCloud(QDataStream&stream,ColorPointCloud&cloud)
{
    stream>>cloud.width;
    stream>>cloud.height;
    int size;
    stream>>size;
    cloud.resize(size);
    ColorPointCloud::iterator iter;
    for(iter=cloud.begin();iter!=cloud.end();++iter)
    {
        loadColorPoint(stream,*iter);
    }
}

void XKCommon::loadNormal(QDataStream&stream,pcl::Normal&norm)
{
    stream>>norm.normal_x;
    stream>>norm.normal_y;
    stream>>norm.normal_z;
    stream>>norm.curvature;
}

void XKCommon::loadNormalPointCloud(QDataStream&stream,pcl::PointCloud<pcl::Normal>&cloud)
{
    stream>>cloud.width;
    stream>>cloud.height;
    int size;
    stream>>size;
    cloud.resize(size);
    pcl::PointCloud<pcl::Normal>::iterator iter;
    for(iter=cloud.begin();iter!=cloud.end();++iter)
    {
        loadNormal(stream,*iter);
    }
}

QString XKCommon::getNativeArguments(Config& config,QStringList& keys)
{
    QString arguments;
    while(!keys.empty())
    {
        arguments+=QString::fromStdString(config.getString(keys.takeFirst().toStdString()));
        if( !keys.empty() && config.has( keys.front().toStdString() ) )arguments += " ";
    }
    return arguments;
}
