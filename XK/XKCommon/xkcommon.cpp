#include "xkcommon.h"
#include <QMultiHash>
#include <QByteArray>
template class PCL_EXPORTS pcl::Supervoxel<FullPoint>;
template class PCL_EXPORTS pcl::SupervoxelClustering<FullPoint>;

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
    savePointXYZRGBA(stream,voxel.centroid_);
    saveFullPointCloud(stream,*voxel.voxels_);
    saveNormal(stream,voxel.normal_);
    saveNormalPointCloud(stream,*voxel.normals_);
}

void XKCommon::savePointXYZRGBA(QDataStream&stream,pcl::PointXYZRGBA&point)
{
    stream<<point.x;
    stream<<point.y;
    stream<<point.z;
    stream<<point.rgba;
}

void XKCommon::saveFullPoint(QDataStream&stream,FullPoint&point)
{
    stream<<point.x;
    stream<<point.y;
    stream<<point.z;
    stream<<point.rgba;
    stream<<point.normal_x;
    stream<<point.normal_y;
    stream<<point.normal_z;
    stream<<point.curvature;
}

void XKCommon::saveFullPointCloud(QDataStream&stream,FullPointCloud&cloud)
{
    stream<<cloud.width;
    stream<<cloud.height;
    stream<<cloud.size();
    foreach(FullPoint p,cloud)
    {
        saveFullPoint(stream,p);
    }
}

void XKCommon::saveNormal(QDataStream&stream,pcl::Normal&norm)
{
    stream<<norm.normal_x;
    stream<<norm.normal_y;
    stream<<norm.normal_z;
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
    loadPointXYZRGBA(stream,voxel.centroid_);
    loadFullPointCloud(stream,*voxel.voxels_);
    loadNormal(stream,voxel.normal_);
    loadNormalPointCloud(stream,*voxel.normals_);
}

void XKCommon::loadPointXYZRGBA(QDataStream&stream,pcl::PointXYZRGBA&point)
{
    stream>>point.x;
    stream>>point.y;
    stream>>point.z;
    stream>>point.rgba;
}

void XKCommon::loadFullPoint(QDataStream&stream,FullPoint&point)
{
    stream>>point.x;
    stream>>point.y;
    stream>>point.z;
    stream>>point.rgba;
    stream>>point.normal_x;
    stream>>point.normal_y;
    stream>>point.normal_z;
    stream>>point.curvature;
}

void XKCommon::loadFullPointCloud(QDataStream&stream,FullPointCloud&cloud)
{
    stream>>cloud.width;
    stream>>cloud.height;
    int size;
    stream>>size;
    cloud.resize(size);
    FullPointCloud::iterator iter;
    for(iter=cloud.begin();iter!=cloud.end();++iter)
    {
        loadFullPoint(stream,*iter);
    }
}

void XKCommon::loadNormal(QDataStream&stream,pcl::Normal&norm)
{
    stream>>norm.normal_x;
    stream>>norm.normal_y;
    stream>>norm.normal_z;
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

