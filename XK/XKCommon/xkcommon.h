#ifndef XKCOMMON_H
#define XKCOMMON_H
#include "xkcommon_global.h"
#include "supervoxel_clustering.h"
#include "pipe.h"
#include <QDataStream>
#include <map>
#include <memory>
namespace XKCommon{
    typedef std::map<uint32_t,pcl::Supervoxel<FullPoint>::Ptr> SuperVoxelClusters;
    typedef std::multimap<uint32_t,uint32_t> SuperVoxelAdjacency;
    typedef pcl::Supervoxel<FullPoint> SuperVoxel;

    void XKCOMMONSHARED_EXPORT saveClusters(QDataStream&,SuperVoxelClusters&);
    void XKCOMMONSHARED_EXPORT saveAdjacency(QDataStream&,SuperVoxelAdjacency&);
    void XKCOMMONSHARED_EXPORT saveSuperVoxel(QDataStream&,SuperVoxel&);
    void XKCOMMONSHARED_EXPORT saveFullPoint(QDataStream&,FullPoint&);
    void XKCOMMONSHARED_EXPORT saveFullPointCloud(QDataStream&,FullPointCloud&);
    void XKCOMMONSHARED_EXPORT savePointXYZRGBA(QDataStream&,pcl::PointXYZRGBA&);
    void XKCOMMONSHARED_EXPORT saveNormal(QDataStream&,pcl::Normal&);
    void XKCOMMONSHARED_EXPORT saveNormalPointCloud(QDataStream&,pcl::PointCloud<pcl::Normal>&);

    void XKCOMMONSHARED_EXPORT loadClusters(QDataStream&,SuperVoxelClusters&);
    void XKCOMMONSHARED_EXPORT loadAdjacency(QDataStream&,SuperVoxelAdjacency&);
    void XKCOMMONSHARED_EXPORT loadSuperVoxel(QDataStream&,SuperVoxel&);
    void XKCOMMONSHARED_EXPORT loadFullPoint(QDataStream&,FullPoint&);
    void XKCOMMONSHARED_EXPORT loadFullPointCloud(QDataStream&,FullPointCloud&);
    void XKCOMMONSHARED_EXPORT loadPointXYZRGBA(QDataStream&,pcl::PointXYZRGBA&);
    void XKCOMMONSHARED_EXPORT loadNormal(QDataStream&,pcl::Normal&);
    void XKCOMMONSHARED_EXPORT loadNormalPointCloud(QDataStream&,pcl::PointCloud<pcl::Normal>&);
}
#endif // XKCOMMON_H
