#ifndef XKCOMMON_H
#define XKCOMMON_H
#include "xkcommon_global.h"
#include "supervoxel_clustering.h"
#include "pipe.h"
#include <QDataStream>
#include <map>
#include <memory>
#include <QStringList>
template class XKCOMMONSHARED_EXPORT pcl::Supervoxel<pcl::PointXYZRGBA>;
template class XKCOMMONSHARED_EXPORT pcl::SupervoxelClustering<pcl::PointXYZRGBA>;
namespace XKCommon{
    typedef pcl::PointXYZRGBA ColorPoint;
    typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
    typedef pcl::Supervoxel<ColorPoint> SuperVoxel;
    typedef std::map<uint32_t,SuperVoxel::Ptr> SuperVoxelClusters;
    typedef std::multimap<uint32_t,uint32_t> SuperVoxelAdjacency;

    void XKCOMMONSHARED_EXPORT saveClusters(QDataStream&,SuperVoxelClusters&);
    void XKCOMMONSHARED_EXPORT saveAdjacency(QDataStream&,SuperVoxelAdjacency&);
    void XKCOMMONSHARED_EXPORT saveSuperVoxel(QDataStream&,SuperVoxel&);
    void XKCOMMONSHARED_EXPORT saveColorPoint(QDataStream&,ColorPoint&);
    void XKCOMMONSHARED_EXPORT saveColorPointCloud(QDataStream&,ColorPointCloud&);
    void XKCOMMONSHARED_EXPORT saveNormal(QDataStream&,pcl::Normal&);
    void XKCOMMONSHARED_EXPORT saveNormalPointCloud(QDataStream&,pcl::PointCloud<pcl::Normal>&);

    void XKCOMMONSHARED_EXPORT loadClusters(QDataStream&,SuperVoxelClusters&);
    void XKCOMMONSHARED_EXPORT loadAdjacency(QDataStream&,SuperVoxelAdjacency&);
    void XKCOMMONSHARED_EXPORT loadSuperVoxel(QDataStream&,SuperVoxel&);
    void XKCOMMONSHARED_EXPORT loadColorPoint(QDataStream&,ColorPoint&);
    void XKCOMMONSHARED_EXPORT loadColorPointCloud(QDataStream&,ColorPointCloud&);
    void XKCOMMONSHARED_EXPORT loadNormal(QDataStream&,pcl::Normal&);
    void XKCOMMONSHARED_EXPORT loadNormalPointCloud(QDataStream&,pcl::PointCloud<pcl::Normal>&);
    QString XKCOMMONSHARED_EXPORT getNativeArguments(Config&,QStringList&);
}
#endif // XKCOMMON_H
