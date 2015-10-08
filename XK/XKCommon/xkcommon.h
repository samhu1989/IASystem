#ifndef XKCOMMON_H
#define XKCOMMON_H
#include "xkcommon_global.h"
#include "supervoxel_clustering.h"
#include "pipe.h"
#include <QDataStream>
namespace XKCommon{
    typedef std::map<uint32_t,pcl::Supervoxel<FullPoint>::Ptr> SuperVoxelClusters;
    typedef std::multimap<uint32_t,uint32_t> SuperVoxelAdjacency;

    void XKCOMMONSHARED_EXPORT saveClusters(QDataStream&,SuperVoxelClusters&);
    void XKCOMMONSHARED_EXPORT saveAdjacency(QDataStream&,SuperVoxelAdjacency&);
    void XKCOMMONSHARED_EXPORT loadClusters(QDataStream&,SuperVoxelClusters&);
    void XKCOMMONSHARED_EXPORT loadAdjacency(QDataStream&,SuperVoxelClusters&);
}
#endif // XKCOMMON_H
