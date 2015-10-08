#include "xkcommon.h"
template class PCL_EXPORTS pcl::Supervoxel<FullPoint>;
template class PCL_EXPORTS pcl::SupervoxelClustering<FullPoint>;

void XKCommon::saveClusters(QDataStream&stream,SuperVoxelClusters&cluster)
{
    ;
}

void XKCommon::saveAdjacency(QDataStream&stream,SuperVoxelAdjacency&adjacency)
{
    ;
}

void XKCommon::loadClusters(QDataStream&,SuperVoxelClusters&)
{
    ;
}

void XKCommon::loadAdjacency(QDataStream&,SuperVoxelClusters&)
{
    ;
}
