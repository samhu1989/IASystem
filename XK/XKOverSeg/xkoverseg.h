#ifndef XKOVERSEG_H
#define XKOVERSEG_H

#include <QObject>
#include <QLocalServer>
#include <QLocalSocket>
#include <QTimer>
#include "pipe.h"
#include <string>
#include <QStringList>
#include "supervoxel_clustering.h"
#include <memory>
#include <map>
#include <QDataStream>
class XKOverSeg : public QObject
{
    Q_OBJECT
public:
    typedef std::map<uint32_t,pcl::Supervoxel<FullPoint>::Ptr> SuperVoxelClusters;
    typedef std::multimap<uint32_t,uint32_t> SuperVoxelAdjacency;
    explicit XKOverSeg(QObject *parent = 0);
    ~XKOverSeg();
    void init(void);
signals:

protected:
    void parse(void);
    QStringList getInputFiles(void);
    void saveOutputFiles();
    void saveClusters(QDataStream&,SuperVoxelClusters&);
    void saveAdjacency(QDataStream&,SuperVoxelAdjacency&);

protected slots:
    void disconnected(void);
    void connected(void);
    void respond(void);
    void work(void);
    void getSuperVoxel(FullPointCloud::Ptr);
    void toMonitor(void);

private:
std::string _Prefix;
std::string _Suffix;
std::string _InputPath;
std::string _OutputPath;

QString _CurrentFileName;

float voxel_resolution;
float seed_resolution;
float color_importance;
float spatial_importance;
float normal_importance;

std::shared_ptr<pcl::SupervoxelClustering<FullPoint>> super;
SuperVoxelClusters supervoxel_clusters;
SuperVoxelAdjacency supervoxel_adjacency;

bool updateToMonitor;

QLocalSocket* _Socket;
QTimer* timer;
};

#endif // XKOVERSEG_H

