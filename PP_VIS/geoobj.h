#ifndef GEOOBJ_H
#define GEOOBJ_H
#include <memory>
#include "pipe.h"
class GeoObj
{
public:
    typedef std::shared_ptr<GeoObj> Ptr;
    GeoObj(FullPointCloud::Ptr cloud,int idx);
    ~GeoObj();
    FullPointCloud::Ptr getCloud(void);
    cv::Mat getBox(void);
    QString getName(void){return name;}
    void setName(QString& n){name = n;}
    void setIndex(int idx){_Index=idx;}
    int index(void){return _Index;}
private:
    int _Index;
    QString name;
    cv::Mat _BoundingBox;
    FullPointCloud::Ptr _Cloud;
};

#endif // GEOOBJ_H
