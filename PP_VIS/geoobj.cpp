#include "geoobj.h"

GeoObj::GeoObj(FullPointCloud::Ptr cloud,int idx):_Index(idx)
{
   _Cloud = cloud;
}

FullPointCloud::Ptr GeoObj::getCloud(void)
{
    return _Cloud;
}

cv::Mat GeoObj::getBox(void)
{
    if(_BoundingBox.empty())
    {
        Pipe::getBoundingBox(_Cloud,_BoundingBox);
    }
    return _BoundingBox;
}

GeoObj::~GeoObj()
{

}

