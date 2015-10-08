#ifndef HISTOGRAM_H
#define HISTOGRAM_H
#include "common_global.h"
#include "pipe.h"
class COMMONSHARED_EXPORT Histogram
{
public:
    static cv::Mat getColorHist(FullPointCloud::Ptr cloud);
    static cv::Mat getBoxSize(FullPointCloud::Ptr cloud);
    static cv::Mat getDifSize(FullPointCloud::Ptr cloud);
    static cv::Mat getDifSize(FullPointCloud::Ptr cloud,FullPoint &center);
    static cv::Mat getDifSize(FullPointCloud::Ptr cloud,FullPoint &center,FullPointCloud::Ptr& corners);
    static cv::Mat getNorm(FullPointCloud::Ptr cloud);
    static cv::Mat getAbsCosineNormHist(FullPointCloud::Ptr cloud);
    static double getEuclidean(cv::Mat&,cv::Mat&);
    static double getCosine(cv::Mat&,cv::Mat&);
};

#endif // HISTOGRAM_H
