#include "histogram.h"
#include <pcl/features/normal_3d.h>
cv::Mat Histogram::getColorHist(FullPointCloud::Ptr cloud)
{
    cv::Mat imgR(1,cloud->size(),CV_8UC1);
    cv::Mat imgG(1,cloud->size(),CV_8UC1);
    cv::Mat imgB(1,cloud->size(),CV_8UC1);
    unsigned int idx;
    for(idx=0;idx<cloud->size();++idx)
    {
        imgB.at<uchar>(0,idx)= cloud->at(idx).b;
        imgG.at<uchar>(0,idx)= cloud->at(idx).g;
        imgR.at<uchar>(0,idx)= cloud->at(idx).r;
    }
    cv::Mat result;
    const int channels[1] = {0};
    const int histsize[1] = {256};
    float range[] = {0, 255};
    const float *ranges[] = {range};
    result.create(256,3,CV_32FC1);
    result.setTo(cv::Scalar::all(0.0));
    cv::Mat hist;
    double sum;
    cv::calcHist(&imgR,1,channels,cv::Mat(),hist,1,histsize,ranges,true,false);
    sum = cv::sum(hist)[0];
    if(sum!=0)hist*=(1.0/sum);
    hist.copyTo(result.col(0));
    cv::calcHist(&imgG,1,channels,cv::Mat(),hist,1,histsize,ranges,true,false);
    sum = cv::sum(hist)[0];
    if(sum!=0)hist*=(1.0/sum);
    hist.copyTo(result.col(1));
    cv::calcHist(&imgB,1,channels,cv::Mat(),hist,1,histsize,ranges,true,false);
    sum = cv::sum(hist)[0];
    if(sum!=0)hist*=(1.0/sum);
    hist.copyTo(result.col(2));
    return result;
}

cv::Mat Histogram::getBoxSize(FullPointCloud::Ptr cloud)
{
    cv::Mat box;
    Pipe::get2DOBB(cloud,box);
    cv::Mat result;
    result.create(3,1,CV_32FC1);
    double a = cv::norm(box.col(0) - box.col(1),cv::NORM_L2);
    double b = cv::norm(box.col(1) - box.col(2),cv::NORM_L2);
    result.at<float>(0,0) = std::max(a,b);
    result.at<float>(1,0) = std::min(a,b);
    unsigned int idx = 0;
    float zmax = std::numeric_limits<float>::lowest();
    float zmin = std::numeric_limits<float>::max();
    for(idx=0;idx<cloud->size();++idx)
    {
        float z = cloud->at(idx).z;
        if(z<zmin)zmin = z;
        if(z>zmax)zmax = z;
    }
    result.at<float>(2,0) = zmax - zmin;
    return result;
}

cv::Mat Histogram::getDifSize(FullPointCloud::Ptr cloud)
{
    cv::Mat box;
    Pipe::get2DOBB(cloud,box);
    cv::Mat result;
    result.create(6,1,CV_32FC1);
    double a = cv::norm(box.col(0) - box.col(1),cv::NORM_L2);
    double b = cv::norm(box.col(1) - box.col(2),cv::NORM_L2);
    result.at<float>(0,0) = std::max(a,b);
    result.at<float>(1,0) = std::min(a,b);
    unsigned int idx = 0;
    float zmax = std::numeric_limits<float>::lowest();
    float zmin = std::numeric_limits<float>::max();
    for(idx=0;idx<cloud->size();++idx)
    {
        float z = cloud->at(idx).z;
        if(z<zmin)zmin = z;
        if(z>zmax)zmax = z;
    }
    result.at<float>(2,0) = zmax - zmin;
    result.at<float>(3,0) = result.at<float>(0,0) - result.at<float>(1,0);
    result.at<float>(4,0) = result.at<float>(1,0) - result.at<float>(2,0);
    result.at<float>(5,0) = result.at<float>(2,0) - result.at<float>(0,0);
    return result;
}

cv::Mat Histogram::getDifSize(FullPointCloud::Ptr cloud,FullPoint &center)
{
    cv::Mat box;
    Pipe::get2DOBB(cloud,box);
    cv::Mat result;
    result.create(6,1,CV_32FC1);
    double a = cv::norm(box.col(0) - box.col(1),cv::NORM_L2);
    double b = cv::norm(box.col(1) - box.col(2),cv::NORM_L2);
    result.at<float>(0,0) = std::max(a,b);
    result.at<float>(1,0) = std::min(a,b);
    unsigned int idx = 0;
    center.x = ( box.at<float>(0,0) + box.at<float>(0,2) ) / 2.0;
    center.y = ( box.at<float>(1,0) + box.at<float>(1,2) ) / 2.0;
    float zmax = std::numeric_limits<float>::lowest();
    float zmin = std::numeric_limits<float>::max();
    for(idx=0;idx<cloud->size();++idx)
    {
        float z = cloud->at(idx).z;
        if(z<zmin)zmin = z;
        if(z>zmax)zmax = z;
    }
    center.z = ( zmin + zmax ) / 2.0;
    center.r = 255;
    center.g = 0;
    center.b = 0;
    result.at<float>(2,0) = zmax - zmin;
    result.at<float>(3,0) = 2*(result.at<float>(0,0) - result.at<float>(1,0));
    result.at<float>(4,0) = 2*(result.at<float>(1,0) - result.at<float>(2,0));
    result.at<float>(5,0) = 2*(result.at<float>(2,0) - result.at<float>(0,0));
    return result;
}

cv::Mat Histogram::getDifSize(FullPointCloud::Ptr cloud,FullPoint &center,FullPointCloud::Ptr& corners)
{
    cv::Mat box;
    Pipe::get2DOBBVar(cloud,box);
    cv::Mat result;
    result.create(6,1,CV_32FC1);
    double a = cv::norm(box.col(0) - box.col(1),cv::NORM_L2);
    double b = cv::norm(box.col(1) - box.col(2),cv::NORM_L2);
    result.at<float>(0,0) = std::max(a,b);
    result.at<float>(1,0) = std::min(a,b);
    unsigned int idx = 0;
    center.x = ( box.at<float>(0,0) + box.at<float>(0,2) ) / 2.0;
    center.y = ( box.at<float>(1,0) + box.at<float>(1,2) ) / 2.0;
    corners = FullPointCloud::Ptr(new FullPointCloud);
    float zmax = std::numeric_limits<float>::lowest();
    float zmin = std::numeric_limits<float>::max();
    for(idx=0;idx<cloud->size();++idx)
    {
        float z = cloud->at(idx).z;
        if(z<zmin)zmin = z;
        if(z>zmax)zmax = z;
    }
    double c = zmax - zmin;
    center.z = ( zmin + zmax ) / 2.0;
    center.r = 255;
    center.g = 0;
    center.b = 0;
    FullPoint p;
    p.r = 0;
    p.g = 255;
    p.b = 0;
    if(a>c&&b>c)
    {
        for(unsigned int idx=0;idx<4;++idx)
        {
            p.x = box.at<float>(0,idx);
            p.y = box.at<float>(1,idx);
            p.z = center.z;
            corners->push_back(p);
        }
    }else if(a>b&&c>b)
    {
        p.x = 0.5*(box.at<float>(0,0)+box.at<float>(0,3));
        p.y = 0.5*(box.at<float>(1,0)+box.at<float>(1,3));
        p.z = zmin;
        corners->push_back(p);
        p.z = zmax;
        corners->push_back(p);
        p.x = 0.5*(box.at<float>(0,1)+box.at<float>(0,2));
        p.y = 0.5*(box.at<float>(1,1)+box.at<float>(1,2));
        corners->push_back(p);
        p.z = zmin;
        corners->push_back(p);
    }else{
        p.x = 0.5*(box.at<float>(0,0)+box.at<float>(0,1));
        p.y = 0.5*(box.at<float>(1,0)+box.at<float>(1,1));
        p.z = zmin;
        corners->push_back(p);
        p.z = zmax;
        corners->push_back(p);
        p.x = 0.5*(box.at<float>(0,2)+box.at<float>(0,3));
        p.y = 0.5*(box.at<float>(1,2)+box.at<float>(1,3));
        corners->push_back(p);
        p.z = zmin;
        corners->push_back(p);
    }
    corners->height = 1;
    corners->width = corners->size();
    result.at<float>(2,0) = zmax - zmin;
    result.at<float>(3,0) = 2*(result.at<float>(0,0) - result.at<float>(1,0));
    result.at<float>(4,0) = 2*(result.at<float>(1,0) - result.at<float>(2,0));
    result.at<float>(5,0) = 2*(result.at<float>(2,0) - result.at<float>(0,0));
    return result;
}

cv::Mat Histogram::getNorm(FullPointCloud::Ptr cloud)
{
    Eigen::Vector4f plane;
    float curvature;
    pcl::computePointNormal<FullPoint>(*cloud,plane,curvature);
    cv::Mat result;
    result.create(3,1,CV_32FC1);
    result.at<float>(0,0) = plane(0);
    result.at<float>(1,0) = plane(1);
    result.at<float>(2,0) = plane(2);
    return result;
}

cv::Mat Histogram::getAbsCosineNormHist(FullPointCloud::Ptr cloud)
{
    Eigen::Vector4f plane;
    float curvature;
    pcl::computePointNormal<FullPoint>(*cloud,plane,curvature);
    cv::Mat img(1,cloud->size(),CV_32FC1);
    Eigen::Vector3f np;
    np(0) = plane(0);np(1) = plane(1);np(2) = plane(2);
    Eigen::Vector3f nc;
    unsigned int idx = 0;
    for(idx=0;idx<img.cols;++idx)
    {
        FullPoint& c = cloud->at(idx);
        nc(0) = c.normal_x;
        nc(1) = c.normal_y;
        nc(2) = c.normal_z;
        img.at<float>(0,idx) = abs(np.dot(nc));
    }
    const int channels[1] = {0};
    const int histsize[1] = {10};
    float range[] = {0.0,1.0};
    const float *ranges[] = {range};
    cv::Mat result;
    cv::calcHist(&img,1,channels,cv::Mat(),result,1,histsize,ranges,true,false);
    float sum;
    sum = cv::sum(result)[0];
    if(sum!=0)result*=(1.0/sum);
    return result;
}

double Histogram::getEuclidean(cv::Mat& l,cv::Mat&r)
{
    unsigned int c = l.cols;
    if(r.cols!=c)return std::numeric_limits<double>::max();
    unsigned int idx;
    double result=0.0;
    for(idx=0;idx<c;++idx)
    {
        result += cv::norm( l.col(idx) - r.col(idx) , cv::NORM_L2 );
    }
    return result;
}

double Histogram::getCosine(cv::Mat& l,cv::Mat& r)
{
    unsigned int c = l.cols;
    if(r.cols!=c)return std::numeric_limits<double>::max();
    unsigned int idx;
    double result=0.0;
    for(idx=0;idx<c;++idx)
    {
        cv::Mat tmpl,tmpr;
        double norml = cv::norm(l.col(idx),cv::NORM_L2);
        double normr = cv::norm(r.col(idx),cv::NORM_L2);

        l.col(idx).copyTo(tmpl);
        r.col(idx).copyTo(tmpr);

        result += tmpl.dot(tmpr) / (norml*normr);
    }
    return result;
}
