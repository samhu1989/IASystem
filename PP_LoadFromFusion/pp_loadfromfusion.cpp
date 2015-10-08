#include "pp_loadfromfusion.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include "region_growing.h"
Pipe* createPipe(void)
{
    return new PP_LoadFromFusion();
}

PP_LoadFromFusion::PP_LoadFromFusion()
{

}

PP_LoadFromFusion::~PP_LoadFromFusion()
{

}

int PP_LoadFromFusion::configure(Config& config)
{
    if(config.has("Frame_Path"))
    {
        _FramePath = config.getString("Frame_Path");
    }else{
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Suffix"))
    {
        _Suffix = config.getString("Suffix");
    }else{
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Frame_Num"))
    {
        _FrameNum = config.getInt("Frame_Num");
    }else{
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Model_Grid_Size"))
    {
        ICP::DOWN_SAMPLE_GRID_SIZE = config.getFloat("Model_Grid_Size");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Angle_Threshold"))
    {
        _Angle_Threshold = config.getFloat("Angle_Threshold");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Curve_Threshold"))
    {
        _Curve_Threshold = config.getFloat("Curve_Threshold");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Plane_Error"))
    {
        _PlaneError = config.getFloat("Plane_Error");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("K_Neighbor"))
    {
        _K_Neighbor = config.getInt("K_Neighbor");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("R_Neighbor"))
    {
        _R_Neighbor = config.getFloat("R_Neighbor");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Dist_To_Wall"))
    {
        _Dist2Wall = config.getFloat("Dist_To_Wall");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Bg_Plane_Num"))
    {
        _Bg_Plane_Num = config.getInt("Bg_Plane_Num");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    return 0;
}

int PP_LoadFromFusion::init(void)
{
    _Bg = FullPointCloud::Ptr(new FullPointCloud);
    while(_FgList.size() < _FrameNum)
    {
        _FgList.push_back(FullPointCloud::Ptr(new FullPointCloud));
    }
    return 0;
}

int PP_LoadFromFusion::work(void)
{
    loadFrame();
    segFrameRegionGrow();
    return 0;
}

int PP_LoadFromFusion::saveToData(void)
{
    inform("Saving Data");
    pcl::search::KdTree<FullPoint>::Ptr tree (new pcl::search::KdTree<FullPoint> ());
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(0,0,0);
    est.setInputCloud(_Bg);
    if(_K_Neighbor==0)est.setRadiusSearch(_R_Neighbor);
    else est.setKSearch(_K_Neighbor);
    est.compute(*_Bg);
    addToData(_Bg,_BGKey,QString("Background"));

    unsigned int idx;
    QList<quint32> _FrameKeyList;
    QList<quint32> _IdMapKeyList;
    for(idx=0;idx<_FgList.size();++idx)
    {
        QString info;
        info = info.sprintf("Foreground_%d",idx);
        est.setInputCloud(_FgList[idx]);
        est.compute(*_FgList[idx]);
        quint32 fkey = addToData(_FgList[idx],info);
        info = info.sprintf("IdMap_%d",idx);
        quint32 idkey = addToData(_IdMapList[idx],info);
        _FrameKeyList.push_back(fkey);
        _IdMapKeyList.push_back(idkey);
    }
    addToData(_FrameKeyList,_FrameListKey,QString("Frame_Key_List"));
    addToData(_IdMapKeyList,_IdMapListKey,QString("IdMap_Key_List"));
    return 0;
}

void PP_LoadFromFusion::loadFrame(void)
{
    unsigned int frame_idx;
    unsigned int planeidx;
    for(frame_idx=1;frame_idx <= _FrameNum;++frame_idx)
    {
        loadFrame(frame_idx);
        segFloor(_FgList[frame_idx-1]);
        for(planeidx=0;planeidx<_Bg_Plane_Num-1;++planeidx)
        {
            segWall(_FgList[frame_idx-1]);
        }
    }
    return ;
}

void PP_LoadFromFusion::loadFrame(unsigned int frameIdx)
{
    std::stringstream stream("");
    std::string path;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    FullPointCloud::Ptr tmpfull(new FullPointCloud);
    pcl::search::KdTree<FullPoint>::Ptr tree (new pcl::search::KdTree<FullPoint> ());
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(0,0,0);
    if(_K_Neighbor==0)est.setRadiusSearch(_R_Neighbor);
    else est.setKSearch(_K_Neighbor);

    stream.clear();
    stream<<_FramePath<<frameIdx<<"."<<_Suffix;
    stream>>path;
    Eigen::Matrix4f t;

    if(_Suffix=="pcd"||_Suffix=="PCD")pcl::io::loadPCDFile(path,*tmp);
    else pcl::io::loadPLYFile(path,*tmp);
    tmpfull->clear();
    pcl::copyPointCloud(*tmp,*tmpfull);
    (*_FgList[frameIdx-1])+=*tmpfull;

    ICP::DownSampler down;
    down.downSampled->clear();
    down(_FgList[frameIdx-1]);
    (*_FgList[frameIdx-1]) = (*down.downSampled);

    est.setInputCloud(_FgList[frameIdx-1]);
    est.compute(*_FgList[frameIdx-1]);
    std::vector<int> idx;

    pcl::removeNaNFromPointCloud<FullPoint>(*_FgList[frameIdx-1],idx);
    return ;
}

void PP_LoadFromFusion::segFloor(FullPointCloud::Ptr& pcd)
{
    FullPointCloud::Ptr cloud_p (new FullPointCloud);
    FullPointCloud::Ptr cloud_f (new FullPointCloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<FullPoint> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/18);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (_PlaneError);
    pcl::ExtractIndices<FullPoint> extract;
    seg.setInputCloud (pcd);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    // Extract the inliers
    extract.setInputCloud (pcd);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    *_Bg+=*cloud_p;

    extract.setNegative (true);
    cloud_f->clear();
    extract.filter (*cloud_f);
    *pcd = *cloud_f;

    ICP::DownSampler down;
    down(_Bg);
    (*_Bg) = (*down.downSampled);
}

void PP_LoadFromFusion::segWall(FullPointCloud::Ptr& pcd)
{
    FullPointCloud::Ptr cloud_p (new FullPointCloud);
    FullPointCloud::Ptr cloud_f (new FullPointCloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<FullPoint> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/36);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (_PlaneError);

    pcl::ExtractIndices<FullPoint> extract;
    seg.setInputCloud (pcd);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    // Extract the inliers
    extract.setInputCloud (pcd);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    *_Bg+=*cloud_p;

    // Create the filtering object
    extract.setNegative (true);
    cloud_f->clear();
    extract.filter (*cloud_f);
    *pcd = *cloud_f;

    ICP::DownSampler down;
    down(_Bg);
    (*_Bg) = (*down.downSampled);
}

void PP_LoadFromFusion::segFrameRegionGrow(void)
{
    pcl::search::Search<FullPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<FullPoint> > (new pcl::search::KdTree<FullPoint>);
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max()
                );
    if(_K_Neighbor!=0)est.setKSearch(_K_Neighbor);
    else est.setRadiusSearch(_R_Neighbor);

    pcl::RegionGrowing<FullPoint,FullPoint> reg;

    reg.setMinClusterSize(10);
    reg.setMaxClusterSize(std::numeric_limits<int>::max());
    reg.setSearchMethod(tree);
    if(_K_Neighbor!=0)reg.setNumberOfNeighbours(_K_Neighbor);
    else reg.setRadiusOfNeighbours(_R_Neighbor);
    reg.setSmoothnessThreshold( _Angle_Threshold/180.0*M_PI );
    reg.setCurvatureThreshold( _Curve_Threshold );

    unsigned int idx;
    for(idx = 0; idx < _FgList.size() ;++idx)
    {

       est.setInputCloud(_FgList[idx]);
       est.compute(*_FgList[idx]);
       reg.setInputCloud(_FgList[idx]);
       reg.setInputNormals(_FgList[idx]);

       std::vector<pcl::PointIndices> clusters;
       reg.extract(clusters);

       cv::Mat idmap;
       idmap.create(1,_FgList[idx]->size(),CV_32SC1);
       idmap.setTo(-1);
       unsigned int cnt = 0;

       while( cnt < clusters.size() )
       {
           std::vector<int>::iterator iter;
           iter = clusters[cnt].indices.begin();
           for(;iter!=clusters[cnt].indices.end();++iter)
           {
               int idx;
               idx = *iter;
               idmap.at<__int32>(0,idx) = cnt+1;
           }
           ++cnt;
       }

       _IdMapList.push_back(cv::Mat());
       idmap.copyTo(_IdMapList.back());
    }
    return ;
}
