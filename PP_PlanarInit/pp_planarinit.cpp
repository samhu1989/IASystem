#include "pp_planarinit.h"
#include <QFile>
#include <QTextStream>
#include <QDir>
#include  "icp.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
Pipe* createPipe(void)
{
    return new PP_PlanarInit();
}

PP_PlanarInit::PP_PlanarInit()
{

}

int PP_PlanarInit::configure(Config&config)
{
    if(config.has("Seg_Image_Path"))
    {
        _Seg_Image_Path = config.getString("Seg_Image_Path");
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
    if(config.has("Frame_Path"))
    {
        _FramePath = config.getString("Frame_Path");
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
    if(config.has("Frame_Num"))
    {
        _FrameNum = config.getInt("Frame_Num");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Cam_File"))
    {
        _CamPath = config.getString("Cam_File");
    } else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Cam_Num"))
    {
        _CamNum = config.getInt("Cam_Num");
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

int PP_PlanarInit::init(void)
{
    _Bg = FullPointCloud::Ptr(new FullPointCloud);
    while( _FrameNum > _DepthMapList.size() )
    {
        _DepthMapList.push_back(std::vector<cv::Mat>());
    }
    while(_FgList.size() < _FrameNum)
    {
        _FgList.push_back(FullPointCloud::Ptr(new FullPointCloud));
    }
    return 0;
}

int PP_PlanarInit::work(void)
{
    loadCam();
    loadFrame();
    Pipe::inform("Planarizing");
    segFramePlanarize();
    return 0;
}

int PP_PlanarInit::saveToData(void)
{
    inform("Saving Data");
    pcl::search::KdTree<FullPoint>::Ptr tree (new pcl::search::KdTree<FullPoint> ());
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(0,0,0);
    est.setInputCloud(_Bg);
    est.setRadiusSearch(5*ICP::DOWN_SAMPLE_GRID_SIZE);
    est.compute(*_Bg);
    addToData(_Bg,_BGKey,QString("Background"));

    if(!_Seg_Image_Path.empty())
    {
        QDir dir;
        dir.setPath(QString::fromStdString(_Seg_Image_Path));
        QString filename;
        Eigen::Matrix3f intri;
        unsigned int r,c;
        for(r=0;r<3;++r)
            for(c=0;c<3;++c)
            {
                intri(r,c) = _CamIn.at<float>(r,c);
            }
        unsigned int idx;
        for(idx=0;idx<_IdMapList.size();++idx)
        {
            filename = filename.sprintf("Frame_%d",idx)+_Time.toString("_hh_mm_ss_zzz")+".png";
            IdMap2Img(_Bg,_FgList[idx],_IdMapList[idx],_CamExList,intri,dir.filePath(filename).toStdString());
        }
    }

    unsigned int idx;
    QList<quint32> _FrameKeyList;
    QList<quint32> _IdMapKeyList;
    for(idx=0;idx<_FgList.size();++idx)
    {
        QString info;
        info = info.sprintf("Foreground_%d",idx);
        quint32 fkey = addToData(_FgList[idx],info);
        info = info.sprintf("IdMap_%d",idx);
        quint32 idkey = addToData(_IdMapList[idx],info);
        _FrameKeyList.push_back(fkey);
        _IdMapKeyList.push_back(idkey);
    }
    addToData(_FrameKeyList,_FrameListKey,QString("Frame_Key_List"));
    addToData(_IdMapKeyList,_IdMapListKey,QString("IdMap_Key_List"));
    QList<quint32> _CamExKeyList;
    for(idx=0;idx<_CamExList.size();++idx)
    {
        QString info;
        info = info.sprintf("Cam_%d",idx);
        _CamExKeyList.push_back(addToData(_CamExList[idx],info));
    }
    addToData(_CamExKeyList,_CamExKey,QString("CamEx_Key_List"));
    addToData(_CamIn,_CamKey,QString("Camera_Intrinsic"));

    unsigned int f,v;

    _DepthMapKeyList.resize(_DepthMapList.size());

    std::stringstream stream;
    std::string path;

    for(f=0;f<_DepthMapList.size();++f)
    {
//        if(!_Seg_Image_Path.empty())
//        {
//            stream.clear();
//            stream<<_Seg_Image_Path <<"Depth"<<f<<"_";
//            stream>>path;
//            debug(_DepthMapList[f],path);
//        }
        for(v=0;v<_DepthMapList[f].size();++v)
        {
            unsigned int key;
            QString msg;
            msg = msg.sprintf("Depth_f%d_v%d",f,v);
            key = addToData(_DepthMapList[f][v],msg);
            _DepthMapList[f][v].release();
            _DepthMapKeyList[f].push_back(key);
        }
    }

    addToData(_DepthMapKeyList,_DepthMapKey,QString("DepthMap_Key_List"));
    return 0;
}

void PP_PlanarInit::loadFrame(void)
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

void PP_PlanarInit::loadFrame(unsigned int frameIdx)
{
    unsigned int cam_idx;
    std::stringstream stream("");
    std::string path;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    FullPointCloud::Ptr tmpfull(new FullPointCloud);
    pcl::search::KdTree<FullPoint>::Ptr tree (new pcl::search::KdTree<FullPoint> ());
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(0,0,0);
    est.setRadiusSearch(10*ICP::DOWN_SAMPLE_GRID_SIZE);
    for(cam_idx=1;cam_idx<=_CamNum;++cam_idx)
    {
        stream.clear();
        stream<<_FramePath<<frameIdx<<"_"<<cam_idx<<".pcd";
        stream>>path;
        Eigen::Matrix4f t;

        pcl::io::loadPCDFile(path,*tmp);
        tmpfull->clear();
        pcl::copyPointCloud(*tmp,*tmpfull);
        (*_FgList[frameIdx-1])+=*tmpfull;

        ICP::DownSampler down;
        down.downSampled->clear();
        down(_FgList[frameIdx-1]);
        (*_FgList[frameIdx-1]) = (*down.downSampled);

        est.setInputCloud(_FgList[frameIdx-1]);
        est.compute(*_FgList[frameIdx-1]);

        getT(_CamExList[cam_idx-1],t);
        pcl::transformPointCloud<FullPoint>(*tmpfull,*tmpfull,t);
        std::cerr<<frameIdx<<std::endl;
        addToDepthMap(tmpfull,frameIdx-1);
    }
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud<FullPoint>(*_FgList[frameIdx-1],idx);
    return ;
}

void PP_PlanarInit::loadCam(void)
{
    unsigned int cam_idx;
    cv::Mat ex;
    ex.create(4,4,CV_32FC1);
    Eigen::Matrix4f camEx;
    std::stringstream stream("");
    std::string path;
    for(cam_idx=1;cam_idx<=_CamNum;++cam_idx)
    {
        stream.clear();
        stream<<_CamPath<<cam_idx<<".txt";
        stream>>path;
        loadCam(path,camEx);
        getMat(camEx,ex);
        _CamExList.push_back(cv::Mat());
        ex.copyTo(_CamExList.back());
    }
    return ;
}

void PP_PlanarInit::segPlane(FullPointCloud::Ptr& pcd)
{
    FullPointCloud::Ptr cloud_p (new FullPointCloud);
    FullPointCloud::Ptr cloud_f (new FullPointCloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentationFromNormals<FullPoint,FullPoint> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType (pcl::SAC_RRANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (_PlaneError);
    seg.setNormalDistanceWeight(0.1);
//    pcl::search::Search<FullPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<FullPoint> > (new pcl::search::KdTree<FullPoint>);
//    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
//    est.setSearchMethod (tree);
//    est.setViewPoint(0,0,0);
//    est.setKSearch(_K_Neighbor);
//    est.setInputCloud(pcd);
//    est.compute(*pcd);
    pcl::ExtractIndices<FullPoint> extract;
    // While 30% of the original cloud is still there
    // Segment the largest planar component from the remaining cloud
    seg.setInputNormals(pcd);
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

void PP_PlanarInit::segFloor(FullPointCloud::Ptr& pcd)
{
    FullPointCloud::Ptr cloud_p (new FullPointCloud);
    FullPointCloud::Ptr cloud_f (new FullPointCloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentationFromNormals<FullPoint,FullPoint> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/18);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (_PlaneError);
    seg.setNormalDistanceWeight(0.1);
//    pcl::search::Search<FullPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<FullPoint> > (new pcl::search::KdTree<FullPoint>);
//    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
//    est.setSearchMethod (tree);
//    est.setViewPoint(0,0,0);
//    est.setKSearch(_K_Neighbor);
//    est.setInputCloud(pcd);
//    est.compute(*pcd);
    pcl::ExtractIndices<FullPoint> extract;
    // While 30% of the original cloud is still there
    // Segment the largest planar component from the remaining cloud
    seg.setInputNormals(pcd);
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

void PP_PlanarInit::segWall(FullPointCloud::Ptr& pcd)
{
    FullPointCloud::Ptr cloud_p (new FullPointCloud);
    FullPointCloud::Ptr cloud_f (new FullPointCloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentationFromNormals<FullPoint,FullPoint> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/18);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (_PlaneError);
    seg.setNormalDistanceWeight(0.1);
//    pcl::search::Search<FullPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<FullPoint> > (new pcl::search::KdTree<FullPoint>);
//    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
//    est.setSearchMethod (tree);
//    est.setViewPoint(0,0,0);
//    est.setKSearch(_K_Neighbor);
//    est.setInputCloud(pcd);
//    est.compute(*pcd);
    pcl::ExtractIndices<FullPoint> extract;
    // While 30% of the original cloud is still there
    // Segment the largest planar component from the remaining cloud
    seg.setInputNormals(pcd);
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

void PP_PlanarInit::segFramePlanarize(void)
{
    pcl::search::Search<FullPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<FullPoint> > (new pcl::search::KdTree<FullPoint>);
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(0,0,0);
    est.setRadiusSearch(5*ICP::DOWN_SAMPLE_GRID_SIZE);

    pcl::SACSegmentationFromNormals<FullPoint,FullPoint> seg;
//    pcl::SACSegmentation<FullPoint> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/18);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setNormalDistanceWeight(0.2);
    seg.setRadiusLimits(0.5*ICP::DOWN_SAMPLE_GRID_SIZE,2*ICP::DOWN_SAMPLE_GRID_SIZE);
    seg.setDistanceThreshold (_PlaneError);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());

    unsigned int idx;
    for(idx = 0; idx < _FgList.size() ;++idx)
    {
       cv::Mat idmap;
       idmap.create(1,_FgList[idx]->size(),CV_32SC1);
       idmap.setTo(-1);
//       est.setInputCloud(_FgList[idx]);
//       est.compute(*_FgList[idx]);
       std::cerr<<"size:"<<_FgList[idx]->size()<<std::endl;
       seg.setInputCloud(_FgList[idx]);
       seg.setInputNormals(_FgList[idx]);
       outliers->indices.clear();
       int p=0;
       while(p<idmap.cols){
           outliers->indices.push_back(p);
           ++p;
       }
       unsigned int cnt=0;
       do{
           pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients ());
           inliers->indices.clear();
//           std::cerr<<"unseg:"<<outliers->indices.size()<<std::endl;
           seg.setIndices(outliers);
           seg.segment(*inliers,*coeff);
//           std::cerr<<"newseg:"<<inliers->indices.size()<<std::endl;
           std::vector<int>::iterator iter;
           int pidx;
           for(iter=inliers->indices.begin();iter!=inliers->indices.end();++iter)
           {
               pidx = *iter;
               idmap.at<__int32>(0,pidx) = cnt+1;
           }
           ++cnt;
           int outidx;
           outliers->indices.clear();
           for(outidx=0;outidx<idmap.cols;++outidx)
           {
               if(-1==idmap.at<__int32>(0,outidx))
               {
                   outliers->indices.push_back(outidx);
               }
           }
       }while(inliers->indices.size()>300);
       _IdMapList.push_back(cv::Mat());
       idmap.copyTo(_IdMapList.back());
    }
    return ;
}

void PP_PlanarInit::loadCam(const std::string& path, Eigen::Matrix4f& _CamEx)
{
    QFile file;
    file.setFileName(QString::fromStdString(path));
    file.open(file.ReadOnly);
    if(!file.isOpen()){
        error("Can't open file:"+path);
        return;
    }
    std::cerr<<file.fileName().toStdString()<<std::endl;

    QTextStream stream(&file);
    Eigen::Matrix3f intrinsics = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f _ExR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f _ExT(0,0,0);

    stream  >> intrinsics(0,0) >> intrinsics(0,1) >> intrinsics(0,2)
            >> intrinsics(1,0) >> intrinsics(1,1) >> intrinsics(1,2)
            >> intrinsics(2,0) >> intrinsics(2,1) >> intrinsics(2,2);
    // Extrinsic: Rotation
    stream  >>_ExR(0,0) >>_ExR(0,1) >>_ExR(0,2)
            >>_ExR(1,0) >>_ExR(1,1) >>_ExR(1,2)
            >>_ExR(2,0) >>_ExR(2,1) >>_ExR(2,2);
    //Translation
    stream >> _ExT(0) >> _ExT(1) >> _ExT(2);
    _CamEx.block(0,0,3,3) = _ExR;
    _CamEx.block(0,3,3,1) = _ExT;
    _CamEx(3,3) = 1.0;
    //overwrite the _CamEx with view matrix

    cv::Mat camIntrinsics;
    camIntrinsics.create(3,3,CV_32FC1);
    unsigned int r,c;
    for(r=0;r<camIntrinsics.rows;++r)
    {
        for(c=0;c<camIntrinsics.cols;++c)
        {
            camIntrinsics.at<float>(r,c) = intrinsics(r,c);
        }
    }
    camIntrinsics.copyTo(_CamIn);
    file.close();
}

void PP_PlanarInit::addToDepthMap(const FullPointCloud::Ptr& pcd,unsigned int frameIdx)
{
    Pipe::inform("Adding to Depth Map");
    unsigned int r,c;
    cv::Mat depth;
    depth.create(pcd->height,pcd->width,CV_32FC1);
    std::cerr<<pcd->height<<","<<pcd->width<<std::endl;
    for(r=0;r<pcd->height;++r)
        for(c=0;c<pcd->width;++c)
        {
            if(pcd->height>1)depth.at<float>(r,c) = pcd->at(c,r).z;
            else{
                depth.at<float>(r,c) = pcd->at(c).z;
            }
        }
    if(frameIdx>_DepthMapList.size())
    {
        error("Exceed the size of std::vector");
    }
    _DepthMapList[frameIdx].push_back(cv::Mat());
    depth.copyTo(_DepthMapList[frameIdx].back());
}

void PP_PlanarInit::segWalls(FullPointCloud::Ptr& ptr)
{
    inform("segWall");
    FullPointCloud::Ptr wall(new FullPointCloud);
    FullPointCloud::Ptr fg(new FullPointCloud);
    if(_2DBB.empty())
    {
        Pipe::get2DOBB(_Bg,_2DBB);
    }
    unsigned int idx;
    for(idx=0;idx<ptr->size();++idx)
    {
        if( isWithinWall( ptr->at(idx) ))
        {
            fg->push_back( ptr->at(idx) );
        }else{
            wall->push_back( ptr->at(idx) );
        }
    }
    *_Bg+=*wall;
    *ptr = *fg;
    ICP::DownSampler down;
    down(_Bg);
    (*_Bg) = (*down.downSampled);
    down.downSampled->clear();
    down(fg);
    (*ptr) = (*down.downSampled);
}

bool PP_PlanarInit::isWithinWall(FullPoint& p)
{
    Eigen::Vector2f v;
    v(0) = p.x;
    v(1) = p.y;

    Eigen::Vector2f p0;
    p0(0) = _2DBB.at<float>(0,0);
    p0(1) = _2DBB.at<float>(1,0);

    Eigen::Vector2f p1;
    p1(0) = _2DBB.at<float>(0,1);
    p1(1) = _2DBB.at<float>(1,1);

    Eigen::Vector2f p3;
    p3(0) = _2DBB.at<float>(0,3);
    p3(1) = _2DBB.at<float>(1,3);

    Eigen::Vector2f p0v;
    Eigen::Vector2f p01;
    Eigen::Vector2f p03;
    p0v = v - p0;
    p01 = p1 - p0;
    p03 = p3 - p0;

    float product01 = p0v.dot(p01) / p01.norm();
    float product03 = p0v.dot(p03) / p03.norm();

    if( product01 < _Dist2Wall || product01 >  p01.norm() - _Dist2Wall  )
    {
        return false;
    }

    if( product03 < _Dist2Wall || product03 > p03.norm() - _Dist2Wall   )
    {
        return false;
    }

    return true;
}
