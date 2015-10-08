#include "pp_scenegraph.h"
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common_headers.h>
#include <QImage>
#include <strstream>
#include <iomanip>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/poisson.h>
Pipe* createPipe(void)
{
    return new PP_SceneGraph();
}

PP_SceneGraph::PP_SceneGraph()
{

}

int PP_SceneGraph::configure(Config& config)
{
    if(config.has("Yan_Meng_Path"))
    {
        _Yan_Meng_Path = config.getString("Yan_Meng_Path");
        inform("Yan_Meng_Path="+_Yan_Meng_Path);
    }else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Frame_Num"))
    {
        _FrameNum = config.getInt("Frame_Num");
    }else {
        NECESSARY_CONFIG();
        return -1;
    }
    if(config.has("Dump_Source_Path"))
    {
        _Dump_Source_Path = config.getString("Dump_Source_Path");
        inform("Dump_Source_Path="+_Dump_Source_Path);
        return 0;
    }else if(!_PipeData.empty()){
        inform("PipeData is already loaded");
        return 0;
    }
    NECESSARY_CONFIG();
    return -1;
}

int PP_SceneGraph::init(void)
{
    if(_PipeData.empty())
    {
        loadPipeDataFrom(_Dump_Source_Path);
    }
    return 0;
}

int PP_SceneGraph::work(void)
{
    getFloor();
    getBoundingBox();
    outputBox();
    getProximate();
    outputObj();
    return 0;
}

int PP_SceneGraph::saveToData(void)
{
    return 0;
}

void PP_SceneGraph::getFloor(void)
{
    FullPointCloud::Ptr ptr(new FullPointCloud);
    QString info;
    Pipe::loadData(ptr,info,Pipe::_BGKey);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<FullPoint> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (ptr);
    seg.segment (*inliers, _FloorCoeff);
}

void PP_SceneGraph::getBoundingBox(void)
{
    FullPointCloud::Ptr obj(new FullPointCloud);
    QString info;
    QList<quint32> objList;
    Pipe::loadData(objList,info,Pipe::_ObjListKey);

    cv::Mat tmp;
    obj->clear();
    Pipe::loadData(obj,info,_BGKey);
    getBoundingBox(obj,tmp);
    _BoundingBoxList.push_back(cv::Mat());
    tmp.copyTo(_BoundingBoxList.back());

    foreach(quint32 key,objList)
    {
        cv::Mat tmp;
        obj->clear();
        Pipe::loadData(obj,info,key);
        getBoundingBox(obj,tmp);
        _BoundingBoxList.push_back(cv::Mat());
        tmp.copyTo(_BoundingBoxList.back());
    }
}

void PP_SceneGraph::getBoundingBox( const FullPointCloud::Ptr& obj, cv::Mat& vertexes )
{
    vertexes.create(3,8,CV_32FC1);
    cv::Mat vertex2d;
    Pipe::get2DOBB(obj,vertex2d);
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::min();
    unsigned int idx;
    for(idx=0;idx<obj->size();++idx)
    {
        if(minZ>obj->at(idx).z)minZ=obj->at(idx).z;
        if(maxZ<obj->at(idx).z)maxZ=obj->at(idx).z;
    }
    idx=0;
    for(idx=0;idx<4;++idx)
    {
        vertexes.at<float>(0,idx) = vertex2d.at<float>(0,idx);
        vertexes.at<float>(1,idx) = vertex2d.at<float>(1,idx);
        vertexes.at<float>(2,idx) = minZ;
    }
    for(idx=0;idx<4;++idx)
    {
        vertexes.at<float>(0,idx+4) = vertex2d.at<float>(0,idx);
        vertexes.at<float>(1,idx+4) = vertex2d.at<float>(1,idx);
        vertexes.at<float>(2,idx+4) = maxZ;
    }
}

void PP_SceneGraph::getProximate(void)
{
    unsigned int idx;
    std::cerr<<_FrameNum<<std::endl;
    for(idx=0;idx<_FrameNum;++idx)
    {
        std::vector<cv::Mat>layouts;
        std::vector<unsigned int>obj_id;
        QImage debug(2000,1500,QImage::Format_ARGB32);
        layouts.clear();
        getSceneLayout(idx,layouts,obj_id,debug);
        outputScene(idx,layouts,obj_id);
        QString filename;
        filename = ".\\log\\"+filename.sprintf("%d_%d",_Time.msec(),rand())+".png";
        debug.save(filename);
    }
}

void PP_SceneGraph::outputScene(unsigned int FrameIdx,const std::vector<cv::Mat>&layouts,const std::vector<unsigned int>&objIdx)
{
    std::fstream objfile;
    std::stringstream stream("");
    std::string objfilename;
    stream<<_Yan_Meng_Path<<"scene_"<<FrameIdx<<".obj";
    stream>>objfilename;
    objfile.open(objfilename,objfile.out);
    unsigned int idx,c;
    unsigned int cnt = 1;
    inform("Output Scene");
    std::cerr<<objfilename<<std::endl;
    for(idx=0;idx<layouts.size();++idx)
    {
        objfile<<"#obj "<<objIdx[idx]<<std::endl;
        cv::Mat layout = layouts[idx];
        for(c=0;c<layout.cols;++c)
        {
            objfile<<"v "<<layout.at<float>(0,c)
                       <<" "<<layout.at<float>(1,c)
                       <<" "<<layout.at<float>(2,c)
                       <<std::endl;
        }
        objfile<<"f "<<cnt+0<<" "<<cnt+1<<" "<<cnt+2<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+4<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+7<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+5<<" "<<cnt+1<<std::endl;
        objfile<<"f "<<cnt+1<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+2<<std::endl;
        objfile<<"f "<<cnt+2<<" "<<cnt+6<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        cnt+=8;
    }
    objfile.close();
}

void PP_SceneGraph::getProximate(std::vector<cv::Mat>&layouts,std::vector<unsigned int>&objIdx,QImage&debug)
{
    cv::Mat proxMat;
    Eigen::MatrixXf centers;
    getLayoutCenter(layouts,centers);
    getProximate(centers,proxMat);
    debug2DRelation(centers,proxMat,debug);
}

void PP_SceneGraph::getProximate(const Eigen::MatrixXf& centers,cv::Mat& relation)
{
    unsigned int i,j;
    Eigen::MatrixXf angleRelation(centers.cols(),centers.cols());
    Eigen::MatrixXf distRelation(centers.cols(),centers.cols());
    for(i=0;i<centers.cols();++i)
    {
        for(j=i;j<centers.cols();++j)
        {
            float dX = centers(0,i) - centers(0,j);
            float dY = centers(1,i) - centers(1,j);
            float dist = std::sqrtf(dX*dX+dY*dY);
            distRelation(i,j) = 1.0/(1.0+dist);
            distRelation(j,i) = 1.0/(1.0+dist);
        }
    }
    relation.create(centers.cols(),centers.cols(),CV_32FC1);
    getMat(distRelation,relation);
    std::vector<unsigned int> closest(centers.cols(),0);
    std::vector<float> closestRelation(centers.cols(),0.0);
    for(i=0;i<centers.cols();++i)
    {
        for(j=0;j<centers.cols();++j)
        {
            if(j==i)continue;
            if( closestRelation[i]<distRelation(i,j) )
            {
                closest[i] = j;
                closestRelation[i] = distRelation(i,j);
            }
        }
    }
    for(i=0;i<centers.cols();++i)
    {
        Eigen::Vector2f  zeroAxis;
        zeroAxis(0) = centers(0,closest[i]) - centers(0,i);
        zeroAxis(1) = centers(1,closest[i]) - centers(1,i);
        for(j=0;j<centers.cols();++j)
        {
            if(i==j){
                angleRelation(i,j)=0;
                continue;
            }
            Eigen::Vector2f dir;
            dir(0) = centers(0,j) - centers(0,i);
            dir(1) = centers(1,j) - centers(1,i);
        }
    }
}

void PP_SceneGraph::getLayoutCenter(std::vector<cv::Mat>&layouts,Eigen::MatrixXf& centers)
{
    unsigned int idx;
    centers = Eigen::MatrixXf(3,layouts.size());
    for(idx=0;idx<layouts.size();++idx)
    {
       Eigen::Vector3f c;
       getLayoutCenter(layouts[idx],c);
       centers.col(idx) = c;
    }
}

void PP_SceneGraph::getLayoutCenter(const cv::Mat&layout,Eigen::Vector3f& center)
{
    center = Eigen::Vector3f::Zero();
    unsigned int idx;
    for(idx=0;idx<layout.cols;++idx)
    {
        center(0)+=layout.at<float>(0,idx);
        center(1)+=layout.at<float>(1,idx);
        center(2)+=layout.at<float>(2,idx);
    }
    center /= float(layout.cols);
}

void PP_SceneGraph::transformBoundingBox(const std::vector<std::vector<cv::Mat>>&transList,std::vector<cv::Mat>&layouts,std::vector<unsigned int>&obj_id,QImage&debug)
{
    unsigned int idx;
    unsigned int t;
    QString info;
    /*debug=>*/
    std::vector<FullPointCloud::Ptr> transformedPCD;
    QList<quint32> objList;
    Pipe::loadData(objList,info,Pipe::_ObjListKey);
    /*<=debug*/
    for(idx=0;idx<transList.size();++idx)
    {
        for(t=0;t<transList[idx].size();++t)
        {
            cv::Mat transMat;
            cv::Mat box;

            transList[idx][t].copyTo(transMat);
            _BoundingBoxList[idx].copyTo(box);
            Pipe::transformOBB(transMat,box);
            /*debug=>*/
            FullPointCloud::Ptr cloud(new FullPointCloud);
            FullPointCloud::Ptr tcloud(new FullPointCloud);

            if(idx==0)
            {
                Pipe::loadData(cloud,info,_BGKey);
            }else{
                Pipe::loadData(cloud,info,objList[idx-1]);
            }

            Eigen::Matrix4f t;
            Eigen::Matrix4f invt;
            getT(transMat,t);
            invt = t.inverse();
            pcl::transformPointCloudWithNormals<FullPoint>(*cloud,*tcloud,invt);
            transformedPCD.push_back(tcloud);
            /*<=debug*/

            transMat.release();
            layouts.push_back(box);
            obj_id.push_back(idx);
        }
    }
    Pipe::debug2DOBB(transformedPCD,layouts,obj_id,debug);
}

void PP_SceneGraph::getSceneLayout(unsigned int frameNum, std::vector<cv::Mat>&layouts, std::vector<unsigned int> &obj_id, QImage &debug)
{
    unsigned int objIdx;
    QVector<QVector<quint32>> objScoreList;
    std::vector<std::vector<cv::Mat>> transList;
    QString info;
    Pipe::loadData(objScoreList,info,Pipe::_ObjScoreKey);

    cv::Mat identity;
    identity.create(4,4,CV_32FC1);
    transList.push_back(std::vector<cv::Mat>());
    transList[0].push_back(cv::Mat());
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    getMat(t,identity);
    identity.copyTo(transList[0].back());

    for(objIdx=0;objIdx<objScoreList.size();++objIdx)
    {
        QVector<quint32>& list = objScoreList[objIdx] ;
        unsigned int tIdx;
        cv::Mat frameList;
        Pipe::loadData(frameList,info,list.back());
        transList.push_back(std::vector<cv::Mat>());
        for(tIdx=0;tIdx<frameList.cols;++tIdx)
        {
            if(frameList.at<int>(tIdx)==frameNum)
            {
                cv::Mat trans;
                Pipe::loadData(trans,info,list[tIdx+1]);
                transList[objIdx+1].push_back(cv::Mat());
                trans.copyTo(transList[objIdx+1].back());
            }
        }
    }
    transformBoundingBox(transList,layouts,obj_id,debug);
}

void PP_SceneGraph::outputBox(void)
{
    std::fstream objfile;
    std::stringstream stream("");
    std::string objfilename;

    unsigned int idx,c;
    unsigned int cnt = 1;

    for(idx=0;idx<_BoundingBoxList.size();++idx)
    {
        stream.clear();
        stream<<_Yan_Meng_Path<<"obj_box"<<idx<<".obj";
        stream>>objfilename;
        objfile.open(objfilename,objfile.out);
        inform("Output Obj");
        std::cerr<<objfilename<<std::endl;
        objfile<<"#obj "<<idx<<std::endl;
        cv::Mat layout = _BoundingBoxList[idx];
        for(c=0;c<layout.cols;++c)
        {
            objfile<<"v "<<layout.at<float>(0,c)
                       <<" "<<layout.at<float>(1,c)
                       <<" "<<layout.at<float>(2,c)
                       <<std::endl;
        }
        objfile<<"f "<<cnt+0<<" "<<cnt+1<<" "<<cnt+2<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+4<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+7<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+5<<" "<<cnt+1<<std::endl;
        objfile<<"f "<<cnt+1<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+2<<std::endl;
        objfile<<"f "<<cnt+2<<" "<<cnt+6<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        objfile.close();
    }
}

void PP_SceneGraph::outputObj(void)
{
    FullPointCloud::Ptr obj(new FullPointCloud);
    std::stringstream stream("");
    std::string objfilename;
    QString info;
    QList<quint32> objList;
    Pipe::loadData(objList,info,Pipe::_ObjListKey);
    obj->clear();
    Pipe::loadData(obj,info,_BGKey);

    stream.clear();
    stream<<_Yan_Meng_Path<<"obj"<<0;
    stream>>objfilename;

    pcl::Poisson<FullPoint> poisson;

    pcl::io::savePCDFileBinary(objfilename+".pcd",*obj);
    pcl::PolygonMesh mesh;
    poisson.setInputCloud(obj);
    poisson.performReconstruction(mesh);
    pcl::io::savePolygonFile(objfilename+".ply",mesh);

    unsigned int idx = 1;
    foreach(quint32 key,objList)
    {
        stream.clear();
        stream<<_Yan_Meng_Path<<"obj"<<idx;
        stream>>objfilename;
        obj->clear();
        Pipe::loadData(obj,info,key);
        pcl::io::savePCDFileBinary(objfilename+".pcd",*obj);
        pcl::PolygonMesh mesh;
        poisson.setInputCloud(obj);
        poisson.performReconstruction(mesh);
        pcl::io::savePolygonFile(objfilename+".ply",mesh);
        ++idx;
    }
}

