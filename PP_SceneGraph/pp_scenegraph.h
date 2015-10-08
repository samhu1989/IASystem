#ifndef PP_SCENEGRAPH_H
#define PP_SCENEGRAPH_H

#include "pp_scenegraph_global.h"
#include "pipe.h"
#include <pcl/ModelCoefficients.h>
#include <QImage>
class PP_SCENEGRAPHSHARED_EXPORT PP_SceneGraph:public Pipe
{

public:
    PP_SceneGraph();
    std::string name(){return "PP_SceneGraph";}
    __int32 version(){return 1;}

    int configure(Config&);
    int init(void);
    int work(void);
    int saveToData(void);

protected:

    void getFloor(void);
    void getBoundingBox(void);
    void getBoundingBox(const FullPointCloud::Ptr&, cv::Mat& vertexes );
    void getProximate(void);
    void getProximate(std::vector<cv::Mat>&layouts, std::vector<unsigned int>&objIdx,QImage &debug);
    void getProximate(const Eigen::MatrixXf& centers,cv::Mat& relation);
    void getSceneLayout(unsigned int frameNum,std::vector<cv::Mat>&layouts,std::vector<unsigned int>&objIdx,QImage& debug = QImage());
    void getLayoutCenter(std::vector<cv::Mat>&layouts,Eigen::MatrixXf& centers);
    void getLayoutCenter(const cv::Mat&layout,Eigen::Vector3f&);
    void transformBoundingBox(const std::vector<std::vector<cv::Mat>>&transList,std::vector<cv::Mat>&layouts,std::vector<unsigned int>&obj_id,QImage& debug = QImage());
    void outputScene(unsigned int FrameIdx,const std::vector<cv::Mat>&layouts,const std::vector<unsigned int>&objIdx);
    void outputBox(void);
    void outputObj(void);

private:
    std::string _Dump_Source_Path;
    std::string _Yan_Meng_Path;
    int _FrameNum;
    pcl::ModelCoefficients _FloorCoeff;
    std::vector<cv::Mat> _BoundingBoxList;

};

extern "C" PP_SCENEGRAPHSHARED_EXPORT Pipe* createPipe(void);

#endif // PP_SCENEGRAPH_H
