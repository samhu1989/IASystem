#ifndef PP_MULTIMERGEPLANE_H
#define PP_MULTIMERGEPLANE_H

#include "pp_multimergeplane_global.h"
#include "pipe.h"
class PP_MULTIMERGEPLANESHARED_EXPORT PP_MultiMergePlane:public Pipe
{
public:
    PP_MultiMergePlane();
    std::string name(){return "PP_MultiMergePlane";}
    __int32 version(){return 0;}

    int configure(Config&);
    int init(void);
    int work(void);
    int saveToData(void);

protected:

    void loadFrame(void);
    void loadCam(void);
    void loadCam(const std::string& path, Eigen::Matrix4f& _CamEx);
    void segPlane(FullPointCloud::Ptr&);
    void segWalls(FullPointCloud::Ptr&);
    void segFloor(FullPointCloud::Ptr& pcd);
    void segWall(FullPointCloud::Ptr& pcd);
    void loadFrame(unsigned int frameIdx);
    void segFrameRegionGrow(void);
    void addToDepthMap(const FullPointCloud::Ptr& pcd,unsigned int frameIdx);
    bool isWithinWall(FullPoint& p);

private:
    std::string _Seg_Image_Path;
    std::vector<FullPointCloud::Ptr> _FgList;
    FullPointCloud::Ptr _Bg;
    std::vector<std::vector<cv::Mat>> _DepthMapList;
    QVector<QVector<quint32>> _DepthMapKeyList;
    cv::Mat _CamIn;
    std::vector<cv::Mat> _CamExList;
    std::vector<cv::Mat> _IdMapList;

    cv::Mat _2DBB;
    float _Dist2Wall;
    float _PlaneError;

    int _Bg_Plane_Num;
    int _K_Neighbor;
    float _R_Neighbor;
    float _Angle_Threshold;
    float _Curve_Threshold;
    std::string _FramePath;
    std::string _CamPath;
    int _FrameNum;
    int _CamNum;
};
extern "C" PP_MULTIMERGEPLANESHARED_EXPORT Pipe* createPipe(void);
#endif // PP_MULTIMERGEPLANE_H
