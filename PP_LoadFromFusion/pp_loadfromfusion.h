#ifndef PP_LOADFROMFUSION_H
#define PP_LOADFROMFUSION_H

#include "pp_loadfromfusion_global.h"
#include "pipe.h"
class PP_LOADFROMFUSIONSHARED_EXPORT PP_LoadFromFusion:public Pipe
{
public:
    PP_LoadFromFusion();
    ~PP_LoadFromFusion();
    std::string name(){return "PP_LoadFromFusion";}
    __int32 version(){return 0;}

    int configure(Config&);
    int init(void);
    int work(void);
    int saveToData(void);

protected:
    void loadFrame(void);
    void loadFrame(unsigned int frameIdx);
    void segFloor(FullPointCloud::Ptr& pcd);
    void segWall(FullPointCloud::Ptr& pcd);
    void segFrameRegionGrow(void);

private:
    std::string _FramePath;
    std::string _Suffix;
    int _FrameNum;

    FullPointCloud::Ptr _Bg;
    std::vector<FullPointCloud::Ptr> _FgList;
    std::vector<cv::Mat> _IdMapList;

    float _Dist2Wall;
    float _PlaneError;

    int _Bg_Plane_Num;
    int _K_Neighbor;
    float _R_Neighbor;
    float _Angle_Threshold;
    float _Curve_Threshold;
};
extern "C" PP_LOADFROMFUSIONSHARED_EXPORT Pipe* createPipe(void);
#endif // PP_LOADFROMFUSION_H
