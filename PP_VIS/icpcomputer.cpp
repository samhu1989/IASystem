#include "icpcomputer.h"
#include "icp.h"
ICPComputer::ICPComputer()
{

}

void ICPComputer::setCorr(std::vector<int>&pickeda,std::vector<int>&pickedb)
{
    _PickedA = pickeda;
    _PickedB = pickedb;
}

void ICPComputer::setA(FullPointCloud::Ptr a)
{
    _A = a;
}

void ICPComputer::setB(FullPointCloud::Ptr b)
{
    _B = b;
}

void ICPComputer::initCenter(void)
{
    ;
}

void ICPComputer::initManual(void)
{
    std::cerr<<"init manually"<<std::endl;
    Eigen::Map<Eigen::Matrix4f> T(_T);
    pcl::Correspondences corr;
    pcl::registration::TransformationEstimationSVD<FullPoint,FullPoint> est;

    unsigned int idx;
    for(idx=0;idx<_PickedA.size();++idx)
    {
        pcl::Correspondence c;
        c.index_query = _PickedA[idx];
        c.index_match = _PickedB[idx];
        FullPoint &ps = _A->at(_PickedA[idx]);
        FullPoint &pt = _B->at(_PickedB[idx]);
        c.distance = std::sqrtf(
                    (ps.x-pt.x)*(ps.x-pt.x)
                    +(ps.y-pt.y)*(ps.y-pt.y)
                    +(ps.z-pt.z)*(ps.z-pt.z)
                    );
        c.weight = 1.0/float(_PickedA.size());
        corr.push_back(c);
    }

    Eigen::Matrix4f t;
    est.estimateRigidTransformation(*_A,*_B,corr,t);
    T = t;
}

void ICPComputer::run(void)
{
    if(_PickedA.size()==_PickedB.size()&&_PickedA.size()>=3)
    {
        initManual();
    }else{
        initCenter();
    }
    Eigen::Map<Eigen::Matrix4f> T(_T);
    std::cerr<<"icp"<<std::endl;
    ICP::ICPGeneral icp;
    icp._T = T;
    icp(_A,_B);
    T = icp._T;
}

void ICPComputer::getT(Eigen::Matrix4f& T)
{
    Eigen::Map<Eigen::Matrix4f> t(_T);
    T = t;
}

ICPComputer::~ICPComputer()
{

}

