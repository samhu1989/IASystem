#ifndef ICP_HPP
#define ICP_HPP
#include "icp.h"
#include <pcl/registration/ia_ransac.h>
template<typename Feature,typename FeatureExtractor>
void ICP::ICPInit<Feature,FeatureExtractor>::operator()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B)
{
    FeatureExtractor getFeatureA,getFeatureB;
    getFeatureA(A);
    getFeatureB(B);

    Pipe::record("ICP::ICPInit<Feature,FeatureExtractor>::operator()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B)");

    pcl::SampleConsensusInitialAlignment<FullPoint,FullPoint, pcl::PFHRGBSignature250> sac_ia;
    sac_ia.setMinSampleDistance (ICP::MIN_SAMPLE_DISTANCE);
    sac_ia.setNumberOfSamples(ICP::SAMPLE_NUM);
    sac_ia.setCorrespondenceRandomness(ICP::K_NEAREAST_RANDOM);
    sac_ia.setEuclideanFitnessEpsilon(ICP::FITNESS_EPS);
    sac_ia.setMaximumIterations(ICP::MAX_ITERATION_NUM_INIT);
    sac_ia.setTransformationEpsilon(ICP::TRANSFORM_EPS);
    sac_ia.setInputCloud(A);
    sac_ia.setSourceFeatures (getFeatureA.feature);
    sac_ia.setInputTarget(B);
    sac_ia.setTargetFeatures (getFeatureB.feature);
    sac_ia.setMaxCorrespondenceDistance(ICP::MAX_CORRESPONDENCE_DISTANCE_FOR_INIT);
    rCloud = FullPointCloud::Ptr(new FullPointCloud);
    sac_ia.align( *rCloud );
    if(sac_ia.hasConverged()){
        Pipe::record("Converged");
    }else{
        Pipe::record("Not Converged");
    }
    _Score = sac_ia.getFitnessScore(ICP::MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR);
    _T = sac_ia.getFinalTransformation();
}
#endif // ICP_HPP

