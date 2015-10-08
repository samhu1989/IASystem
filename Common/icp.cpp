#include "icp.h"
#include <pcl/features/pfhrgb.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/fpfh_omp.h>
#include "pipe.h"
#include "gicp6d.h"
#include "histogram.h"
#include <map>
#include <vector>
#include <Eigen/SVD>
#include "SparseICP.h"
#include <QTime>
double ICP::DOWN_SAMPLE_GRID_SIZE = 0.05;
int ICP::MIN_SAMPLE_DISTANCE = 0;
int ICP::SAMPLE_NUM = 0 ;
int ICP::K_NEAREAST_RANDOM = 0;
int ICP::MAX_ITERATION_NUM_INIT = 500;
double ICP::TRANSFORM_EPS =1e-15;
double ICP::FITNESS_EPS =1e-15 ;
double ICP::MAX_CORRESPONDENCE_DISTANCE_FOR_INIT = 0.1;
double ICP::MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR = 0.05;
int ICP::FEATURE_SEARCH_K = 10;
double ICP::MAX_CORRESPONDENCE_DISTANCE_FOR_LOOP = 1.0;
int ICP::MAX_ITERATION_NUM_LOOP = 8000;
double ICP::MAX_NON_OCCLUSION = 0.08;

void ICP::DownSampler::operator()(const FullPointCloud::Ptr& in)
{
    if(in->empty())
    {
        Pipe::error("Down Sampling Zero Sized Point Cloud");
        return;
    }
    if(downSampled->empty())
    {
        pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
        grid.setLeafSize(ICP::DOWN_SAMPLE_GRID_SIZE, ICP::DOWN_SAMPLE_GRID_SIZE, ICP::DOWN_SAMPLE_GRID_SIZE);
        grid.setInputCloud (in);
        Pipe::inform("DownSampling:");
        grid.filter (*downSampled);
        std::cerr<<"From "<<in->size()<<" To "<<downSampled->size()<<std::endl;
    }
}

void ICP::FeatureExtractorPFHRGB::operator()(const FullPointCloud::Ptr& in)
{
    if(feature->empty())
    {
        pcl::PFHRGBEstimation<FullPoint,FullPoint,pcl::PFHRGBSignature250> fest;
        fest.setKSearch(ICP::FEATURE_SEARCH_K);
        fest.setInputCloud(in);
        fest.setInputNormals(in);
        fest.compute(*feature);
    }
}

void ICP::ICPFiner::operator()(const FullPointCloud::Ptr&A,const FullPointCloud::Ptr&B)
{
    Pipe::inform("ICP");
    pcl::IterativeClosestPoint<FullPoint,FullPoint> reg;
    pcl::registration::TransformationEstimationPointToPlaneLLS<FullPoint,FullPoint>::Ptr est(new pcl::registration::TransformationEstimationPointToPlaneLLS<FullPoint,FullPoint>);
    rCloud = FullPointCloud::Ptr(new FullPointCloud);
    PointCurve<FullPoint> point_representation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);
    pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,_T);
    reg.setPointRepresentation(boost::make_shared<const PointCurve<FullPoint>> (point_representation));
    reg.setRANSACOutlierRejectionThreshold(std::sqrt(std::numeric_limits<double>::max()));
    reg.setMaxCorrespondenceDistance(std::sqrt(std::numeric_limits<double>::max()));
    reg.setRANSACIterations(5);
    reg.setTransformationEpsilon(TRANSFORM_EPS);
    reg.setEuclideanFitnessEpsilon(FITNESS_EPS);
//    reg.setTransformationEstimation(est);
    reg.setInputCloud( rCloud );
    reg.setInputTarget( B );
    reg.setMaximumIterations(100);
    reg.align(*rCloud);
    _T = reg.getFinalTransformation()*_T;
    _Score = reg.getFitnessScore();

    double threshold = MAX_CORRESPONDENCE_DISTANCE_FOR_INIT;
    reg.setRANSACOutlierRejectionThreshold(threshold);
    reg.setMaxCorrespondenceDistance(threshold);

    unsigned int iter;
    double lasterror=std::numeric_limits<double>::max();
    double error = reg.getFitnessScore(MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR);
    for(iter=0;iter<MAX_ITERATION_NUM_LOOP;iter+=1)
    {
        if(error>=lasterror)break;
        if(threshold<MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR)break;
        pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,_T);
        reg.setInputCloud(rCloud);
        reg.setInputTarget(B);
        reg.align(*rCloud);
        _T = reg.getFinalTransformation()*_T;
        if(reg.hasConverged())
        {
            threshold *= 0.75;
            reg.setRANSACOutlierRejectionThreshold(threshold);
            reg.setMaxCorrespondenceDistance(threshold);
        }
        lasterror = error;
        error = reg.getFitnessScore(MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR);
    }
    _Score = reg.getFitnessScore(MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR);
}

void ICP::ICPGeneral::operator()(const FullPointCloud::Ptr&A,const FullPointCloud::Ptr&B)
{
    Pipe::inform("ICP");
    pcl::GeneralizedIterativeClosestPoint6D reg;
    rCloud = FullPointCloud::Ptr(new FullPointCloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _A(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _rCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::copyPointCloud(*A,*_A);
    pcl::copyPointCloud(*B,*tCloud);

    pcl::transformPointCloud<pcl::PointXYZRGBA>(*_A,*_rCloud,_T);

    reg.setRANSACOutlierRejectionThreshold(0.08);
    reg.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE_FOR_INIT);
    reg.setRANSACIterations(5);
    reg.setTransformationEpsilon(TRANSFORM_EPS);
    reg.setEuclideanFitnessEpsilon(FITNESS_EPS);

    reg.setInputSource( _rCloud );
    reg.setInputTarget( tCloud );
    reg.setMaximumIterations(MAX_ITERATION_NUM_LOOP);

    reg.align(*_rCloud);

    _T = reg.getFinalTransformation()*_T;
    _Score = reg.getFitnessScore(MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR);
    pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,_T);
    Pipe::inform("End-ICP");
}

void ICP::ICPPoint2Plane::operator()(const FullPointCloud::Ptr&A,const FullPointCloud::Ptr&B)
{
    pcl::IterativeClosestPoint<FullPoint,FullPoint> reg;
    pcl::registration::TransformationEstimationPointToPlane<FullPoint,FullPoint>::Ptr est(new pcl::registration::TransformationEstimationPointToPlane<FullPoint,FullPoint>);
    rCloud = FullPointCloud::Ptr(new FullPointCloud);
    PointCurve<FullPoint> point_representation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);
    pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,_T);
    reg.setPointRepresentation(boost::make_shared<const PointCurve<FullPoint>> (point_representation));
    reg.setRANSACOutlierRejectionThreshold(std::sqrt(std::numeric_limits<double>::max()));
    reg.setMaxCorrespondenceDistance(std::sqrt(std::numeric_limits<double>::max()));
    reg.setRANSACIterations(5);
    reg.setTransformationEpsilon(TRANSFORM_EPS);
    reg.setEuclideanFitnessEpsilon(FITNESS_EPS);
    reg.setTransformationEstimation(est);
    reg.setInputCloud( rCloud );
    reg.setInputTarget( B );
    reg.setMaximumIterations(100);
    reg.align(*rCloud);
    _T = reg.getFinalTransformation()*_T;
    _Score = reg.getFitnessScore();

    double threshold = 1.0;
    reg.setRANSACOutlierRejectionThreshold(threshold);
    reg.setMaxCorrespondenceDistance(threshold);

    unsigned int iter;
    for(iter=0;iter<MAX_ITERATION_NUM_LOOP;iter+=1)
    {
        pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,_T);
        reg.setInputCloud(rCloud);
        reg.setInputTarget(B);
        reg.align(*rCloud);
        _T = reg.getFinalTransformation()*_T;
        if(reg.hasConverged())
        {
            if(threshold<MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR)break;
            threshold *= 0.75;
            reg.setRANSACOutlierRejectionThreshold(threshold);
            reg.setMaxCorrespondenceDistance(threshold);
        }
    }
    _Score = reg.getFitnessScore(MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR);
}

double ICP::getFitnessError(const FullPointCloud::Ptr&rCloud,const FullPointCloud::Ptr&tCloud,const std::string& info)
{
    Pipe::record(info);
    double overlap = 0.0;
    double curvesim = 0.0;
    double distance = 0.0;
    double rgb_dist = 0.0;

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> tree;
    tree.setInputCloud( tCloud);
    double nx1,ny1,nz1, nx2,ny2,nz2;

    // For each point in the transformed source cloud
    int nr = 0;

    for (size_t i = 0; i < rCloud->size(); ++i)
    {
        Eigen::Vector4f p1 = Eigen::Vector4f (rCloud->at(i).x, rCloud->at(i).y, rCloud->at(i).z, 0);

        FullPoint &r = rCloud->at(i);

        // Find its nearest neighbor in the target
        tree.nearestKSearch( rCloud->at(i), 1, nn_indices, nn_dists);
        if( nn_dists[0] > ICP::MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR )
            continue;
        //if( tree.radiusSearch( rCloud->at(i),max_range, nn_indices, nn_dists,1) < 1 )
        //	continue;

        Eigen::Vector4f p2 = Eigen::Vector4f (tCloud->points[nn_indices[0]].x,
                                              tCloud->points[nn_indices[0]].y,
                                              tCloud->points[nn_indices[0]].z, 0);
        FullPoint &t = tCloud->points[nn_indices[0]];

        if( _isnan(nx1)|| _isnan(nx2) ||_isnan(ny1) ||_isnan(ny2) || _isnan(nz1) || _isnan(nz2) )
        {
            continue;
        }
        // Calculate the fitness score
        double dis = fabs ((p1-p2).squaredNorm ());
        overlap ++;
        curvesim += ( r.curvature - t.curvature )*( r.curvature - t.curvature );
        distance += dis;
        rgb_dist += Pipe::RGB_Dist(r.r,r.g,r.b,t.r,t.g,t.b);

    }
//    normalsim/=overlap;
    distance/=overlap;
    rgb_dist/=overlap;
    curvesim/=overlap;
    overlap = 2*overlap / ( rCloud->size() + tCloud->size() );
    std::cout<<"norm"<<curvesim<<std::endl;
    std::cout<<"distance"<<distance<<std::endl;
    std::cout<<"rgb_dist"<<rgb_dist<<std::endl;
    std::cout<<"overlap"<<overlap<<std::endl;
    double fitness_score = ( 0.33* distance + 0.33*rgb_dist + 0.33*curvesim ) * ( 2 - overlap );
    return fitness_score;
}

void ICP::ICPMixedInit::operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B)
{
    _A = A;
    _B = B;
    rCloud = FullPointCloud::Ptr(new FullPointCloud);
    detectPlanes(_A,_PlanesLabelA,_PlanesA,_PlaneCenterA,_PlaneHistsA,_NonPlaneIndexA);
    detectPlanes(_B,_PlanesLabelB,_PlanesB,_PlaneCenterB,_PlaneHistsB,_NonPlaneIndexB);
    assignPlanes();
    unsigned int planepairnum = _ZPlanesMatch.size()+_NPlanesMatch.size();
    if( planepairnum <= 1)
    {
        estimateTransformationCenter();
    }else{
        derivePoints();
        assignPoints();
        estimateTransformationMixed();
    }
    Pipe::inform("Done Compute");
}

void ICP::ICPMixedInit::getPlanesA( cv::Mat& labels)
{
    _PlanesLabelA.copyTo(labels);
}

void ICP::ICPMixedInit::getPlanesB( cv::Mat& labels)
{
    _PlanesLabelB.copyTo(labels);
}

void ICP::ICPMixedInit::getMatchedPlanesA( cv::Mat& labels)
{
    _PlanesLabelA.copyTo(labels);
    std::vector<std::pair<int,int>>::iterator iter;
    std::map<int,int> label_map;
    unsigned int idx = 0;
    for( iter=_ZPlanesMatch.begin() ; iter!=_ZPlanesMatch.end() ; ++iter )
    {
        label_map[iter->first] = idx+1;
        ++idx;
    }
    for( iter=_NPlanesMatch.begin() ; iter!=_NPlanesMatch.end() ; ++iter )
    {
        label_map[iter->first] = idx+1;
        ++idx;
    }
    for(idx=0;idx<labels.cols;++idx)
    {
        int lbl = labels.at<int>(0,idx);
        if(label_map.find(lbl)!=label_map.end())
        {
            labels.at<int>(0,idx) = label_map[lbl];
        }else{
            labels.at<int>(0,idx) = -1;
        }
    }
}

void ICP::ICPMixedInit::getMatchedPlanesB( cv::Mat& labels )
{
    _PlanesLabelB.copyTo(labels);
    std::vector<std::pair<int,int>>::iterator iter;
    std::map<int,int> label_map;
    unsigned int idx = 0;
    for( iter=_ZPlanesMatch.begin() ; iter!=_ZPlanesMatch.end() ; ++iter )
    {
        std::cerr<<iter->first<<","<<iter->second<<std::endl;
        label_map[iter->second] = idx+1;
        ++idx;
    }
    for( iter=_NPlanesMatch.begin() ; iter!=_NPlanesMatch.end() ; ++iter )
    {
        std::cerr<<iter->first<<","<<iter->second<<std::endl;
        label_map[iter->second] = idx+1;
        ++idx;
    }
    for(idx=0;idx<labels.cols;++idx)
    {
        int lbl = labels.at<int>(0,idx);
        if(label_map.find(lbl)!=label_map.end())
        {
            labels.at<int>(0,idx) = label_map[lbl];
        }else{
            labels.at<int>(0,idx) = -1;
        }
    }
}

void ICP::ICPMixedInit::getPlaneCorr(std::vector<std::pair<int,int>>&corres)
{
    std::vector<std::pair<int,int>>::iterator iter;
    for( iter=_ZPlanesMatch.begin() ; iter!=_ZPlanesMatch.end() ; ++iter )
    {
        corres.push_back(*iter);
    }
    for( iter=_NPlanesMatch.begin() ; iter!=_NPlanesMatch.end() ; ++iter )
    {
        corres.push_back(*iter);
    }
}

void ICP::ICPMixedInit::getPointsCorr(
        FullPointCloud::Ptr& pA,
        FullPointCloud::Ptr& pB
        )
{
    pA = FullPointCloud::Ptr(new FullPointCloud);
    pB = FullPointCloud::Ptr(new FullPointCloud);
    std::vector<std::pair<int,int>>::iterator iter;
    for(iter=_PointsMatch.begin();iter!=_PointsMatch.end();++iter)
    {
        pA->push_back(_A->at(iter->first));
        pB->push_back(_B->at(iter->second));
    }
    pA->width = pA->size();
    pA->height = 1;
    pB->width = pB->size();
    pB->height = 1;
}

void ICP::ICPMixedInit::getDerivedPointsCorr(
        FullPointCloud::Ptr& pA,
        FullPointCloud::Ptr& pB
        )
{
    pA = FullPointCloud::Ptr(new FullPointCloud);
    pB = FullPointCloud::Ptr(new FullPointCloud);
    std::vector<std::pair<int,int>>::iterator iter;
    for(iter=_ZPlanesMatch.begin();iter!=_ZPlanesMatch.end();++iter)
    {
        pA->push_back(_PlaneCenterA->at(iter->first-1));
        pB->push_back(_PlaneCenterB->at(iter->second-1));
    }
    for(iter=_NPlanesMatch.begin();iter!=_NPlanesMatch.end();++iter)
    {
        pA->push_back(_PlaneCenterA->at(iter->first-1));
        pB->push_back(_PlaneCenterB->at(iter->second-1));
    }
    pA->width = pA->size();
    pA->height = 1;
    pB->width = pB->size();
    pB->height = 1;
}

void ICP::ICPMixedInit::detectPlanes(
        const FullPointCloud::Ptr& cloud,
        cv::Mat&labels,
        std::vector<pcl::ModelCoefficients::Ptr>&models,
        FullPointCloud::Ptr& centers,
        std::vector<cv::Mat>&hists,
        std::vector<int>&remaining
        )
{
    labels.create(1,cloud->size(),CV_32SC1);
    labels.setTo(-1);
    _MinPlanePointNum = 200;
    detectPlanes(cloud,labels,models,centers,hists,remaining,pcl::SACMODEL_NORMAL_PLANE);
}


void ICP::ICPMixedInit::detectPlanes(const FullPointCloud::Ptr& cloud,
        cv::Mat&labels,
        std::vector<pcl::ModelCoefficients::Ptr>&models,
        FullPointCloud::Ptr& centers,
        std::vector<cv::Mat>&hists,
        std::vector<int>&remaining,
        int plane_type
        )
{
    Pipe::inform("detecting planes");
    unsigned int idx;
    pcl::SACSegmentationFromNormals<FullPoint,FullPoint> seg;
    pcl::ExtractIndices<FullPoint>extract;

    extract.setNegative (false);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (plane_type);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/36);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setNormalDistanceWeight(0.15);
    seg.setRadiusLimits(0.06,0.12);
    seg.setDistanceThreshold (ICP::DOWN_SAMPLE_GRID_SIZE*1.50);
    unsigned int count = 0;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
    FullPointCloud::Ptr cloud_p(new FullPointCloud);
    centers = FullPointCloud::Ptr(new FullPointCloud);

    outliers->indices.clear();
    for(idx=0;idx<labels.cols;++idx)
    {
        if(-1==labels.at<int>(0,idx))
        outliers->indices.push_back(idx);
    }
    Pipe::inform("starting plane iter");
    while(count<_MaxPlaneNum)
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud);
        seg.setIndices(outliers);
        seg.segment(*inliers,*coefficients);
        if( inliers->indices.size() < _MinPlanePointNum ){
            break;
        }

        //add new plane
        models.push_back(coefficients);
        for(idx=0;idx<inliers->indices.size();++idx)
        {
            labels.at<int>(0,inliers->indices[idx]) = models.size();
        }

        //extract plane and calculate features
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.filter (*cloud_p);

        //plane features
        cv::Mat colorhist;
        cv::Mat sizehist;

        colorhist = Histogram::getColorHist(cloud_p);
        FullPoint planecenter;
        sizehist = Histogram::getDifSize(cloud_p,planecenter);
        centers->push_back(planecenter);

        hists.push_back(cv::Mat());
        colorhist.copyTo(hists.back());
        hists.push_back(cv::Mat());
        sizehist.copyTo(hists.back());

        //label remaining
        outliers->indices.clear();
        for(idx=0;idx<labels.cols;++idx)
        {
            if(-1==labels.at<int>(0,idx))
            outliers->indices.push_back(idx);
        }
        ++count;
    }
    remaining = outliers->indices;
    centers->height = 1;
    centers->width = centers->size();
    std::cerr<<"plane num:"<<count<<std::endl;
    Pipe::inform("done plane iter");
}

typedef struct{
    int a;
    int b;
    float d;
    char type;
}DistPair;

bool uniqueByB(DistPair& p0,DistPair& p1)
{
    return p0.b == p1.b;
}

bool lessByBD(DistPair& p0,DistPair& p1)
{
    if(p0.b == p1.b)
    {
        return p0.d < p1.d;
    }else return p0.b < p1.b;
}

bool lessByD(DistPair& p0,DistPair& p1)
{
    return p0.d < p1.d;
}

bool lessByI(DistPair& p0,DistPair& p1)
{
    return p0.a<p1.a;
}

void ICP::ICPMixedInit::assignPlanes(void)
{
    unsigned int iA,iB;
    std::vector<DistPair> pairs;
    Eigen::MatrixXf distMatrix(_PlanesA.size(),_PlanesB.size());
    for( iA = 0 ; iA < _PlanesA.size(); ++iA )
    {
        for( iB = 0 ; iB < _PlanesB.size() ; ++iB)
        {
            float dist = 0;
            dist += 0.2*Histogram::getEuclidean(_PlaneHistsA[2*iA],_PlaneHistsB[2*iB]);
            dist += 0.8*Histogram::getEuclidean(_PlaneHistsA[2*iA+1],_PlaneHistsB[2*iB+1]);
            distMatrix(iA,iB) = dist;
        }
    }
    for( iA = 0 ; iA < _PlanesA.size(); ++iA )
    {
        float _1stmin = std::numeric_limits<float>::max()-1;
        float _2ndmin = std::numeric_limits<float>::max();
        int best_ib = -1;
        for( iB = 0 ; iB < _PlanesB.size() ; ++iB)
        {
            float d = distMatrix(iA,iB);
            if( d <= _1stmin)
            {
                _2ndmin = _1stmin;
                _1stmin = d;
                best_ib = iB;
            }else if( d < _2ndmin )
            {
                _2ndmin = d;
            }
        }
        if( _1stmin < 0.8*_2ndmin && _2ndmin < std::numeric_limits<float>::max()-1 )
        {
            int minRow;
            distMatrix.col(best_ib).minCoeff(&minRow);
            if(iA == minRow)
            {
                DistPair p;
                p.a = iA+1;
                p.b = best_ib+1;
                p.d = distMatrix(iA,best_ib);
                if( 0 > ( std::abs(_PlanesB[best_ib]->values[2]) - 0.707 )*( std::abs(_PlanesA[iA]->values[2]) - 0.707 )   )
                {
                    p.type = 'x';
                }else if( std::abs(_PlanesA[iA]->values[2]) - 0.707 < 0 )
                {
                    p.type = 'n';
                }else{
                    p.type = 'z';
                }
                pairs.push_back(p);
            }
        }
    }
    //check if it is best for B
    std::vector<DistPair>::iterator iter;
    std::sort(pairs.begin(),pairs.end(),lessByD);
    unsigned int cntZ = 0;
    unsigned int cntN = 0;
    for(iter=pairs.begin();iter!=pairs.end();++iter)
    {
        if(iter->type=='z')
        {
            if(cntZ>1)continue;
            else{
                _ZPlanesMatch.push_back(std::pair<int,int>());
                _ZPlanesMatch.back().first = iter->a;
                _ZPlanesMatch.back().second = iter->b;
                ++cntZ;
            }
        }
        if(iter->type=='n')
        {
            if(cntN>1)continue;
            else{
                _NPlanesMatch.push_back(std::pair<int,int>());
                _NPlanesMatch.back().first = iter->a;
                _NPlanesMatch.back().second = iter->b;
                ++cntN;
            }
        }
    }
    std::cerr<<"Matched Z Pairs"<<_ZPlanesMatch.size()<<std::endl;
    std::cerr<<"Matched N Pairs"<<_NPlanesMatch.size()<<std::endl;
}

void ICP::ICPMixedInit::assignPoints(void)
{
    FPFHCloud::Ptr featureA(new FPFHCloud);
    FPFHCloud::Ptr featureB(new FPFHCloud);

    pcl::FPFHEstimation<FullPoint,FullPoint> fest;
    fest.setRadiusSearch(0.3);

    fest.setInputCloud(_A);
    fest.setInputNormals(_A);
    fest.compute(*featureA);

    fest.setInputCloud(_B);
    fest.setInputNormals(_B);
    fest.compute(*featureB);

    pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr flannA(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
    pcl::IndicesPtr indiceA(new std::vector<int>());
    *indiceA = _NonPlaneIndexA;
    flannA->setInputCloud(featureA,indiceA);

    pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr flannB(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
    pcl::IndicesPtr indiceB(new std::vector<int>());
    *indiceB = _NonPlaneIndexB;
    flannB->setInputCloud(featureB,indiceB);

    //mutual closest feature
    std::vector<int>::iterator iter;
    std::vector<int> idxA;
    std::vector<int> idxB;
    std::vector<float> a2b;
    std::vector<float> b2a;
    for(iter=_NonPlaneIndexA.begin();iter!=_NonPlaneIndexA.end();++iter)
    {
        flannB->nearestKSearchT(featureA->at(*iter),2,idxB,a2b);
        flannA->nearestKSearchT(featureB->at(idxB.front()),2,idxA,b2a);
        if( *iter == idxA.front() && ( a2b.front() < 0.25*a2b.back() ) )
        {
            _PointsMatch.push_back(std::pair<int,int>());
            _PointsMatch.back().first = *iter;
            _PointsMatch.back().second = idxB.front();
        }
    }
    std::cerr<<"Matched Points Number:"<<_PointsMatch.size()<<std::endl;
}
//derive the plane center as point pairs
void ICP::ICPMixedInit::derivePoints(void)
{
    _DerivedPointsA = FullPointCloud::Ptr(new FullPointCloud);
    _DerivedPointsB = FullPointCloud::Ptr(new FullPointCloud);
    std::vector<std::pair<int,int>>::iterator iter;
    for(iter=_ZPlanesMatch.begin();iter!=_ZPlanesMatch.end();++iter)
    {
        _DerivedPointsA->push_back(_PlaneCenterA->at(iter->first-1));
        _DerivedPointsB->push_back(_PlaneCenterB->at(iter->second-1));
    }
    for(iter=_NPlanesMatch.begin();iter!=_NPlanesMatch.end();++iter)
    {
        _DerivedPointsA->push_back(_PlaneCenterA->at(iter->first-1));
        _DerivedPointsB->push_back(_PlaneCenterB->at(iter->second-1));
    }
}

double ICP::getFitnessError(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B)
{
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> tree;
    tree.setInputCloud(B);
    // For each point in the transformed source cloud
    int nr = 0;
    double sum = 0.0;
    for (size_t i = 0; i < A->size(); ++i)
    {
        FullPoint &r = A->at(i);
        tree.nearestKSearch( r, 1, nn_indices, nn_dists);
        sum += nn_dists[0];
    }
    return sum/A->size();
}

double ICP::getOverlapNum(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B)
{
    std::vector<int> indicesa (1);
    std::vector<float> distsa (1);

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> tree;
    tree.setInputCloud(B);
    // For each point in the transformed source cloud
    int N = 0;
    for (size_t i = 0; i < A->size(); ++i)
    {
        FullPoint &s = A->at(i);
        tree.nearestKSearch( s, 1, indicesa, distsa);
        if( distsa[0] < ICP::DOWN_SAMPLE_GRID_SIZE )N++;
    }
    return double(N);
}

double ICP::getOverlapNorm(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B)
{
    std::vector<int> indicesa (1);
    std::vector<float> distsa (1);
    std::vector<int> indicesb (1);
    std::vector<float> distsb (1);

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> treeB;
    treeB.setInputCloud(B);
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> treeA;
    treeA.setInputCloud(A);
    // For each point in the transformed source cloud
    double N = 0;
    for (size_t i = 0; i < A->size(); ++i)
    {
        FullPoint &s = A->at(i);
        treeB.nearestKSearch( s, 1, indicesa, distsa);
        FullPoint &t = B->at(indicesa[0]);
        treeA.nearestKSearch( t, 1, indicesb, distsb);
        if( i == indicesb[0] )
        {
            N += std::abs(
                        t.normal_x*s.normal_x
                        + t.normal_y*s.normal_y
                        + t.normal_z*s.normal_z
                        ) / (1+distsa[0]);
        }
    }
    return double(N);
}

void ICP::ICPMixedInit::estimateTransformationCenter(void)
{
    FullPoint centerA;
    FullPoint centerB;
    if( _ZPlanesMatch.size() > 0  )
    {
        centerA = _PlaneCenterA->at(_ZPlanesMatch[0].first-1);
        centerB = _PlaneCenterB->at(_ZPlanesMatch[0].second-1);
    }else if( _NPlanesMatch.size() > 0 ){
        centerA = _PlaneCenterA->at(_NPlanesMatch[0].first-1);
        centerB = _PlaneCenterB->at(_NPlanesMatch[0].second-1);
    }else{
        Pipe::getPCDBoxCenter(_A,centerA);
        Pipe::getPCDBoxCenter(_B,centerB);
    }
    ICP::RotateAloneZ rotate;
    rotate(_A,centerA,_B,centerB);
    _T = rotate._T;
}

void ICP::ICPMixedInit::estimateTransformationMixed(void)
{
    ICP::MixedEstimator slam;
    std::vector<pcl::ModelCoefficients::Ptr> _cA;
    FullPointCloud::Ptr _pA(new FullPointCloud);
    std::vector<pcl::ModelCoefficients::Ptr> _cB;
    FullPointCloud::Ptr _pB(new FullPointCloud);
    _Score = std::numeric_limits<float>::max();

    std::vector<std::pair<int,int>>::iterator iter;
    for(iter=_PointsMatch.begin();iter!=_PointsMatch.end();++iter)
    {
        _pA->push_back(_A->at(iter->first));
        _pB->push_back(_B->at(iter->second));
    }
    unsigned int ip;
    for(ip=0;ip<_DerivedPointsA->size();++ip)
    {
        _pA->push_back(_DerivedPointsA->at(ip));
        _pB->push_back(_DerivedPointsB->at(ip));
    }
    _pA->width = _pA->size();
    _pA->height = 1;
    _pB->width = _pB->size();
    _pB->height = 1;
    /*
     * flip the z plane normal up
     */
    for(iter=_ZPlanesMatch.begin();iter!=_ZPlanesMatch.end();++iter)
    {
        pcl::ModelCoefficients::Ptr pA = _PlanesA[(iter->first-1)];
        pcl::ModelCoefficients::Ptr pB = _PlanesB[(iter->second-1)];
        if(pA->values[2]<0)
        {
            pA->values[0] = -(pA->values[0]);
            pA->values[1] = -(pA->values[1]);
            pA->values[2] = -(pA->values[2]);
            pA->values[3] = -(pA->values[3]);
        }
        if(pB->values[2]<0)
        {
            pB->values[0] = -(pB->values[0]);
            pB->values[1] = -(pB->values[1]);
            pB->values[2] = -(pB->values[2]);
            pB->values[3] = -(pB->values[3]);
        }
        _cA.push_back(pA);
        _cB.push_back(pB);
    }
    //input none-z plane
    for(iter=_NPlanesMatch.begin();iter!=_NPlanesMatch.end();++iter)
    {
        _cA.push_back(_PlanesA[(iter->first-1)]);
        _cB.push_back(_PlanesB[(iter->second-1)]);
    }
    /*
     * flip none for none-z planes
     */
    std::cerr<<"Start None Flip"<<std::endl;
    slam(_cA,_pA,_cB,_pB);
    std::cerr<<"Done None Flip"<<std::endl;
    FullPointCloud::Ptr transA(new FullPointCloud);
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    double currentScore = std::numeric_limits<double>::max();
    if(!_isnan(slam._T(0,0))&&!_isnan(slam._T(1,1)))currentScore = ICP::getFitnessError(transA,_B);
    else currentScore = std::numeric_limits<double>::max();
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
    std::cerr<<"T0:"<<std::endl;
    std::cerr<<slam._T<<std::endl;
    std::cerr<<currentScore<<std::endl;
    if(_NPlanesMatch.size()<=0)return;
    /*
     *flip first for none-z planes
     */
    iter=_NPlanesMatch.begin();

    pcl::ModelCoefficients::Ptr pA = _PlanesA[(iter->first-1)];
    pA->values[0] = -(pA->values[0]);
    pA->values[1] = -(pA->values[1]);
    pA->values[2] = -(pA->values[2]);
    pA->values[3] = -(pA->values[3]);

    slam(_cA,_pA,_cB,_pB);
    std::cerr<<"Done Flip First"<<std::endl;

    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    if(!_isnan(slam._T(0,0))&&!_isnan(slam._T(1,1)))currentScore = ICP::getFitnessError(transA,_B);
    else currentScore = std::numeric_limits<double>::max();
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
    std::cerr<<"T1:"<<std::endl;
    std::cerr<<slam._T<<std::endl;
    std::cerr<<currentScore<<std::endl;
    /*
     *flip both for none-z planes
     */
    ++iter;
    //terminate if only one none-z plane
    if(iter==_NPlanesMatch.end())return;
    pA = _PlanesA[(iter->first-1)];
    pA->values[0] = -(pA->values[0]);
    pA->values[1] = -(pA->values[1]);
    pA->values[2] = -(pA->values[2]);
    pA->values[3] = -(pA->values[3]);

    slam(_cA,_pA,_cB,_pB);
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    if(!_isnan(slam._T(0,0))&&!_isnan(slam._T(1,1)))currentScore = ICP::getFitnessError(transA,_B);
    else currentScore = std::numeric_limits<double>::max();
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
    std::cerr<<"T2:"<<std::endl;
    std::cerr<<slam._T<<std::endl;
    std::cerr<<currentScore<<std::endl;
    //flip only the second(flip back the first)
    iter = _NPlanesMatch.begin();
    pA = _PlanesA[(iter->first-1)];
    pA->values[0] = -(pA->values[0]);
    pA->values[1] = -(pA->values[1]);
    pA->values[2] = -(pA->values[2]);
    pA->values[3] = -(pA->values[3]);

    slam(_cA,_pA,_cB,_pB);
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    if(!_isnan(slam._T(0,0))&&!_isnan(slam._T(1,1)))currentScore = ICP::getFitnessError(transA,_B);
    else currentScore = std::numeric_limits<double>::max();
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
    std::cerr<<"T3:"<<std::endl;
    std::cerr<<slam._T<<std::endl;
    std::cerr<<currentScore<<std::endl;
}

void ICP::ICPMixedInit::estimateTransformationPlane(void)
{
    ICP::PlanarSLAM slam;
    std::vector<pcl::ModelCoefficients::Ptr> _cA;
    std::vector<pcl::ModelCoefficients::Ptr> _cB;
    _Score = std::numeric_limits<float>::max();

    std::vector<std::pair<int,int>>::iterator iter;
    //flip the z plane normal up
    for(iter=_ZPlanesMatch.begin();iter!=_ZPlanesMatch.end();++iter)
    {
        pcl::ModelCoefficients::Ptr pA = _PlanesA[(iter->first-1)];
        pcl::ModelCoefficients::Ptr pB = _PlanesB[(iter->second-1)];
        if(pA->values[2]<0)
        {
            pA->values[0] = -(pA->values[0]);
            pA->values[1] = -(pA->values[1]);
            pA->values[2] = -(pA->values[2]);
            pA->values[3] = -(pA->values[3]);
        }
        if(pB->values[2]<0)
        {
            pB->values[0] = -(pB->values[0]);
            pB->values[1] = -(pB->values[1]);
            pB->values[2] = -(pB->values[2]);
            pB->values[3] = -(pB->values[3]);
        }
        _cA.push_back(pA);
        _cB.push_back(pB);
    }

    for(iter=_NPlanesMatch.begin();iter!=_NPlanesMatch.end();++iter)
    {
        _cA.push_back(_PlanesA[(iter->first-1)]);
        _cB.push_back(_PlanesB[(iter->second-1)]);
    }
    //flip none for none-z planes
    slam(_cA,_cB);
    FullPointCloud::Ptr transA(new FullPointCloud);
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    double currentScore = ICP::getFitnessError(transA,_B);
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
    //flip first for none-z planes
    iter=_NPlanesMatch.begin();

    pcl::ModelCoefficients::Ptr pA = _PlanesA[(iter->first-1)];
    pA->values[0] = -(pA->values[0]);
    pA->values[1] = -(pA->values[1]);
    pA->values[2] = -(pA->values[2]);
    pA->values[3] = -(pA->values[3]);

    slam(_cA,_cB);
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    currentScore = ICP::getFitnessError(transA,_B);
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
    //flip both for none-z planes
    ++iter;
    //terminate if only one none-z plane
    if(iter==_NPlanesMatch.end())return;
    pA = _PlanesA[(iter->first-1)];
    pA->values[0] = -(pA->values[0]);
    pA->values[1] = -(pA->values[1]);
    pA->values[2] = -(pA->values[2]);
    pA->values[3] = -(pA->values[3]);

    slam(_cA,_cB);
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    currentScore = ICP::getFitnessError(transA,_B);
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
    //flip only the second(flip back the first)
    iter = _NPlanesMatch.begin();
    pA = _PlanesA[(iter->first-1)];
    pA->values[0] = -(pA->values[0]);
    pA->values[1] = -(pA->values[1]);
    pA->values[2] = -(pA->values[2]);
    pA->values[3] = -(pA->values[3]);

    slam(_cA,_cB);
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*transA,slam._T);
    currentScore = ICP::getFitnessError(transA,_B);
    if(currentScore<_Score)
    {
        _T = slam._T;
        _Score = currentScore;
    }
}

void ICP::PlanarSLAM::operator ()(
        std::vector<pcl::ModelCoefficients::Ptr>& A,
        std::vector<pcl::ModelCoefficients::Ptr>& B
        )
{

    int N = A.size();
    if(N!=B.size())
    {
        Pipe::error("Correspondent Planes Number Doesn't Match");
    }

    Eigen::MatrixXf X(3,N);
    Eigen::MatrixXf Y(3,N);
    Eigen::Vector3f t;
    Eigen::VectorXf dd(N);

    unsigned int idx=0;
    std::vector<pcl::ModelCoefficients::Ptr>::iterator iter;
    for(iter=A.begin();iter!=A.end();++iter)
    {
        X(0,idx) = (*iter)->values[0];
        X(1,idx) = (*iter)->values[1];
        X(2,idx) = (*iter)->values[2];
        dd(idx)  = (*iter)->values[3];
        ++idx;
    }
    idx=0;
    for(iter=B.begin();iter!=B.end();++iter)
    {
        Y(0,idx) = (*iter)->values[0];
        Y(1,idx) = (*iter)->values[1];
        Y(2,idx) = (*iter)->values[2];
        dd(idx) -= (*iter)->values[3];
        ++idx;
    }
    Eigen::MatrixXf H;
    H = X*(Y.transpose());
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H,Eigen::ComputeFullU|Eigen::ComputeFullV);

    Eigen::MatrixXf R;
    Eigen::Matrix3f I(Eigen::Matrix3f::Identity());
    R = svd.matrixV()*(svd.matrixU().transpose());

    if(R.determinant()<0)
    {
        I(2,2) = -1;
        R = svd.matrixV()*I*(svd.matrixU().transpose());
    }

    t = (Y.transpose()).colPivHouseholderQr().solve(dd);
    _T = Eigen::Matrix4f::Identity();
    _T.block(0,0,3,3) = R;
    _T.block(0,3,3,1) = t;
}

void ICP::MixedEstimator::operator ()(
        std::vector<pcl::ModelCoefficients::Ptr>& A,
        FullPointCloud::Ptr pA,
        std::vector<pcl::ModelCoefficients::Ptr>& B,
        FullPointCloud::Ptr pB
        )
{
    int Nnorm = A.size();
    int Npoint = pA->size();
    if( Nnorm != B.size() || Npoint != pB->size() )
    {
        Pipe::error("Correspondent Planes Number Doesn't Match");
    }

    Eigen::MatrixXf Xn(3,Nnorm);
    Eigen::MatrixXf Yn(3,Nnorm);
    Eigen::MatrixXf X(3,Npoint);
    Eigen::MatrixXf Y(3,Npoint);
    Eigen::Vector3f t;
    Eigen::VectorXf dd(Nnorm);
    Eigen::Vector4f cA;
    Eigen::Vector4f cB;
    Eigen::Vector3f cA3;
    Eigen::Vector3f cB3;

    pcl::compute3DCentroid(*pA,cA);
    pcl::compute3DCentroid(*pB,cB);

    cA3(0)=cA(0);cA3(1)=cA(1);cA3(2)=cA(2);
    cB3(0)=cB(0);cB3(1)=cB(1);cB3(2)=cB(2);

    int idx=0;
    int pidx;
    std::vector<pcl::ModelCoefficients::Ptr>::iterator iter;
    for(iter=A.begin();iter!=A.end();++iter)
    {
        Xn(0,idx) = (*iter)->values[0];
        Xn(1,idx) = (*iter)->values[1];
        Xn(2,idx) = (*iter)->values[2];
        dd(idx)  = (*iter)->values[3];
        ++idx;
    }
    for(pidx=0;pidx<pA->size();++pidx)
    {
        X(0,pidx) = pA->at(pidx).x - cA(0);
        X(1,pidx) = pA->at(pidx).y - cA(1);
        X(2,pidx) = pA->at(pidx).z - cA(2);
    }
    idx=0;
    for(iter=B.begin();iter!=B.end();++iter)
    {
        Yn(0,idx) = (*iter)->values[0];
        Yn(1,idx) = (*iter)->values[1];
        Yn(2,idx) = (*iter)->values[2];
        dd(idx) -= (*iter)->values[3];
        ++idx;
    }
    for(pidx=0;pidx<pB->size();++pidx)
    {
        Y(0,pidx) = pB->at(pidx).x - cB(0);
        Y(1,pidx) = pB->at(pidx).y - cB(1);
        Y(2,pidx) = pB->at(pidx).z - cB(2);
    }

    Eigen::Matrix3f H;
    Eigen::MatrixXf Xcombo( 3, Nnorm + Npoint );
    Eigen::MatrixXf Ycombo( 3, Nnorm + Npoint );

    Xcombo.block(0,0,Xn.rows(),Xn.cols()) = Xn;
    Xcombo.block(0,Xn.cols(),X.rows(),X.cols()) = X;
    Ycombo.block(0,0,Yn.rows(),Yn.cols()) = Yn;
    Ycombo.block(0,Yn.cols(),Y.rows(),Y.cols()) = Y;

    H = Xcombo*(Ycombo.transpose());

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H,Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::MatrixXf R;

    Eigen::Matrix3f I(Eigen::Matrix3f::Identity());
    R = svd.matrixV()*(svd.matrixU().transpose());

    if(R.determinant()<0)
    {
        I(2,2) = -1;
        R = svd.matrixV()*I*(svd.matrixU().transpose());
    }

    Eigen::MatrixXf Y2(Yn.rows(),Yn.cols()+3);
    Eigen::VectorXf dd2(Yn.cols()+3);

    Y2.block(0,0,Yn.rows(),Yn.cols()) = Yn;
    Y2.block(0,Yn.cols(),3,1) = Eigen::Vector3f(1,0,0);
    Y2.block(0,Yn.cols()+1,3,1) = Eigen::Vector3f(0,1,0);
    Y2.block(0,Yn.cols()+2,3,1) = Eigen::Vector3f(0,0,1);

    dd2.head(dd.size()) = dd;
    Eigen::Vector3f t_ = cB3 - R*cA3;

    dd2(Yn.cols()) = t_(0);
    dd2(Yn.cols()+1) = t_(1);
    dd2(Yn.cols()+2) = t_(2);

    t = (Y2.transpose()).colPivHouseholderQr().solve(dd2);

    _T = Eigen::Matrix4f::Identity();
    _T.block(0,0,3,3) = R;
    _T.block(0,3,3,1) = t;
}

void ICP::RotateAloneZ::operator ()(
        const FullPointCloud::Ptr& A,
        FullPoint& CenterA,
        const FullPointCloud::Ptr& B,
        FullPoint& CenterB
        )
{
    Eigen::Matrix4f scT(Eigen::Matrix4f::Identity());
    scT(0,3) = CenterA.x;
    scT(1,3) = CenterA.y;
    scT(2,3) = CenterA.z;
    Eigen::Matrix4f tcT(Eigen::Matrix4f::Identity());
    tcT(0,3) = CenterB.x;
    tcT(1,3) = CenterB.y;
    tcT(2,3) = CenterB.z;

    unsigned int k = 0;
    _Score = std::numeric_limits<double>::max();
    double score;
    Eigen::Matrix4f _t;
    Eigen::Matrix4f best_t;

    rCloud = FullPointCloud::Ptr(new FullPointCloud);

    int N = 36;

    for(k=0;k<N;++k)
    {
        Eigen::Matrix4f guess(Eigen::Matrix4f::Identity());
        Eigen::Affine3f t;
        t = pcl::getTransformation(0,0,0,0,0,k*2*M_PI/N);
        guess = tcT*t*(scT.inverse());
        pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,guess);
        score = ICP::getFitnessError(rCloud,B);
        if(_isnan(score))
        {
            score = std::numeric_limits<double>::max();
        }
        if(score<_Score)
        {
            _Score = score;
            best_t = _t;
        }
    }
    _T = best_t;
    pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,_T);
}

void ICP::ICPMixRandInit::operator()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B)
{
    if( A->size() > B->size() )
    {
        _A = B;
        _B = A;
    }else{
        _A = A;
        _B = B;
    }
    rCloud = FullPointCloud::Ptr(new FullPointCloud);
    detectPlanes(_A,_PlanesLabelA,_PlanesA,_PlaneCenterA,_PlaneHistsA,_NonPlaneIndexA);
    detectPlanes(_B,_PlanesLabelB,_PlanesB,_PlaneCenterB,_PlaneHistsB,_NonPlaneIndexB);
    if( _PlanesA.size() < 2 || _PlanesB.size() < 2 )
    {
        estimateTransformationCenter();
    }else{
        generatePossibleCorrs();
        estimateTransformation();
    }
}

void ICP::ICPMixRandInit::estimateTransformationCenter(void)
{
    FullPoint centerA;
    FullPoint centerB;
    ICP::RotateAloneZ rotate;
    if( _PlanesA.size() == 0  )
    {
        Pipe::getPCDBoxCenter(_A,centerA);
        Pipe::getPCDBoxCenter(_B,centerB);
        rotate(_A,centerA,_B,centerB);
        _T = rotate._T;
    }else{
        unsigned int idx;
        centerA = _PlaneCenterA->at(0);
        _Score = std::numeric_limits<float>::max();
        for(idx=0;idx<_PlanesB.size();++idx)
        {
            centerB = _PlaneCenterB->at(idx);
            rotate(_A,centerA,_B,centerB);
            if( _Score > rotate._Score )
            {
                _T = rotate._T;
                _Score = rotate._Score;
            }
        }
    }

}

void ICP::ICPMixRandInit::detectPlanes(
        const FullPointCloud::Ptr& cloud,
        cv::Mat&labels,
        std::vector<pcl::ModelCoefficients::Ptr>&models,
        FullPointCloud::Ptr& centers,
        std::vector<cv::Mat>&hists,
        std::vector<int>&remaining
        )
{
    labels.create(1,cloud->size(),CV_32SC1);
    labels.setTo(-1);
    _MinPlanePointNum = 200;
    detectPlanes(cloud,labels,models,centers,hists,remaining,pcl::SACMODEL_NORMAL_PLANE);
}

void ICP::ICPMixRandInit::detectPlanes(const FullPointCloud::Ptr& cloud,
        cv::Mat&labels,
        std::vector<pcl::ModelCoefficients::Ptr>&models,
        FullPointCloud::Ptr& centers,
        std::vector<cv::Mat>&hists,
        std::vector<int>&remaining,
        int plane_type
        )
{
    Pipe::inform("detecting planes");
    unsigned int idx;
    pcl::SACSegmentationFromNormals<FullPoint,FullPoint> seg;
    pcl::ExtractIndices<FullPoint>extract;

    extract.setNegative (false);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (plane_type);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(M_PI/36);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setNormalDistanceWeight(0.1);
    seg.setRadiusLimits(0.5*ICP::DOWN_SAMPLE_GRID_SIZE,2*ICP::DOWN_SAMPLE_GRID_SIZE);
    seg.setDistanceThreshold (0.03);
    unsigned int count = 0;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
    FullPointCloud::Ptr cloud_p(new FullPointCloud);
    centers = FullPointCloud::Ptr(new FullPointCloud);

    outliers->indices.clear();
    for(idx=0;idx<labels.cols;++idx)
    {
        if(-1==labels.at<int>(0,idx))
        outliers->indices.push_back(idx);
    }
    Pipe::inform("starting plane iter");
    while(count<_MaxPlaneNum)
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud);
        seg.setIndices(outliers);
        seg.segment(*inliers,*coefficients);
        if( inliers->indices.size() < _MinPlanePointNum ){
            break;
        }

        //add new plane
        models.push_back(coefficients);
        for(idx=0;idx<inliers->indices.size();++idx)
        {
            labels.at<int>(0,inliers->indices[idx]) = models.size();
        }

        //extract plane and calculate features
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.filter (*cloud_p);

        //plane features
        cv::Mat colorhist;
        cv::Mat sizehist;

        colorhist = Histogram::getColorHist(cloud_p);
        FullPoint planecenter;
        sizehist = Histogram::getDifSize(cloud_p);
        Pipe::getPCDCenter(cloud_p,planecenter);
        centers->push_back(planecenter);

        hists.push_back(cv::Mat());
        colorhist.copyTo(hists.back());
        hists.push_back(cv::Mat());
        sizehist.copyTo(hists.back());

        //label remaining
        outliers->indices.clear();
        for(idx=0;idx<labels.cols;++idx)
        {
            if(-1==labels.at<int>(0,idx))
            outliers->indices.push_back(idx);
        }
        ++count;
    }
    remaining = outliers->indices;
    centers->height = 1;
    centers->width = centers->size();
    std::cerr<<"plane num:"<<count<<std::endl;
    Pipe::inform("done plane iter");
}

void ICP::ICPMixRandInit::getPlanesA( cv::Mat& labels)
{
    _PlanesLabelA.copyTo(labels);
}

void ICP::ICPMixRandInit::getPlanesB( cv::Mat& labels)
{
    _PlanesLabelB.copyTo(labels);
}

bool ICP::ICPMixRandInit::validateCorrespondence( MixedCorrespondence& c )
{
    if( 0 > ( std::abs(_PlanesA[c.planeA0]->values[2]) - 0.707 )
            *( std::abs(_PlanesB[c.planeB0]->values[2]) - 0.707 )
     )return false;

    if( 0 > ( std::abs(_PlanesA[c.planeA1]->values[2]) - 0.707 )
            *( std::abs(_PlanesB[c.planeB1]->values[2]) - 0.707 )
     )return false;

    return true;
}

void ICP::ICPMixRandInit::generatePossibleCorrs(void)
{
    MixedCorrespondence corr;
    int plA0,plA1,plB0,plB1;
    for( plA0 = 0 ; plA0 < _PlanesA.size() - 1 ; ++plA0 )
    for( plA1 = plA0 + 1 ; plA1 < _PlanesA.size() ; ++plA1 )
    for( plB0 = 0 ; plB0 < _PlanesB.size() ; ++plB0 )
    for( plB1 = 0 ; plB1 < _PlanesB.size() ; ++plB1 )
    {
        if( plB0 == plB1 )continue;
        corr.planeA0 = plA0;
        corr.planeA1 = plA1;
        corr.planeB0 = plB0;
        corr.planeB1 = plB1;
        corr.pointA = plA0;
        corr.pointB = plB0;
        corr.score = 0.0;
        if(validateCorrespondence(corr))
        {
            _Corr.push_back(MixedCorrespondence());
            _Corr.back() = corr;
        }
        corr.planeA0 = plA0;
        corr.planeA1 = plA1;
        corr.planeB0 = plB0;
        corr.planeB1 = plB1;
        corr.pointA = plA1;
        corr.pointB = plB1;
        corr.score = 0.0;
        if(validateCorrespondence(corr))
        {
            _Corr.push_back(MixedCorrespondence());
            _Corr.back() = corr;
        }
    }
    std::cerr<<"Correspondence Num:"<<_Corr.size()<<std::endl;
}

void ICP::ICPMixRandInit::flipPlaneDirection(pcl::ModelCoefficients::Ptr plane)
{
    plane->values[0] = -plane->values[0];
    plane->values[1] = -plane->values[1];
    plane->values[2] = -plane->values[2];
    plane->values[3] = -plane->values[3];
}

void ICP::ICPMixRandInit::getMatchedPlanesA(cv::Mat &labels)
{
    _PlanesLabelA.copyTo(labels);
    if(_PlanesA.size()<2)return;
    std::map<int,int> label_map;
    label_map[_BestCorr.planeA0+1] = 1;
    label_map[_BestCorr.planeA1+1] = 2;
    for(int idx=0;idx<labels.cols;++idx)
    {
        int lbl = labels.at<int>(0,idx);
        if(label_map.find(lbl)!=label_map.end())
        {
            labels.at<int>(0,idx) = label_map[lbl];
        }else{
            labels.at<int>(0,idx) = -1;
        }
    }
}

void ICP::ICPMixRandInit::getMatchedPlanesB(cv::Mat &labels)
{
    _PlanesLabelB.copyTo(labels);
    if(_PlanesB.size()<2)return;
    std::map<int,int> label_map;
    label_map[_BestCorr.planeB0+1] = 1;
    label_map[_BestCorr.planeB1+1] = 2;
    for(int idx=0;idx<labels.cols;++idx)
    {
        int lbl = labels.at<int>(0,idx);
        if(label_map.find(lbl)!=label_map.end())
        {
            labels.at<int>(0,idx) = label_map[lbl];
        }else{
            labels.at<int>(0,idx) = -1;
        }
    }
}

void ICP::ICPMixRandInit::getPlaneCorr(std::vector<std::pair<int,int>>&corres)
{
    corres.push_back(std::pair<int,int>(_BestCorr.planeA0+1,_BestCorr.planeB0+1));
    corres.push_back(std::pair<int,int>(_BestCorr.planeA1+1,_BestCorr.planeB1+1));
}

void ICP::ICPMixRandInit::getPointsCorr(
        FullPointCloud::Ptr& pA,
        FullPointCloud::Ptr& pB
        )
{
    pA = FullPointCloud::Ptr(new FullPointCloud);
    pB = FullPointCloud::Ptr(new FullPointCloud);
    pA->width = pA->size();
    pA->height = 1;
    pB->width = pB->size();
    pB->height = 1;
}

void ICP::ICPMixRandInit::getDerivedPointsCorr(
        FullPointCloud::Ptr& pA,
        FullPointCloud::Ptr& pB
        )
{
    pA = FullPointCloud::Ptr(new FullPointCloud);
    pB = FullPointCloud::Ptr(new FullPointCloud);
    if(_PlanesA.size()>=2&&_PlanesB.size()>=2)
    {
        pA->push_back( _PlaneCenterA->at(_BestCorr.pointA) );
        pB->push_back( _PlaneCenterB->at(_BestCorr.pointB) );
    }
    pA->width = pA->size();
    pA->height = 1;
    pB->width = pB->size();
    pB->height = 1;
}

void ICP::ICPMixRandInit::estimateTransformation(void)
{
    std::vector<pcl::ModelCoefficients::Ptr> _cA;
    FullPointCloud::Ptr _pA(new FullPointCloud);
    std::vector<pcl::ModelCoefficients::Ptr> _cB;
    FullPointCloud::Ptr _pB(new FullPointCloud);
    _Score = 0;
    rCloud = FullPointCloud::Ptr(new FullPointCloud);
    float score;
    ICP::MixedEstimator est;
    std::vector<MixedCorrespondence>::iterator iter;
    _BestCorr.score = 0.0;
    for(iter=_Corr.begin();iter!=_Corr.end();++iter)
    {
        _cA.clear();
        _cB.clear();
        _pA->clear();
        _pB->clear();
        _cA.push_back(_PlanesA[iter->planeA0]);
        _cA.push_back(_PlanesA[iter->planeA1]);
        _cB.push_back(_PlanesB[iter->planeB0]);
        _cB.push_back(_PlanesB[iter->planeB1]);
        _pA->push_back(_PlaneCenterA->at(iter->pointA));
        _pB->push_back(_PlaneCenterB->at(iter->pointB));
        Eigen::Map<Eigen::Matrix4f> t(iter->t);
        //flip none
        est(_cA,_pA,_cB,_pB);
        pcl::transformPointCloudWithNormals<FullPoint>(*_A,*rCloud,est._T);
        score = ICP::getOverlapNorm(rCloud,_B);
        if( iter->score < score )
        {
            iter->score = score ;
            t = est._T ;
        }
        //flip first
        flipPlaneDirection(_PlanesA[iter->planeA0]);
        est(_cA,_pA,_cB,_pB);
        pcl::transformPointCloudWithNormals<FullPoint>(*_A,*rCloud,est._T);
        score = ICP::getOverlapNorm(rCloud,_B);
        if( iter->score < score )
        {
            iter->score = score ;
            t = est._T ;
        }
        //flip both
        flipPlaneDirection(_PlanesA[iter->planeA1]);
        est(_cA,_pA,_cB,_pB);
        pcl::transformPointCloudWithNormals<FullPoint>(*_A,*rCloud,est._T);
        score = ICP::getOverlapNorm(rCloud,_B);
        if( iter->score < score )
        {
            iter->score = score ;
            t = est._T ;
        }
        //flip second
        flipPlaneDirection(_PlanesA[iter->planeA0]);
        est(_cA,_pA,_cB,_pB);
        pcl::transformPointCloudWithNormals<FullPoint>(*_A,*rCloud,est._T);
        score = ICP::getOverlapNorm(rCloud,_B);
        if( iter->score < score )
        {
            iter->score = score ;
            t = est._T ;
        }
        if( _BestCorr.score < iter->score )
        {
            _BestCorr = *iter;
        }
    }
    std::cerr<<"Best Correspondence"<<std::endl;
    std::cerr<<"Planes:"<<std::endl;
    std::cerr<<_BestCorr.planeA0<<" To "<<_BestCorr.planeB0<<std::endl;
    std::cerr<<_BestCorr.planeA1<<" To "<<_BestCorr.planeB1<<std::endl;
    std::cerr<<"Points:"<<std::endl;
    std::cerr<<_BestCorr.pointA<<" To "<<_BestCorr.pointB<<std::endl;
    std::cerr<<_BestCorr.score<<std::endl;

    Eigen::Map<Eigen::Matrix4f> transform(_BestCorr.t);
    _T = transform;
    pcl::transformPointCloudWithNormals<FullPoint>(*_A,*rCloud,_T);
    _Score = ICP::getFitnessError(rCloud,_B);
}

void ICP::ICPSparse::operator()(const FullPointCloud::Ptr&A,const FullPointCloud::Ptr&B)
{
    Pipe::inform("ICP");
    QTime time;
    time.start();
    rCloud = FullPointCloud::Ptr(new FullPointCloud);
    pcl::transformPointCloudWithNormals<FullPoint>(*A,*rCloud,_T);
    Vertices vertices_source;
    Vertices vertices_target;
    Vertices vertices_tarnorm;
    VerticesFromCloudXYZ(rCloud,vertices_source);
    VerticesFromCloudXYZ(B,vertices_target);
    VerticesFromCloudNorm(B,vertices_tarnorm);
    SICP::Parameters pars;
    pars.p = .5;
    pars.max_icp = 15;
    pars.print_icpn = true;
    Eigen::Matrix4d _Td;
    _Td = SICP::point_to_plane(vertices_source,vertices_target,vertices_tarnorm, pars).matrix();
    Eigen::Matrix4f initT = _T;
    _T(0,0) = float(_Td(0,0));_T(0,1) = float(_Td(0,1));_T(0,2) = float(_Td(0,2));_T(0,3) = float(_Td(0,3));
    _T(1,0) = float(_Td(1,0));_T(1,1) = float(_Td(1,1));_T(1,2) = float(_Td(1,2));_T(1,3) = float(_Td(1,3));
    _T(2,0) = float(_Td(2,0));_T(2,1) = float(_Td(2,1));_T(2,2) = float(_Td(2,2));_T(2,3) = float(_Td(2,3));
    _T(3,0) = float(_Td(3,0));_T(3,1) = float(_Td(3,1));_T(3,2) = float(_Td(3,2));_T(3,3) = float(_Td(3,3));
    _T = _T*initT;
    Pipe::inform("End ICP");
    std::cerr<<"Used:"<<time.elapsed()<<"ms"<<std::endl;
}

void ICP::ICPSparse::VerticesFromCloudXYZ(const FullPointCloud::Ptr&cloud,Vertices&vertices)
{
    vertices = Eigen::Matrix3Xd(3,cloud->size());
    int idx;
    for(idx=0;idx<cloud->size();++idx)
    {
        vertices(0,idx)=cloud->at(idx).x;
        vertices(1,idx)=cloud->at(idx).y;
        vertices(2,idx)=cloud->at(idx).z;
    }
}

void ICP::ICPSparse::VerticesFromCloudNorm(const FullPointCloud::Ptr&cloud,Vertices&vertices)
{
    vertices = Eigen::Matrix3Xd(3,cloud->size());
    int idx;
    for(idx=0;idx<cloud->size();++idx)
    {
        vertices(0,idx)=cloud->at(idx).normal_x;
        vertices(1,idx)=cloud->at(idx).normal_y;
        vertices(2,idx)=cloud->at(idx).normal_z;
    }
}
