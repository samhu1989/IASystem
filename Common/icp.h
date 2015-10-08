#ifndef ICP_H
#define ICP_H
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/correspondence.h>
#include <opencv2/core/core.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include "common_global.h"
typedef pcl::PointXYZRGBNormal FullPoint;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> FullPointCloud;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFHCloud;
typedef pcl::PointCloud<pcl::PFHSignature125> PFHCloud;
typedef pcl::PointCloud<pcl::PFHRGBSignature250> PFHRGBCloud;
class COMMONSHARED_EXPORT ICP
{
public:

    static double DOWN_SAMPLE_GRID_SIZE;
    static int MIN_SAMPLE_DISTANCE;
    static int SAMPLE_NUM;
    static int K_NEAREAST_RANDOM;
    static int MAX_ITERATION_NUM_INIT;
    static double TRANSFORM_EPS;
    static double FITNESS_EPS;
    static double MAX_CORRESPONDENCE_DISTANCE_FOR_INIT;
    static double MAX_CORRESPONDENCE_DISTANCE_FOR_ERROR;
    static int FEATURE_SEARCH_K;
    static double MAX_CORRESPONDENCE_DISTANCE_FOR_LOOP;
    static int MAX_ITERATION_NUM_LOOP;
    static double MAX_NON_OCCLUSION;

    static double getFitnessError(const FullPointCloud::Ptr&,const FullPointCloud::Ptr&,const std::string&);
    static double getFitnessError(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
    static double getOverlapNum(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
    static double getOverlapNorm(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);

    class COMMONSHARED_EXPORT DownSampler
    {
    public:
        DownSampler(void){
            downSampled = FullPointCloud::Ptr(new FullPointCloud);
        }
        void operator()(const FullPointCloud::Ptr& in);
        FullPointCloud::Ptr downSampled;

    };
    class COMMONSHARED_EXPORT FeatureExtractorPFHRGB
    {
    public:
        FeatureExtractorPFHRGB(void){
            feature = PFHRGBCloud::Ptr(new PFHRGBCloud);
        }
        void operator()(const FullPointCloud::Ptr& in);
        PFHRGBCloud::Ptr feature;

    };
    template<typename Feature,typename FeatureExtractor>
    class ICPInit
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    };
    class COMMONSHARED_EXPORT ICPFiner
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    };

    class COMMONSHARED_EXPORT ICPSparse
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef Eigen::Matrix3Xd Vertices;
        ICPSparse():_T(Eigen::Matrix4f::Identity()){;}
        void operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    protected:
        void VerticesFromCloudXYZ(const FullPointCloud::Ptr&cloud,Vertices&vertices);
        void VerticesFromCloudNorm(const FullPointCloud::Ptr&cloud,Vertices&vertices);
    };

    class COMMONSHARED_EXPORT ICPGeneral
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    };
    template<typename PointNormalT>
    class PointCurve : public pcl::PointRepresentation<PointNormalT>
    {
      using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
      public:
      PointCurve ()
      {
        // Define the number of dimensions
        nr_dimensions_ = 4;
      }
      virtual void copyToFloatArray (const PointNormalT &p, float * out) const
      {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
      }
    };
    class COMMONSHARED_EXPORT ICPPoint2Plane
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    };
    class COMMONSHARED_EXPORT PlanarSLAM
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(
                std::vector<pcl::ModelCoefficients::Ptr>& A,
                std::vector<pcl::ModelCoefficients::Ptr>& B
                );
        Eigen::Matrix4f _T;
    };
    class COMMONSHARED_EXPORT MixedEstimator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(
                std::vector<pcl::ModelCoefficients::Ptr>& A,
                FullPointCloud::Ptr pA,
                std::vector<pcl::ModelCoefficients::Ptr>& B,
                FullPointCloud::Ptr pB
                );
        Eigen::Matrix4f _T;
    };
    class COMMONSHARED_EXPORT RotateAloneZ
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(
                const FullPointCloud::Ptr& A,
                FullPoint& CenterA,
                const FullPointCloud::Ptr& B,
                FullPoint& CenterB
                );
        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    };
    class COMMONSHARED_EXPORT ICPMixedInit
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);
        void getPlanesA( cv::Mat& );
        void getPlanesB( cv::Mat& );
        void getMatchedPlanesA( cv::Mat& );
        void getMatchedPlanesB( cv::Mat& );
        void getPointsCorr(
                FullPointCloud::Ptr& pA,
                FullPointCloud::Ptr& pB
                );
        void getDerivedPointsCorr(
                FullPointCloud::Ptr& pA,
                FullPointCloud::Ptr& pB
                );
        void getPlaneCorr(std::vector<std::pair<int,int>>&corres);
        void setMaxPlaneNum(int num){_MaxPlaneNum = num;}
        int getMaxPlaneNum(void){return _MaxPlaneNum;}
        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    protected:
        void detectPlanes(const FullPointCloud::Ptr& cloud,
                cv::Mat&labels,
                std::vector<pcl::ModelCoefficients::Ptr>&models,
                FullPointCloud::Ptr &centers,
                std::vector<cv::Mat>&hists,
                std::vector<int>&remaining
                );
        void detectPlanes(
                const FullPointCloud::Ptr& cloud,
                cv::Mat&labels,
                std::vector<pcl::ModelCoefficients::Ptr>&models,
                FullPointCloud::Ptr &centers,
                std::vector<cv::Mat>&hists,
                std::vector<int>&remaining,
                int plane_type
                );
        void assignPlanes(void);
        void derivePoints(void);
        void assignPoints(void);
        void estimateTransformationPlane(void);
        void estimateTransformationCenter(void);
        void estimateTransformationMixed(void);

    private:
        int _MaxPlaneNum;
        int _MinPlanePointNum;
        FullPointCloud::Ptr _A;
        FullPointCloud::Ptr _B;
        std::vector<int> _NonPlaneIndexA;
        std::vector<int> _NonPlaneIndexB;
        cv::Mat _PlanesLabelA;
        cv::Mat _PlanesLabelB;
        std::vector<pcl::ModelCoefficients::Ptr> _PlanesA;
        FullPointCloud::Ptr _PlaneCenterA;
        std::vector<cv::Mat> _PlaneHistsA;
        std::vector<pcl::ModelCoefficients::Ptr> _PlanesB;
        FullPointCloud::Ptr _PlaneCenterB;
        std::vector<cv::Mat> _PlaneHistsB;
        std::vector<std::pair<int,int>>_ZPlanesMatch; // matched plane of parallel to z-axis
        std::vector<std::pair<int,int>>_NPlanesMatch; // matched plane of parallel to z-axis
        std::vector<std::pair<int,int>> _PointsMatch;
        FullPointCloud::Ptr _DerivedPointsA;
        FullPointCloud::Ptr _DerivedPointsB;
    };
    class COMMONSHARED_EXPORT ICPMixRandInit
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef struct{
            int planeA0;
            int planeA1;
            int planeB0;
            int planeB1;
            int pointA;
            int pointB;
            double score;
            float t[16];
        }MixedCorrespondence;

        void operator ()(const FullPointCloud::Ptr& A,const FullPointCloud::Ptr& B);

        void getPlanesA( cv::Mat& );
        void getPlanesB( cv::Mat& );

        void getMatchedPlanesA( cv::Mat& );
        void getMatchedPlanesB( cv::Mat& );
        void getPlaneCorr(std::vector<std::pair<int,int>>&corres);

        void getPointsCorr(
                FullPointCloud::Ptr& pA,
                FullPointCloud::Ptr& pB
                );
        void getDerivedPointsCorr(
                FullPointCloud::Ptr& pA,
                FullPointCloud::Ptr& pB
                );

        void setMaxPlaneNum(int num){_MaxPlaneNum = num;}
        int getMaxPlaneNum(void){return _MaxPlaneNum;}

        Eigen::Matrix4f _T;
        double _Score;
        FullPointCloud::Ptr rCloud;
    protected:
        void detectPlanes(const FullPointCloud::Ptr& cloud,
                cv::Mat&labels,
                std::vector<pcl::ModelCoefficients::Ptr>&models,
                FullPointCloud::Ptr &centers,
                std::vector<cv::Mat>&hists,
                std::vector<int>&remaining
                );
        void detectPlanes(
                const FullPointCloud::Ptr& cloud,
                cv::Mat&labels,
                std::vector<pcl::ModelCoefficients::Ptr>&models,
                FullPointCloud::Ptr &centers,
                std::vector<cv::Mat>&hists,
                std::vector<int>&remaining,
                int plane_type
                );
        void flipPlaneDirection(pcl::ModelCoefficients::Ptr);
        void estimateTransformationCenter(void);
        void generatePossibleCorrs(void);
        void estimateTransformation(void);
        bool validateCorrespondence(MixedCorrespondence&c);
    private:
        int _MaxPlaneNum;
        int _MinPlanePointNum;
        FullPointCloud::Ptr _A;
        FullPointCloud::Ptr _B;
        std::vector<int> _NonPlaneIndexA;
        std::vector<int> _NonPlaneIndexB;
        cv::Mat _PlanesLabelA;
        cv::Mat _PlanesLabelB;
        std::vector<pcl::ModelCoefficients::Ptr> _PlanesA;
        FullPointCloud::Ptr _PlaneCenterA;
        std::vector<cv::Mat> _PlaneHistsA;
        std::vector<pcl::ModelCoefficients::Ptr> _PlanesB;
        FullPointCloud::Ptr _PlaneCenterB;
        std::vector<cv::Mat> _PlaneHistsB;
        std::vector<MixedCorrespondence> _Corr;
        MixedCorrespondence _BestCorr;
    };
};
#include "icp.hpp"
#endif // ICP_H
