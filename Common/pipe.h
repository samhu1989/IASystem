#ifndef PIPE_H
#define PIPE_H
#include "common_global.h"
#include "configure.h"
#include <string>
#include <QHash>
#include <QTime>
#include <QDate>
#include <QVariant>
#include <QVector>
#include <opencv2/opencv.hpp>
#include "icp.h"
class COMMONSHARED_EXPORT Pipe
{
public:
    typedef Pipe*(*Fetcher)(void);
    Pipe();
    virtual ~Pipe();

    virtual std::string name(){return "Pipe";}
    virtual __int32 version(){return 0;}

    static void error(const std::string&);
    static void warn(const std::string&);
    static void inform(const std::string&);
    static void record(const std::string&);
    static std::string timestamp();

    static QHash<quint32,QByteArray> _PipeData;
    static void loadPipeDataFrom(const std::string&);//load from given path
    static void dumpTo(const std::string&);// dump to given path
    static void dump(void);// dump to fix path and name after time
    static quint32 generateKey(void);


    static quint32 addToData(FullPointCloud::Ptr,QString custom=QString(""));
    static void    addToData(FullPointCloud::Ptr,quint32,QString);
    static bool    loadData(FullPointCloud::Ptr &pcd,QString& custom,quint32 key);
    static void    addToData(const QList<quint32>& keyList,quint32 key,QString custom=QString(""));
    static bool    loadData(QList<quint32>& keyList,QString& custom,quint32 key);
    static quint32 addToData(const cv::Mat&,QString);
    static void    addToData(const cv::Mat&,quint32,QString);
    static bool    loadData(cv::Mat&,QString&,quint32 key);
    static void    addToData(const QVector<QVector<quint32>>&,quint32,QString);
    static bool    loadData(QVector<QVector<quint32>>&,QString&,quint32);

    static void pcd2rgbd(quint32,quint32,const FullPointCloud::Ptr&,const Eigen::Matrix3f&,cv::Mat&);
    static void pcd2rgbd(quint32,quint32,const FullPointCloud::Ptr&,const Eigen::Matrix3f&,cv::Mat&,unsigned int&withinSight);
    static void rgbd2pcd(cv::Mat&,const Eigen::Matrix3f&,FullPointCloud::Ptr&);
    static void visualize(FullPointCloud::Ptr&);
    static void getPCDCenter(const FullPointCloud::Ptr&,FullPoint& p);
    static void getPCDBoxCenter(const FullPointCloud::Ptr&pcd, FullPoint& center);

    static void debug2DOBB(const FullPointCloud::Ptr&,const cv::Mat&);
    static void debug2DOBB(const std::vector<FullPointCloud::Ptr>& pcd,std::vector<cv::Mat>&layouts,std::vector<unsigned int>&obj_id);
    static void debug2DOBB(const std::vector<FullPointCloud::Ptr>& pcd,std::vector<cv::Mat>&layouts,std::vector<unsigned int>&obj_id,QImage&);

    static void debug2DRelation(const Eigen::MatrixXf& centers, cv::Mat& relation, QImage&output);

    //error functions to measure the match of point cloud
    static double RGB_Energy(uchar r0,uchar g0,uchar b0,uchar r1, uchar g1, uchar b1);
    static double RGB_Dist(uchar r0,uchar g0,uchar b0,uchar r1, uchar g1, uchar b1);

    static double match3D(
                        const FullPointCloud::Ptr&input,
                        const Eigen::Matrix3f &intrinsic,
                        const FullPointCloud::Ptr&reference,
                        std::vector<double>& scoreList,
                        double Occlusion_Threshold = 0.05
                        );
    static double match(const FullPointCloud::Ptr&input,
                        const Eigen::Matrix3f& intrinsic,
                        const FullPointCloud::Ptr& reference,
                        double Occlusion_Threshold = 0.05
                        );
    static double match(
                        const FullPointCloud::Ptr&input,
                        const Eigen::Matrix3f& intrinsic,
                        const FullPointCloud::Ptr& reference,
                        std::vector<double>& scoreList,
                        double Occlusion_Threshold = 0.05
                        );

    static void doICP(const FullPointCloud::Ptr& s,const FullPointCloud::Ptr& t,Eigen::Matrix4f& transform,bool ifDownSample=true);

    //output idmap to false color image
    static void IdMap2Img(const FullPointCloud::Ptr& cloud,const cv::Mat& idmap,QImage&output);
    static void IdMap2Img(const FullPointCloud::Ptr&,const FullPointCloud::Ptr&,const cv::Mat&,const std::vector<cv::Mat>&,const Eigen::Matrix3f&,QImage&);

    static void IdMap2Img(const FullPointCloud::Ptr& cloud,const cv::Mat& idmap,std::string filename);
    static void IdMap2Img(const FullPointCloud::Ptr&,const FullPointCloud::Ptr&,const cv::Mat&,const std::vector<cv::Mat>&,const Eigen::Matrix3f&,std::string filename);
    static void IdMap2Img(const FullPointCloud::Ptr&,const std::vector<cv::Mat>&,const Eigen::Matrix3f&,std::string filename);

    static void getT(const cv::Mat&,Eigen::Matrix4f&);
    static void getT(const cv::Mat&,Eigen::MatrixXf&);
    static void getMat(const Eigen::Matrix4f&,cv::Mat&);
    static void getMat(const Eigen::MatrixXf& T,cv::Mat& mat);

    static void getSegList(
                    const FullPointCloud::Ptr&framefgCloud,
                    cv::Mat&idmap,
                    std::vector<FullPointCloud::Ptr>&Segs
                    );

    static void transformOBB(const cv::Mat& invTransMat,cv::Mat& boxVertex);
    static void doFalseColorOnPCD(unsigned int coloridx,FullPointCloud::Ptr&);
    static void doFalseColorOnPCD(cv::Mat& coloridx,FullPointCloud::Ptr&);

    static void get2DOBB(const FullPointCloud::Ptr&, cv::Mat&);
    static void get2DOBBVar(const FullPointCloud::Ptr&,cv::Mat&);
    static void getBoundingBox( const FullPointCloud::Ptr& obj, cv::Mat& vertexes );

    static void debug(const FullPointCloud::Ptr& a,const FullPointCloud::Ptr& b,const Eigen::Matrix4f& T,const std::string& path);
    static void debug(const std::vector<FullPointCloud::Ptr>&,const std::string& path);
    static void debug(const std::vector<cv::Mat>& depth,const std::string& path);
    static int debug(FullPointCloud::Ptr &a, FullPointCloud::Ptr &b, bool& merge);

    /*img log related*/
    //enable painting function
    static void startImgLog(void);
    static void addImgToLog(quint32 row,quint32 col,QImage&);
    //transfer pcd to rgb image
    static QImage pcdToImg(const FullPointCloud::Ptr&);
    //project pcd onto image
    static QImage pcdToImg(quint32 w,quint32 h,const FullPointCloud::Ptr&,const Eigen::Matrix3f&);
    static QImage overlapImg(QImage& bg,QImage&fg);
    static QImage hScrollPainting(QList<QImage>&);

    static QImage drawTextForLogUnit(const QImage&);
    static void endImgLog(void);

    static QVector<QVariant> parseDataHeadAt(quint32 row);    
    static quint32 _ImgLogUnitWidth;
    static quint32 _ImgLogUnitHeight;
    static int _LastRow;
    static int _LastCol;
    static QVector<QVector<QImage>> _ImgLog;
    static QString _ImgLogFile;

    static bool _ErrorToStdErr;//to screen
    static bool _ErrorToStdOut;//to log file
    static bool _WarnToStdErr;//to screen
    static bool _WarnToStdOut;//to log file
    static bool _InformToStdErr;//to screen
    static bool _InformToStdOut;//to log file
    static QDate _Date;
    static QTime _Time;

    static void outputFalseColorBar(unsigned int firstK,std::string path);
    const static unsigned int  FalseColorNum = 25;
    static unsigned char FalseColor[FalseColorNum][3];

    static quint32 _FrameListKey;
    static quint32 _IdMapListKey;
    static quint32 _SegListKey;
    static quint32 _BGKey;
    static quint32 _BGScoreKey;
    static quint32 _ObjListKey;
    static quint32 _ObjScoreKey;
    static quint32 _DepthMapKey;
    static quint32 _CamKey;
    static quint32 _CamExKey;

    virtual int configure(Config&);
    virtual int init(void); //initialize pipe
    virtual int work(void); //do computation
    virtual int saveToData(void); //save result to global buffer
};
namespace pcl{
template <typename PointT> void
removeZeroFromPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                               std::vector<int> &index)
{
    // If the clouds are not the same, prepare the output
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize (cloud_in.points.size ());
    }
    // Reserve enough space for the indices
    index.resize (cloud_in.points.size ());
    size_t j = 0;

    // If the data is dense, we don't need to check for NaN
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    {
        if (0==cloud_in.points[i].x||
                0==cloud_in.points[i].y||
                0==cloud_in.points[i].z)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        index[j] = static_cast<int>(i);
        j++;
    }
    if (j != cloud_in.points.size ())
    {
        // Resize to the correct size
        cloud_out.points.resize (j);
        index.resize (j);
        cloud_out.height = 1;
        cloud_out.width  = static_cast<uint32_t>(j);
    }

}
}

#define LACK_CONFIG(a) Pipe::warn("Missing Config: "+a)
#define NECESSARY_CONFIG() Pipe::error("The Missing Config is Mandatory")
#define TOID(row, col, width) (row*width+col)
#endif // PIPE_H
