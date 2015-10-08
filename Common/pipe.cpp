#include "pipe.h"
#include <iostream>
#include <sstream>
#include <QDataStream>
#include <QFile>
#include <random>
#include <QVector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <QImage>
#include <QPainter>
#include <QTextStream>
#include <QPointF>
#include <pcl/search/kdtree.h>
#include <Eigen/Eigenvalues>
#include <opencv2/core/core.hpp>
#include <QApplication>
#include <QDir>
#include "checkview.h"
quint32 Pipe::_FrameListKey = 1;
quint32 Pipe::_IdMapListKey = 2;
quint32 Pipe::_BGKey = 5;
quint32 Pipe::_BGScoreKey = 6;
quint32 Pipe::_SegListKey = 7;
quint32 Pipe::_ObjListKey = 8;
quint32 Pipe::_ObjScoreKey = 9;
quint32 Pipe::_DepthMapKey = 10;
quint32 Pipe::_CamKey = 100;
quint32 Pipe::_CamExKey = 101;

QHash<quint32,QByteArray> Pipe::_PipeData;

bool Pipe::_ErrorToStdErr=true;//to screen
bool Pipe::_ErrorToStdOut=true;//to log file
bool Pipe::_WarnToStdErr=true;//to screen
bool Pipe::_WarnToStdOut=false;//to log file
bool Pipe::_InformToStdErr=true;//to screen
bool Pipe::_InformToStdOut=false;//to log file

QDate Pipe::_Date;
QTime Pipe::_Time;
quint32 Pipe::_ImgLogUnitWidth = 300;
quint32 Pipe::_ImgLogUnitHeight= 300;
QVector<QVector<QImage>> Pipe::_ImgLog;
QString Pipe::_ImgLogFile;
int Pipe::_LastRow = -1;
int Pipe::_LastCol = 0;

Pipe::Pipe()
{
}

Pipe::~Pipe()
{
}

void Pipe::error(const std::string& msg)
{
    if(_ErrorToStdOut)
    {
        std::cout<<timestamp()<<msg<<std::endl;
        std::cout.flush();
    }
    if(_ErrorToStdErr)
    {
        std::cerr<<timestamp()<<msg<<std::endl;
    }
}

void Pipe::warn(const std::string& msg)
{
    if(_WarnToStdOut)
    {
        std::cout<<timestamp()<<msg<<std::endl;
        std::cout.flush();
    }
    if(_WarnToStdErr)
    {
        std::cerr<<timestamp()<<msg<<std::endl;
    }
}

void Pipe::inform(const std::string& msg)
{
    if(_InformToStdOut)
    {
        std::cout<<timestamp()<<msg<<std::endl;
    }
    if(_InformToStdErr)
    {
        std::cerr<<timestamp()<<msg<<std::endl;
    }
}

void Pipe::record(const std::string& msg)
{
     std::cout<<timestamp()<<msg<<std::endl;
}

std::string Pipe::timestamp()
{
    QDate currentDate;
    QTime currentTime;
    currentDate = _Date.currentDate();
    currentTime = _Time.currentTime();
    std::stringstream out("");
    if( 0!=_Date.daysTo(currentDate) )
    {
        std::cout<<currentDate.month()<<"/"<<currentDate.day()<<"/"<<currentDate.year()<<std::endl;
    }
    out.clear();
    out<<"["<<currentTime.hour()<<":"<<currentTime.minute()<<":"<<currentTime.second()<<"]";
    _Date = _Date.currentDate();
    _Time = _Time.currentTime();
    std::string result;
    out >> result;
    return result;
}

void Pipe::loadPipeDataFrom(const std::string& path)
{
    QFile inFile(QString::fromStdString(path));
    inFile.open(inFile.ReadOnly);
    QDataStream stream(&inFile);
    stream >> _PipeData;
    inFile.close();
}

void Pipe::dumpTo(const std::string& path)
{
    QFile outFile(QString::fromStdString(path));
    if( !outFile.open(outFile.WriteOnly) )
    {
        Pipe::error("Can't open file when dump to "+path);
        return;
    }
    QDataStream stream(&outFile);
    record("Dumping to "+path);
    stream << _PipeData;
    outFile.close();
}

void Pipe::dump(void)
{
    QString fileName;
    QDate date;
    QTime time;
    fileName = "./dump/";
    date = date.currentDate();
    fileName += date.toString("yy_MM_dd");
    fileName += time.currentTime().toString("_hh_mm");
    fileName += ".IA.dump";
    QFile outFile(fileName);
    if( !outFile.open(outFile.WriteOnly) )
    {
        Pipe::error("Can't open file when dump to "+fileName.toStdString());
        return ;
    }
    QDataStream stream(&outFile);
    record("Dumping to "+fileName.toStdString());
    stream << _PipeData;
    outFile.close();
}

quint32 Pipe::generateKey(void)
{
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::uniform_int_distribution<quint32> uniform_dist(1024,std::numeric_limits<quint32>::max());
    quint32 r = uniform_dist(e1);
    while(_PipeData.contains(r))
    {
        r = uniform_dist(e1);
    }
    return r;
}

quint32 Pipe::addToData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcd,QString custom)
{
    QByteArray buffer;
    QDataStream stream(&buffer,QIODevice::WriteOnly);
    stream << QString("PCD");
    stream << custom;
    stream << QString("XYZRGBNormal");
    stream << pcd->height;
    stream << pcd->width;
    stream << pcd->size();
    unsigned int idx;
    for(idx = 0; idx < pcd->size() ;idx++)
    {
        stream  <<pcd->at(idx).x
                <<pcd->at(idx).y
                <<pcd->at(idx).z
                <<pcd->at(idx).r
                <<pcd->at(idx).g
                <<pcd->at(idx).b
                <<pcd->at(idx).normal_x
                <<pcd->at(idx).normal_y
                <<pcd->at(idx).normal_z;
    }
    quint32 key = generateKey();
    _PipeData.insert(key,buffer);
    return key;
}

void Pipe::addToData(FullPointCloud::Ptr pcd,quint32 key,QString custom)
{
    QByteArray buffer;
    QDataStream stream(&buffer,QIODevice::WriteOnly);
    stream << QString("PCD");
    stream << custom;
    stream << QString("XYZRGBNormal");
    stream << pcd->height;
    stream << pcd->width;
    stream << pcd->size();
    unsigned int idx;
    for(idx = 0; idx < pcd->size() ;idx++)
    {
        stream  <<pcd->at(idx).x
                <<pcd->at(idx).y
                <<pcd->at(idx).z
                <<pcd->at(idx).r
                <<pcd->at(idx).g
                <<pcd->at(idx).b
                <<pcd->at(idx).normal_x
                <<pcd->at(idx).normal_y
                <<pcd->at(idx).normal_z;
    }
    if(_PipeData.contains(key))
    {
        warn("Pipe::addToData(FullPointCloud::Ptr pcd,quint32 key,QString custom)");
        QString msg;
        msg=msg.sprintf("Over-Writing:already have data at %d",key);
        warn(msg.toStdString());
    }
    _PipeData.insert(key,buffer);
}

bool Pipe::loadData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &pcd,QString& custom,quint32 key)
{
    QByteArray buffer = _PipeData.value(key);
    QDataStream stream(&buffer,QIODevice::ReadOnly);
    QString type;
    QString field;
    stream >> type;
    if(type!="PCD")
    {
        error("Pipe::loadData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &pcd,quint32 key)");
        error("Wrong data head, This may not be a PCD data");
        return false;
    }
    stream >> custom;
    if(!custom.isEmpty())
    {
        inform("Pipe::loadData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &pcd,quint32 key)");
        inform("custom info:"+custom.toStdString());
    }
    stream >> field;
    if(field!="XYZRGBNormal")
    {
        error("Pipe::loadData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &pcd,quint32 key)");
        error("Wrong data head, This may not be a XYZRGBNormal data");
        return false;
    }
    stream >> pcd->height;
    stream >> pcd->width;
    size_t size;
    stream >> size;
    QString msg;
    msg = msg.sprintf("w=%lu,h=%lu,s=%lu",pcd->width,pcd->height,size);
    inform(msg.toStdString());
    if(pcd->height*pcd->width!=size)
    {
        error("Pipe::loadData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &pcd,quint32 key)");
        error("The Size doesn't add up");
        return false;
    }
    unsigned int idx;
    pcd->resize(size);
    for(idx = 0; idx < pcd->size() ;idx++)
    {
        stream  >>pcd->at(idx).x
                >>pcd->at(idx).y
                >>pcd->at(idx).z
                >>pcd->at(idx).r
                >>pcd->at(idx).g
                >>pcd->at(idx).b
                >>pcd->at(idx).normal_x
                >>pcd->at(idx).normal_y
                >>pcd->at(idx).normal_z;
    }
    return true;
}

quint32 Pipe::addToData(const cv::Mat& m,QString custom)
{
    QByteArray buffer;
    QDataStream stream(&buffer,QIODevice::WriteOnly);
    stream << QString("cv::Mat");
    stream << custom;
    stream << m.rows;
    stream << m.cols;
    switch(m.depth())
    {
    case CV_8U:stream<<QString("8U");break;
    case CV_8S:stream<<QString("8S");break;
    case CV_16U:stream<<QString("16U");break;
    case CV_16S:stream<<QString("16S");break;
    case CV_32S:stream<<QString("32S");break;
    case CV_32F:stream<<QString("32F");break;
    case CV_64F:stream<<QString("64F");break;
    }
    stream << m.type();
    int sizeInByte = m.total()*m.elemSize();
    int k;
    for(k = 0;k<sizeInByte;++k)
    {
        stream <<m.data[k];
    }
    quint32 key = generateKey();
    _PipeData.insert(key,buffer);
    return key;
}

void Pipe::addToData(const cv::Mat& m ,quint32 key,QString custom)
{
    QByteArray buffer;
    QDataStream stream(&buffer,QIODevice::WriteOnly);
    stream << QString("cv::Mat");
    stream << custom;
    stream << m.rows;
    stream << m.cols;
    switch(m.depth())
    {
    case CV_8U:stream<<QString("8U");break;
    case CV_8S:stream<<QString("8S");break;
    case CV_16U:stream<<QString("16U");break;
    case CV_16S:stream<<QString("16S");break;
    case CV_32S:stream<<QString("32S");break;
    case CV_32F:stream<<QString("32F");break;
    case CV_64F:stream<<QString("64F");break;
    }
    stream << m.type();
    int sizeInByte = m.total()*m.elemSize();
    int k;
    for(k = 0;k<sizeInByte;++k)
    {
        stream <<m.data[k];
    }
    if(_PipeData.contains(key))
    {
        warn("Pipe::addToData(const cv::Mat& m ,quint32 key,QString custom)");
        QString msg;
        msg=msg.sprintf("Over-Writing:already have data at %d",key);
        warn(msg.toStdString());
    }
    _PipeData.insert(key,buffer);
}

bool Pipe::loadData(cv::Mat& m,QString& custom,quint32 key)
{
    QByteArray buffer = _PipeData.value(key);
    QDataStream stream(&buffer,QIODevice::ReadOnly);
    QString type;
    stream >> type;
    if(type!="cv::Mat")
    {
        error("Pipe::loadData(cv::Mat& m,QString& custom,quint32 key)");
        error("Wrong data head, This may not be a cv::Mat data");
        return false;
    };
    stream >> custom;
    if(!custom.isEmpty())
    {
        inform("Pipe::loadData(cv::Mat& m,QString& custom,quint32 key)");
        inform("custom info:"+custom.toStdString());
    }
    int r,c;
    stream >> r;
    stream >> c;
    QString depth;
    stream >> depth;
    if(!depth.isEmpty())
    {
        inform("Pipe::loadData(cv::Mat& m,QString& custom,quint32 key)");
        inform("depth info:"+depth.toStdString());
    }else{
        error("Pipe::loadData(cv::Mat& m,QString& custom,quint32 key)");
        error("Wrong depth This may not be a cv::Mat data");
        return false;
    }
    int typeofInt;
    stream >> typeofInt;
    m.create(r,c,typeofInt);
    int sizeInByte = m.total()*m.elemSize();
    int k;
    for(k = 0;k<sizeInByte;++k)
    {
        stream >>m.data[k];
    }
    return true;
}

void Pipe::addToData(const QList<quint32>& keyList,quint32 key,QString custom)
{
    QByteArray buffer;
    QDataStream stream(&buffer,QIODevice::WriteOnly);
    stream << QString("KeyList");
    stream << custom;
    stream << keyList;
    if(_PipeData.contains(key))
    {
        warn("Pipe::addToData(const QList<quint32>& keyList,quint32 key)");
        QString msg;
        msg=msg.sprintf("Over-Writing:already have data at %d",key);
        warn(msg.toStdString());
    }
    _PipeData.insert(key,buffer);
}

bool Pipe::loadData(QList<quint32>& keyList,QString& custom,quint32 key)
{
    if( _PipeData.count(key)>1 )
    {
        warn("Pipe::loadData(QList<quint32>& keyList,quint32 key)");
        QString msg;
        msg=msg.sprintf("Have more than one data at %d",key);
        warn(msg.toStdString());
    }
    QByteArray buffer = _PipeData.value(key);
    QDataStream stream(&buffer,QIODevice::ReadOnly);
    QString type;
    stream >> type;
    if(type!="KeyList")
    {
        error("Pipe::loadData(QList<quint32>& keyList,quint32 key)");
        error("Wrong data head, This may not be a KeyList data");
        return false;
    }
    stream >> custom;
    if(!custom.isEmpty())
    {
        inform("Pipe::loadData(QList<quint32>& keyList,quint32 key)");
        inform(custom.toStdString());
    }
    stream >> keyList;
    return true;
}

void Pipe::addToData(const QVector<QVector<quint32>>&keyList,quint32 key,QString custom)
{
    QByteArray buffer;
    QDataStream stream(&buffer,QIODevice::WriteOnly);
    stream << QString("GroupedKeyList");
    stream << custom;
    stream << keyList;
    if(_PipeData.contains(key))
    {
        warn("Pipe::addToData(const QVector<QVector<quint32>>& keyList,quint32 key)");
        QString msg;
        msg=msg.sprintf("Over-Writing:already have data at %d",key);
        warn(msg.toStdString());
    }
    _PipeData.insert(key,buffer);
}

bool Pipe::loadData(QVector<QVector<quint32>>&keyList,QString&custom,quint32 key)
{
    if( _PipeData.count(key)>1 )
    {
        warn("Pipe::loadData(QVector<QVector<quint32>>& keyList,quint32 key)");
        QString msg;
        msg=msg.sprintf("Have more than one data at %d",key);
        warn(msg.toStdString());
    }
    QByteArray buffer = _PipeData.value(key);
    QDataStream stream(&buffer,QIODevice::ReadOnly);
    QString type;
    stream >> type;
    if(type!="GroupedKeyList")
    {
        error("Pipe::loadData(QVector<QVector<quint32>>& keyList,quint32 key)");
        error("Wrong data head, This may not be a KeyList data");
        return false;
    }
    stream >> custom;
    if(!custom.isEmpty())
    {
        inform("Pipe::loadData(QVector<QVector<quint32>>& keyList,quint32 key)");
        inform(custom.toStdString());
    }
    stream >> keyList;
    return true;
}

QVector<QVariant> Pipe::parseDataHeadAt(quint32 row)
{
    QHash<quint32,QByteArray>::Iterator iter;
    QVector<QVariant> result;

    iter = _PipeData.begin();
    iter += (int)row;
    QByteArray buffer = iter.value();
    result.push_back(iter.key());
    QDataStream stream(&buffer,QIODevice::ReadOnly);
    QString type;
    stream >> type;
    if(type=="PCD")
    {
        result.push_back(type);
        QString custom;
        stream >> custom;
        result.push_back(custom);
        QString field;
        stream >> field;
        result.push_back(field);
        uint32_t w;
        stream >> w;
        result.push_back(w);
        uint32_t h;
        stream >> h;
        result.push_back(h);
        size_t s;
        stream >> s;
        result.push_back(s);
    }else
    if(type=="KeyList")
    {
        result.push_back(type);
        QString custom;
        stream >> custom;
        result.push_back(custom);
    }else
    if(type=="cv::Mat")
    {
        result.push_back(type);
        QString custom;
        stream >> custom;
        result.push_back(custom);
        int r,c;
        stream >> r;
        stream >> c;
        result.push_back(r);
        result.push_back(c);
        QString depth;
        stream >> depth;
        result.push_back(depth);
    }else
    if(type=="GroupedKeyList")
    {
        result.push_back(type);
        QString custom;
        stream >> custom;
        result.push_back(custom);
    }
    else{
        result.push_back(QString("Unknown"));
    }
    return result;
}

void Pipe::pcd2rgbd(quint32 width,quint32 height,const FullPointCloud::Ptr& pcd,const Eigen::Matrix3f& trans,cv::Mat& result)
{
    result.create(height,width,CV_32FC4);
    result.setTo(cv::Scalar::all(-1.0));
    unsigned int idx;
    for(idx=0;idx<pcd->size();++idx)
    {
        float r,g,b,d;
        unsigned int row,col;
        r = (float)pcd->at(idx).r / 255.0;
        g = (float)pcd->at(idx).g / 255.0;
        b = (float)pcd->at(idx).b / 255.0;
        d = pcd->at(idx).z;
        Eigen::Vector3f imgCoord;
        Eigen::Vector3f coord;
        coord(0) = pcd->at(idx).x;
        coord(1) = pcd->at(idx).y;
        coord(2) = pcd->at(idx).z;
        imgCoord = trans*coord;
        col = imgCoord(0)/imgCoord(2);
        row = imgCoord(1)/imgCoord(2);
        if(col>=0&&col<width&&row>=0&&row<height)
        {
            if(result.at<cv::Vec4f>(row,col)[0]==-1.0||d<result.at<cv::Vec4f>(row,col)[0])
            {
                result.at<cv::Vec4f>(row,col)[0] = b;
                result.at<cv::Vec4f>(row,col)[1] = g;
                result.at<cv::Vec4f>(row,col)[2] = r;
                result.at<cv::Vec4f>(row,col)[3] = d;
            }
        }
    }
}

void Pipe::getPCDCenter(const FullPointCloud::Ptr& pcd,FullPoint& p)
{
    unsigned int idx;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    float cnt=0.0;
    for(idx=0;idx<pcd->size();++idx)
    {
        if(_isnan(pcd->at(idx).z))continue;
        p.x += pcd->at(idx).x;
        p.y += pcd->at(idx).y;
        p.z += pcd->at(idx).z;
        cnt+=1.0;
    }
    p.x/=cnt;
    p.y/=cnt;
    p.z/=cnt;
}

void Pipe::getPCDBoxCenter(const FullPointCloud::Ptr&pcd,FullPoint& center)
{
    cv::Mat box;
    Pipe::get2DOBB(pcd,box);
    unsigned int idx = 0;
    center.x = ( box.at<float>(0,0) + box.at<float>(0,2) ) / 2.0;
    center.y = ( box.at<float>(1,0) + box.at<float>(1,2) ) / 2.0;
    float zmax = std::numeric_limits<float>::lowest();
    float zmin = std::numeric_limits<float>::max();
    for(idx=0;idx<pcd->size();++idx)
    {
        float z = pcd->at(idx).z;
        if(z<zmin)zmin = z;
        if(z>zmax)zmax = z;
    }
    center.z = ( zmin + zmax ) / 2.0;
    center.r = 255;
    center.g = 0;
    center.b = 0;

}

void Pipe::pcd2rgbd(quint32 width,quint32 height,const FullPointCloud::Ptr& pcd,const Eigen::Matrix3f& trans,cv::Mat& result,unsigned int&withinSight)
{
    withinSight = 0;
    result.create(height,width,CV_32FC4);
    result.setTo(cv::Scalar::all(-1.0));
    unsigned int idx;
    for(idx=0;idx<pcd->size();++idx)
    {
        float r,g,b,d;
        unsigned int row,col;
        r = (float)pcd->at(idx).r / 255.0;
        g = (float)pcd->at(idx).g / 255.0;
        b = (float)pcd->at(idx).b / 255.0;
        d = pcd->at(idx).z;
        Eigen::Vector3f imgCoord;
        Eigen::Vector3f coord;
        coord(0) = pcd->at(idx).x;
        coord(1) = pcd->at(idx).y;
        coord(2) = pcd->at(idx).z;
        imgCoord = trans*coord;
        col = imgCoord(0)/imgCoord(2);
        row = imgCoord(1)/imgCoord(2);
        if(col>=0&&col<width&&row>=0&&row<height)
        {
            ++withinSight;
            if(result.at<cv::Vec4f>(row,col)[0]==-1.0||d<result.at<cv::Vec4f>(row,col)[0])
            {
                result.at<cv::Vec4f>(row,col)[0] = b;
                result.at<cv::Vec4f>(row,col)[1] = g;
                result.at<cv::Vec4f>(row,col)[2] = r;
                result.at<cv::Vec4f>(row,col)[3] = d;
            }
        }
    }
}

void Pipe::rgbd2pcd(cv::Mat& rgbd,const Eigen::Matrix3f& intrinsic,FullPointCloud::Ptr& pcd)
{
    unsigned int r,c;
    Eigen::Matrix3f trans = intrinsic.inverse();
    for(r=0;r<rgbd.rows;++r)
    {
        for(c=0;c<rgbd.cols;++c)
        {
            if(rgbd.at<cv::Vec4f>(r,c)[3]!=-1.0)
            {
                FullPoint p;
                p.b = 255.0*rgbd.at<cv::Vec4f>(r,c)[0];
                p.g = 255.0*rgbd.at<cv::Vec4f>(r,c)[1];
                p.r = 255.0*rgbd.at<cv::Vec4f>(r,c)[2];
                Eigen::Vector3f coord;
                Eigen::Vector3f imgCoord;
                imgCoord(0) = c;
                imgCoord(1) = r;
                imgCoord(2) = 1;
                coord = trans*imgCoord;
                p.x = coord(0);
                p.y = coord(1);
                p.z = rgbd.at<cv::Vec4f>(r,c)[3];
                double w = p.z / coord(2);
                p.x*=w;
                p.y*=w;
                pcd->push_back(p);
            }
        }
    }
    pcd->height = 1;
    pcd->width = pcd->size();
}

void Pipe::IdMap2Img(const FullPointCloud::Ptr& cloud,const cv::Mat& idmap,std::string filename)
{
    QImage output;

    IdMap2Img(cloud,idmap,output);

    output.save(QString::fromStdString(filename));
}

void Pipe::IdMap2Img(const FullPointCloud::Ptr& cloud,const cv::Mat& idmap,QImage&output)
{
    if( cloud->width!=idmap.cols || cloud->height!=idmap.rows )
    {
        error("Pipe::IdMap2Img(const FullPointCloud::Ptr& cloud,const cv::Mat& idmap,QImage&output)");
        error("The Size of Cloud and Idmap doesn't match");
        return;
    }
    output = pcdToImg(cloud);
    //rgba to gray
    unsigned int x,y;
    for(y=0;y<output.height();++y)
    for(x=0;x<output.width();++x)
    {
        QRgb pix = output.pixel(x,y);
        QRgb gray = qGray(pix);
        output.setPixel( x, y, qRgb(gray,gray,gray) );
    }
    //add transparent color mask to patches
    for(y=0;y<output.height();++y)
    for(x=0;x<output.width();++x)
    {
        int lbl = idmap.at<__int32>(y,x);
        if( 0 != lbl )
        {
            QRgb pix = output.pixel(x,y);
            unsigned int gray = qRed(pix);
            unsigned int idx = lbl % FalseColorNum ;
            unsigned int r = FalseColor[idx][0];
            unsigned int g = FalseColor[idx][1];
            unsigned int b = FalseColor[idx][2];
            pix = qRgb((r*3+gray*2)/5,(g*3+gray*2)/5,(b*3+gray*2)/5);
            output.setPixel(x,y,pix);
        }
    }
}

void Pipe::IdMap2Img(const FullPointCloud::Ptr& bg,const FullPointCloud::Ptr& fg,const cv::Mat& fgIdmap,const std::vector<cv::Mat>& camList,const Eigen::Matrix3f& camintri,QImage&output)
{
    QList<QImage> outputList;
    unsigned int idx;
    FullPointCloud::Ptr seg(new FullPointCloud);
    FullPointCloud::Ptr segT(new FullPointCloud);
    FullPointCloud::Ptr segfg(new FullPointCloud);
    FullPointCloud::Ptr segbg(new FullPointCloud);
    *segfg = *fg;
    for(idx=0;idx<segfg->size();++idx)
    {
        FullPoint &p = segfg->at(idx);
        int r = p.r;
        int g = p.g;
        int b = p.b;
        int gray = qGray(r,g,b);
        int cIdx = fgIdmap.at<__int32>(0,idx);
        cIdx = cIdx%FalseColorNum;
        r = FalseColor[cIdx][0];
        g = FalseColor[cIdx][1];
        b = FalseColor[cIdx][2];
        p.r = (r*3+gray*2)/5;
        p.g = (g*3+gray*2)/5;
        p.b = (b*3+gray*2)/5;
    }
    *segbg = *bg;
    for(idx=0;idx<segbg->size();++idx)
    {
        FullPoint &p = segbg->at(idx);
        int r = p.r;
        int g = p.g;
        int b = p.b;
        int gray = qGray(r,g,b);
        p.r = gray;
        p.g = gray;
        p.b = gray;
    }
    *seg = *segfg + *segbg;
    for(idx=0;idx<camList.size();++idx)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        getT(camList[idx],T);
        pcl::transformPointCloudWithNormals<FullPoint>(*seg,*segT,T);
        outputList.push_back(pcdToImg(640,640,segT,camintri));
    }
    output = hScrollPainting(outputList);
}

void Pipe::IdMap2Img(const FullPointCloud::Ptr& bg,const FullPointCloud::Ptr& fg,const cv::Mat& fgIdMap,const std::vector<cv::Mat>& camList,const Eigen::Matrix3f& intrin,std::string filename)
{
    QImage output;
    IdMap2Img(bg,fg,fgIdMap,camList,intrin,output);
    output.save(QString::fromStdString(filename));
}

void Pipe::IdMap2Img(const FullPointCloud::Ptr& pcd,const std::vector<cv::Mat>& camList,const Eigen::Matrix3f& intrin,std::string filename)
{
    QList<QImage> outputList;
    QImage output;
    unsigned int idx;
    FullPointCloud::Ptr segT(new FullPointCloud);
    for(idx=0;idx<camList.size();++idx)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        getT(camList[idx],T);
        pcl::transformPointCloudWithNormals<FullPoint>(*pcd,*segT,T);
        outputList.push_back(pcdToImg(640,640,segT,intrin));
    }
    output = hScrollPainting(outputList);
    output.save(QString::fromStdString(filename));
}

QImage Pipe::pcdToImg(const FullPointCloud::Ptr& pcd)
{
    QImage output(pcd->width,pcd->height,QImage::Format_RGB888);
    output.fill(qRgba(0,0,0,0));
    unsigned int x,y;
    for(y=0;y<pcd->height;++y)
    for(x=0;x<pcd->width;++x)
    {
        quint32 r,g,b,d;
        r = pcd->at(x,y).r;
        g = pcd->at(x,y).g;
        b = pcd->at(x,y).b;
        d = ( 1.0 - pcd->at(x,y).z / 3.0 )* 255.0 ;
        output.setPixel(x,y,qRgb(r,g,b));
    }
    return output;
}

QImage Pipe::pcdToImg(quint32 w,quint32 h,const FullPointCloud::Ptr& pcd,const Eigen::Matrix3f& trans)
{
    QImage output(w,h,QImage::Format_ARGB32);
    output.fill(qRgba(0,0,0,0));
    cv::Mat depth;
    depth.create(h,w,CV_32FC1);
    depth.setTo(-1.0);
    unsigned int idx;
    for(idx=0;idx<pcd->size();++idx)
    {
        quint32 r,g,b;
        float d;
        unsigned int row,col;
        r = pcd->at(idx).r;
        g = pcd->at(idx).g;
        b = pcd->at(idx).b;
        d = pcd->at(idx).z;
        Eigen::Vector3f imgCoord;
        Eigen::Vector3f coord;
        coord(0) = pcd->at(idx).x;
        coord(1) = pcd->at(idx).y;
        coord(2) = pcd->at(idx).z;
        imgCoord = trans*coord;
        col = imgCoord(0)/imgCoord(2);
        row = imgCoord(1)/imgCoord(2);
        if(col>=0&&col<w&&row>=0&&row<h)
        {
            if( depth.at<float>(row,col)==-1.0 || d < depth.at<float>(row,col) )
            {
                output.setPixel(col,row,qRgba(r,g,b,255));
                depth.at<float>(row,col) = d;
            }
        }
    }
    return output;
}

QImage Pipe::drawTextForLogUnit(const QImage& img)
{
    QImage output(_ImgLogUnitWidth,_ImgLogUnitHeight,QImage::Format_ARGB32);
    output.fill(qRgba(255,255,255,0));
    QImage scaledImg;
    if( (float)img.height() / (float)_ImgLogUnitHeight  > (float)img.width() / (float)_ImgLogUnitWidth )
    {
        scaledImg = img.scaledToHeight(_ImgLogUnitHeight);
    }else{
        scaledImg = img.scaledToWidth(_ImgLogUnitWidth);
    }
    QStringList text = img.textKeys();
    QPainter p;
    p.begin(&output);
    p.drawImage(_ImgLogUnitWidth - scaledImg.width(),_ImgLogUnitHeight - scaledImg.height(),scaledImg);
    float y = 12.0;
    float x = 5.0;
    foreach(QString txtKey,text)
    {
        QString msg;
        msg = txtKey+img.text(txtKey);
        QTextStream stream(&msg,QIODevice::ReadOnly);
        QString oneline;
        while( !stream.atEnd() )
        {
            oneline = stream.readLine(_ImgLogUnitWidth / 8 );
            p.drawText(QPointF(x,y),oneline);
            y += 12.0;
        }
    }
    p.end();
    return output;
}

void Pipe::startImgLog(void)
{
    _ImgLog.clear();
    _ImgLogFile.clear();
    Pipe::_LastRow = -1;
    Pipe::_LastCol = 0;
    QTextStream stream(&_ImgLogFile,QIODevice::WriteOnly);
    stream<<_Time.currentTime().hour()<<"_";
    stream<<_Time.currentTime().minute()<<"_";
    stream<<_Time.currentTime().second()<<"_";
    stream<<_Time.currentTime().msec();
}

void Pipe::addImgToLog(quint32 row,quint32 col,QImage& img)
{
    if( row >= _ImgLog.size() ) _ImgLog.resize( row+1 );
    if( col >= _ImgLog[row].size() ) _ImgLog[row].resize( col+1 );
    _ImgLog[row][col] = img.copy(0,0,img.width(),img.height());
    _LastRow = row;
    _LastCol = col;
}

void Pipe::endImgLog(void)
{
    quint32 height;
    quint32 width = 0;
    quint32 xn,yn;
    for(yn=0;yn<_ImgLog.size();++yn)
    {
        if(width<_ImgLog[yn].size()*_ImgLogUnitWidth)
        {
            width = _ImgLog[yn].size()*_ImgLogUnitWidth;
        }
    }
    QImage output(width,height,QImage::Format_ARGB32_Premultiplied);
    output.fill(qRgba(255,255,255,0));
    unsigned int devider = 2;
    quint32 subUnitNum = _ImgLog.size();
    while(NULL==output.paintEngine())
    {
        subUnitNum = ( (_ImgLog.size() + 1 )/ devider );
        height = subUnitNum * _ImgLogUnitHeight;
        output = QImage(width,height,QImage::Format_ARGB32_Premultiplied);
        output.fill(qRgba(255,255,255,0));
        devider *= 2;
    }
    QImage ImgUnit;
    QPainter p;
    quint32 subOutputNum = 0;
    quint32 hn = 0;
    p.begin(&output);
    for(yn=0;yn<_ImgLog.size();++yn)
    {
        for(xn=0;xn<_ImgLog[yn].size();++xn)
        {
            if(_ImgLog[yn][xn].isNull())continue;
            ImgUnit = drawTextForLogUnit(_ImgLog[yn][xn]);            
            p.drawImage(xn*_ImgLogUnitWidth,hn*_ImgLogUnitHeight,ImgUnit);
        }
        ++hn;
        if( hn == subUnitNum )
        {
            QString subname;
            subname = subname.sprintf("%u",subOutputNum);
            output.save(_ImgLogFile+".Log.Sub"+subname+".png");
            output.fill(qRgba(255,255,255,0));
            hn = 0;
            subOutputNum++;
        }
    }
    p.end();
    QString subname;
    subname = subname.sprintf("%u",subOutputNum);
    output.save(_ImgLogFile+".Log.Sub"+subname+".png");
}

double Pipe::match3D(  const FullPointCloud::Ptr&input,
                       const Eigen::Matrix3f& intrinsic,
                       const FullPointCloud::Ptr&reference,
                       std::vector<double>&scoreList,
                       double Occlusion_Threshold)
{
    cv::Mat rgbd;
    unsigned int withinSight = 0;
    pcd2rgbd(reference->width,reference->height,input,intrinsic,rgbd,withinSight);
    unsigned int r,c;
    double occlusion = 0.0;
    double maxOcclusion = 0.0;
    double overlap = 0.0;

    for(r=0;r<rgbd.rows;++r)
        for(c=0;c<rgbd.cols;++c)
        {
            if( rgbd.at<cv::Vec4f>(r,c)[0] != -1 && rgbd.at<cv::Vec4f>(r,c)[3] != -1 )
            {
                float z = rgbd.at<cv::Vec4f>(r,c)[3];
                float d = reference->at(c,r).z - z;
                if( d > Occlusion_Threshold )
                {
                    occlusion += 1.0;
                }
                maxOcclusion += 1.0;
            }

        }

    pcl::search::KdTree<FullPoint> tree;
    tree.setInputCloud(reference);
    std::vector<int> indices;
    std::vector<float> dist;

    for(unsigned int i=0;i<input->size();++i)
    {
        indices.clear();
        dist.clear();
        tree.nearestKSearchT(input->at(i),1,indices,dist);
        overlap += RGB_Energy(
                    input->at(i).r,
                    input->at(i).g,
                    input->at(i).b,
                    reference->at(indices[0]).r,
                    reference->at(indices[0]).g,
                    reference->at(indices[0]).b
                    )*std::expf( - 100*dist[0]*dist[0] );
    }

    scoreList.push_back(occlusion);
    scoreList.push_back(overlap);
    scoreList.push_back(withinSight);

    return  occlusion / maxOcclusion - overlap / double(input->size());
}

double Pipe::match(
        const FullPointCloud::Ptr&input,
        const Eigen::Matrix3f& intrinsic,
        const FullPointCloud::Ptr&reference,
        double Occlusion_Threshold
        )
{
    cv::Mat rgbd;
    unsigned int withinSight = 0;
    pcd2rgbd(reference->width,reference->height,input,intrinsic,rgbd,withinSight);
    unsigned int r,c;
    double occlusion = 1.0;
    double overlap = 1.0;
    for(r=0;r<rgbd.rows;++r)
        for(c=0;c<rgbd.cols;++c)
        {
            if( rgbd.at<cv::Vec4f>(r,c)[0] != -1 && rgbd.at<cv::Vec4f>(r,c)[3] != -1 )
            {
                float z = rgbd.at<cv::Vec4f>(r,c)[3];
                float d = reference->at(c,r).z - z;
                if( d > Occlusion_Threshold )
                {
//                    double od = std::expf( 0.1*d*d );
//                    occlusion +=  od<2.0?od:2.0;
                    occlusion += std::expf( 0.1*d*d );
                }else{
                    overlap += RGB_Energy(
                                uchar(rgbd.at<cv::Vec4f>(r,c)[0]*255.0),
                                uchar(rgbd.at<cv::Vec4f>(r,c)[1]*255.0),
                                uchar(rgbd.at<cv::Vec4f>(r,c)[2]*255.0),
                                reference->at(c,r).r,
                                reference->at(c,r).g,
                                reference->at(c,r).b
                                )*std::expf( - 100*d*d );
                }
            }

        }

//    return 2*occlusion - overlap - withinSight;
    return occlusion - overlap;
}

double Pipe::match(const FullPointCloud::Ptr&input,
                    const Eigen::Matrix3f& intrinsic,
                    const FullPointCloud::Ptr& reference,
                    std::vector<double>& scoreList,
                    double Occlusion_Threshold
                    )
{
    cv::Mat rgbd;
    unsigned int withinSight = 0;
    pcd2rgbd(reference->width,reference->height,input,intrinsic,rgbd,withinSight);
    unsigned int r,c;
    double occlusion = 0.0;
    double overlap = 0.0;
    for(r=0;r<rgbd.rows;++r)
        for(c=0;c<rgbd.cols;++c)
        {
            if( rgbd.at<cv::Vec4f>(r,c)[0] != -1 && rgbd.at<cv::Vec4f>(r,c)[3] != -1 )
            {
                float z = rgbd.at<cv::Vec4f>(r,c)[3];
                float d = reference->at(c,r).z - z;
                if( d > Occlusion_Threshold )
                {
                    occlusion += 1.0;
                }else{
                    overlap += RGB_Energy(
                                uchar(rgbd.at<cv::Vec4f>(r,c)[0]*255.0),
                                uchar(rgbd.at<cv::Vec4f>(r,c)[1]*255.0),
                                uchar(rgbd.at<cv::Vec4f>(r,c)[2]*255.0),
                                reference->at(c,r).r,
                                reference->at(c,r).g,
                                reference->at(c,r).b
                                )*std::expf( - 100*d*d );
                }
            }

        }

    scoreList.push_back(occlusion);
    scoreList.push_back(overlap);
    scoreList.push_back(withinSight);

    return  occlusion - overlap;
}

double Pipe::RGB_Energy(uchar r0,uchar g0,uchar b0,uchar r1, uchar g1, uchar b1)
{
    double r_d,g_d,b_d;
    r_d = ( double(r0) - double(r1) );
    g_d = ( double(g0) - double(g1) );
    b_d = ( double(b0) - double(b1) );
    return std::exp(-r_d*r_d/14450.0)*std::exp(-g_d*g_d/14450.0)*std::exp(-b_d*b_d/14450.0);
}

double Pipe::RGB_Dist(uchar r0,uchar g0,uchar b0,uchar r1, uchar g1, uchar b1)
{
    double r_d,g_d,b_d;
    r_d = ( double(r0) - double(r1) );
    g_d = ( double(g0) - double(g1) );
    b_d = ( double(b0) - double(b1) );
    return ( r_d*r_d + g_d*g_d + b_d*b_d ) / 195075;
}

QImage Pipe::hScrollPainting(QList<QImage>& list)
{
    QImage output;
    unsigned int unitW = list[0].width();
    unsigned int unitH = list[0].height();
    output = QImage(unitW*list.size(),unitH,QImage::Format_ARGB32);
    unsigned int x,y;
    unsigned int i,unitX;
    for( y = 0 ; y < output.height() ; ++y )
        for( x = 0 ; x < output.width() ; ++x )
        {
            i = x / unitW;
            unitX = x - i*unitW;
            QRgb pix = list[i].pixel(unitX,y); // unitY == y
            output.setPixel(x,y,pix);
        }
    return output;
}

void Pipe::get2DOBB(const FullPointCloud::Ptr &pcd,cv::Mat& result)
{
    Eigen::MatrixXf points(2,pcd->size());
    unsigned int idx;
    for(idx=0;idx<points.cols();++idx)
    {
        points(0,idx) = pcd->at(idx).x;
        points(1,idx) = pcd->at(idx).y;
    }
    float meanX,meanY;
    meanX = points.row(0).mean();
    meanY = points.row(1).mean();
//    inform("meanX,meanY");
//    std::cerr<<meanX<<std::endl;
//    std::cerr<<meanY<<std::endl;
    for(idx=0;idx<points.cols();++idx)
    {
        points(0,idx) -= meanX;
        points(1,idx) -= meanY;
    }
    Eigen::MatrixXf box(2,4);
    result.create(box.rows(),box.cols(),CV_32FC1);
    float theta;
    float minArea = std::numeric_limits<float>::max();
    float currentArea = 0.0;
    for(theta=0; theta<2*M_PI ;theta+=(M_PI/180))
    {
        float c = std::cosf(theta);
        float s = std::sinf(theta);
        Eigen::Matrix2f rot;
        Eigen::Matrix2f invRot;
        rot<<c,-s,s,c;
        invRot<<c,s,-s,c;
        Eigen::MatrixXf rotP = rot*points;

        float maxX,minX,maxY,minY;

        maxX = rotP.row(0).maxCoeff();
        minX = rotP.row(0).minCoeff();
        maxY = rotP.row(1).maxCoeff();
        minY = rotP.row(1).minCoeff();

        currentArea = (maxX-minX)*(maxY-minY);

        if(minArea>currentArea)
        {

            box(0,0) = minX;box(0,1)=maxX;box(0,2)=maxX;box(0,3)=minX;
            box(1,0) = minY;box(1,1)=minY;box(1,2)=maxY;box(1,3)=maxY;

            box = invRot*box;

            for(idx=0;idx<box.cols();++idx)
            {
                box(0,idx) += meanX;
                box(1,idx) += meanY;
            }
            getMat(box,result);
            minArea = currentArea;
        }
    }
//    debug2DOBB(pcd,result);
}

void Pipe::get2DOBBVar(const FullPointCloud::Ptr&pcd,cv::Mat&result)
{
    Eigen::MatrixXf points(2,pcd->size());
    unsigned int idx;
    for(idx=0;idx<points.cols();++idx)
    {
        points(0,idx) = pcd->at(idx).x;
        points(1,idx) = pcd->at(idx).y;
    }
    float meanX,meanY;
    meanX = points.row(0).mean();
    meanY = points.row(1).mean();
//    inform("meanX,meanY");
//    std::cerr<<meanX<<std::endl;
//    std::cerr<<meanY<<std::endl;
    for(idx=0;idx<points.cols();++idx)
    {
        points(0,idx) -= meanX;
        points(1,idx) -= meanY;
    }
    Eigen::MatrixXf box(2,4);
    result.create(box.rows(),box.cols(),CV_32FC1);
    float theta;
    float minArea = std::numeric_limits<float>::max();
    float currentArea = 0.0;
    for(theta=0; theta<2*M_PI ;theta+=(M_PI/180))
    {
        float c = std::cosf(theta);
        float s = std::sinf(theta);
        Eigen::Matrix2f rot;
        Eigen::Matrix2f invRot;
        rot<<c,-s,s,c;
        invRot<<c,s,-s,c;
        Eigen::MatrixXf rotP = rot*points;

        float maxX,minX,maxY,minY;

        Eigen::MatrixXf var = rotP*(rotP.transpose());
        Eigen::MatrixXf sqrt = var.cwiseSqrt();

        float mx,my;
        mx = 2*sqrt(0,0);
        my = 2*sqrt(1,1);

        maxX = mx;
        minX = -mx;
        maxY = my;
        minY = -my;

        currentArea = mx*my;

        if(minArea>currentArea)
        {

            box(0,0) = minX;box(0,1)=maxX;box(0,2)=maxX;box(0,3)=minX;
            box(1,0) = minY;box(1,1)=minY;box(1,2)=maxY;box(1,3)=maxY;

            box = invRot*box;

            for(idx=0;idx<box.cols();++idx)
            {
                box(0,idx) += meanX;
                box(1,idx) += meanY;
            }
            getMat(box,result);
            minArea = currentArea;
        }
    }
}

void Pipe::getBoundingBox( const FullPointCloud::Ptr& obj, cv::Mat& vertexes )
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

void Pipe::getT(const cv::Mat &mat,Eigen::MatrixXf& T)
{
    T = Eigen::MatrixXf(mat.rows,mat.cols);
    unsigned int r,c;
    for(r=0;r<mat.rows;++r)
        for(c=0;c<mat.cols;++c)
            T(r,c)=mat.at<float>(r,c);
}

void Pipe::transformOBB(const cv::Mat& invTransMat,cv::Mat& boxVertex)
{
    Eigen::Matrix4f _T;
    Eigen::Matrix4f invT;
    Eigen::MatrixXf vertex;
    Eigen::MatrixXf result;

    getT(invTransMat,_T);

    std::cerr<<_T<<std::endl;
    getT(boxVertex,vertex);

    invT = _T.inverse();

    Eigen::Matrix3f R = invT.block(0,0,3,3);
    Eigen::Vector3f T = invT.block(0,3,3,1);

    result = (R*vertex).colwise() + T ;

    getMat(result,boxVertex);
}

void Pipe::debug2DOBB(const FullPointCloud::Ptr& pcd,const cv::Mat& m)
{
    QImage output(1024,1024,QImage::Format_ARGB32);
    unsigned int idx;

    for(idx=0;idx<pcd->size();++idx)
    {
        int x,y;
        int r,g,b;
        x = pcd->at(idx).x*50+512;
        y = pcd->at(idx).y*50+512;
        r = pcd->at(idx).r;
        g = pcd->at(idx).g;
        b = pcd->at(idx).b;
        output.setPixel(x,y,qRgb(r,g,b));
    }

    for(idx=0;idx<m.cols;++idx)
    {
        int x,y;
        int r,g,b;
        x = m.at<float>(0,idx)*50+512;
        y = m.at<float>(1,idx)*50+512;
        r = 0;
        g = 0;
        b = 255;
        output.setPixel(x,y,qRgb(r,g,b));
    }

    QString filename;
    filename = ".\\log\\"+filename.sprintf("%d_%d",_Time.msec(),rand())+".png";
    output.save(filename);
}

void Pipe::debug2DOBB(const std::vector<FullPointCloud::Ptr>& pcd,std::vector<cv::Mat>&layouts,std::vector<unsigned int>&obj_id)
{
    QImage output(2000,1500,QImage::Format_ARGB32);
    FullPointCloud::Ptr bg(new FullPointCloud);
    QString info;
    unsigned int idx;
    if(Pipe::loadData(bg,info,Pipe::_BGKey))
    {
        for(idx=0;idx<bg->size();++idx)
        {
            int x,y;
            int r,g,b;
            x = bg->at(idx).x*50+1000;
            y = bg->at(idx).y*50+750;
            r = bg->at(idx).r;
            g = bg->at(idx).g;
            b = bg->at(idx).b;
            output.setPixel(x,y,qRgb(r,g,b));
        }
    }
    unsigned int p;
    for(p=0;p<pcd.size();++p)
    for(idx=0;idx<pcd[p]->size();++idx)
    {
        int x,y;
        int r,g,b;
        x = pcd[p]->at(idx).x*50+1000;
        y = pcd[p]->at(idx).y*50+750;
        r = pcd[p]->at(idx).r;
        g = pcd[p]->at(idx).g;
        b = pcd[p]->at(idx).b;
        output.setPixel(x,y,qRgb(r,g,b));
    }

    QPainter painter;
    painter.begin(&output);
    for(idx=0;idx<layouts.size();++idx)
    {
        cv::Mat box;
        layouts[idx].copyTo(box);
        int r,g,b;
        r = Pipe::FalseColor[obj_id[idx]][0];
        g = Pipe::FalseColor[obj_id[idx]][1];
        b = Pipe::FalseColor[obj_id[idx]][2];
        painter.setPen(QColor(qRgb(r,g,b)));
        float x0 = box.at<float>(0,0)*50+1000;
        float y0 = box.at<float>(1,0)*50+750;
        float x1 = box.at<float>(0,1)*50+1000;
        float y1 = box.at<float>(1,1)*50+750;
        float x2 = box.at<float>(0,2)*50+1000;
        float y2 = box.at<float>(1,2)*50+750;
        float x3 = box.at<float>(0,3)*50+1000;
        float y3 = box.at<float>(1,3)*50+750;
        painter.drawLine(x0,y0,x1,y1);
        painter.drawLine(x1,y1,x2,y2);
        painter.drawLine(x2,y2,x3,y3);
        painter.drawLine(x3,y3,x0,y0);
    }
    painter.end();
    QString filename;
    filename = ".\\log\\"+filename.sprintf("%d_%d",_Time.msec(),rand())+".png";
    output.save(filename);
}

void Pipe::debug2DOBB(const std::vector<FullPointCloud::Ptr>& pcd,std::vector<cv::Mat>&layouts,std::vector<unsigned int>&obj_id,QImage& output)
{
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    unsigned int idx;
    unsigned int p;
    for(p=0;p<pcd.size();++p)
    for(idx=0;idx<pcd[p]->size();++idx)
    {
        int x,y;
        x = pcd[p]->at(idx).x*50;
        y = pcd[p]->at(idx).y*50;
        if(minX > x) minX = x;
        if(maxX < x) maxX = x;
        if(minY > y) minY = y;
        if(maxY < y) maxY = y;
    }
    output = QImage(int(maxX-minX+2),int(maxY-minY+2),QImage::Format_ARGB32);
    for(p=0;p<pcd.size();++p)
    for(idx=0;idx<pcd[p]->size();++idx)
    {
        int x,y;
        int r,g,b;
        x = pcd[p]->at(idx).x*50-minX+1;
        y = pcd[p]->at(idx).y*50-minY+1;
        r = pcd[p]->at(idx).r;
        g = pcd[p]->at(idx).g;
        b = pcd[p]->at(idx).b;
        output.setPixel(x,y,qRgb(r,g,b));
    }
    QPainter painter;
    painter.begin(&output);
    for(idx=0;idx<layouts.size();++idx)
    {
        cv::Mat box;
        layouts[idx].copyTo(box);
        int r,g,b;
        r = Pipe::FalseColor[obj_id[idx]][0];
        g = Pipe::FalseColor[obj_id[idx]][1];
        b = Pipe::FalseColor[obj_id[idx]][2];
        painter.setPen(QColor(qRgb(r,g,b)));
        float x0 = box.at<float>(0,0)*50-minX+1;
        float y0 = box.at<float>(1,0)*50-minY+1;
        float x1 = box.at<float>(0,1)*50-minX+1;
        float y1 = box.at<float>(1,1)*50-minY+1;
        float x2 = box.at<float>(0,2)*50-minX+1;
        float y2 = box.at<float>(1,2)*50-minY+1;
        float x3 = box.at<float>(0,3)*50-minX+1;
        float y3 = box.at<float>(1,3)*50-minY+1;
        painter.drawLine(x0,y0,x1,y1);
        painter.drawLine(x1,y1,x2,y2);
        painter.drawLine(x2,y2,x3,y3);
        painter.drawLine(x3,y3,x0,y0);
    }
    painter.end();
}

void Pipe::debug2DRelation(const Eigen::MatrixXf& centers,cv::Mat& relation,QImage&output)
{
    unsigned int ir,ic;
    QPainter painter;
    painter.begin(&output);
    unsigned int cnt = 0;
    for(ir=0;ir<relation.rows;++ir)
    for(ic=0;ic<relation.cols;++ic)
    {
        if(ir!=ic&&relation.at<float>(ir,ic)>0.2)
        {
            int r,g,b;
            r = Pipe::FalseColor[cnt][0];
            g = Pipe::FalseColor[cnt][1];
            b = Pipe::FalseColor[cnt][2];
            painter.setPen(QColor(qRgb(r,g,b)));
            float x0 = centers(0,ir)*50+10;
            float y0 = centers(1,ir)*50+520;
            float x1 = centers(0,ic)*50+10;
            float y1 = centers(1,ic)*50+520;
            painter.drawLine(x0,y0,x1,y1);
            ++cnt;
        }

    }
    painter.end();
}

void Pipe::doICP(const FullPointCloud::Ptr &s, const FullPointCloud::Ptr &t, Eigen::Matrix4f& transform, bool ifDownSample)
{
    FullPointCloud::Ptr source;
    FullPointCloud::Ptr target;
    if(ifDownSample)
    {
        ICP::DownSampler downS,downT;
        downS(s);
        source = downS.downSampled;
        downT(t);
        target = downT.downSampled;
    }else{
        source = s;
        target = t;
    }
    inform("initICP");
    ICP::ICPInit<pcl::PFHRGBSignature250,ICP::FeatureExtractorPFHRGB> initICP;
    initICP(source,target);
    std::cerr<<initICP._T<<std::endl;
    ICP::ICPFiner finerICP;
    finerICP._T = initICP._T;
    inform("finerICP");
    finerICP(source,target);
    std::cerr<<finerICP._T<<std::endl;
    transform = finerICP._T;
}



QImage Pipe::overlapImg(QImage& bg,QImage&fg)
{
    QImage output(bg.width(),bg.height(),QImage::Format_ARGB32);
    unsigned int x,y;
    for(y=0;y<output.height();++y)
    for(x=0;x<output.width();++x)
    {
        QRgb pix = bg.pixel(x,y);
        unsigned int gray = qGray(pix);
        if( 0 != qAlpha(fg.pixel(x,y)) )
        {

            QRgb fpix = fg.pixel(x,y);
            unsigned int r = qRed(fpix);
            unsigned int g = qGreen(fpix);
            unsigned int b = qBlue(fpix);
            fpix = qRgb((r*3+gray*2)/5,(g*3+gray*2)/5,(b*3+gray*2)/5);
            output.setPixel(x,y,fpix);
        }else{
            QRgb fpix = qRgb(gray,gray,gray);
            output.setPixel(x,y,fpix);
        }
    }
    return output;
}

void Pipe::getT(const cv::Mat& mat,Eigen::Matrix4f& T)
{
    unsigned int r,c;
    for(r=0;r<mat.rows;++r)
        for(c=0;c<mat.cols;++c)
            T(r,c)=mat.at<float>(r,c);
}

void Pipe::getMat(const Eigen::Matrix4f& T,cv::Mat& mat)
{
    unsigned int r,c;
    mat.create(4,4,CV_32FC1);
    for(r=0;r<T.rows();++r)
        for(c=0;c<T.cols();++c)
            mat.at<float>(r,c) = T(r,c);
}

void Pipe::getMat(const Eigen::MatrixXf& T,cv::Mat& mat)
{
    unsigned int r,c;
    mat.create(T.rows(),T.cols(),CV_32FC1);
    for(r=0;r<T.rows();++r)
        for(c=0;c<T.cols();++c)
            mat.at<float>(r,c) = T(r,c);
}

void Pipe::debug(const FullPointCloud::Ptr& a,const FullPointCloud::Ptr& b,const Eigen::Matrix4f& T,const std::string& path)
{
    FullPointCloud::Ptr pcd(new FullPointCloud);
    FullPointCloud::Ptr pcd2(new FullPointCloud);
    pcl::transformPointCloudWithNormals<FullPoint>(*a,*pcd,T);
    unsigned int idx;
    for(idx=0;idx<pcd->size();++idx)
    {
        FullPoint &p = pcd->at(idx);
        p.r=0;
        p.g=255;
        p.b=0;
    }
    *pcd2=*b;
    for(idx=0;idx<pcd2->size();++idx)
    {
        FullPoint &p = pcd2->at(idx);
        p.r=255;
        p.g=0;
        p.b=0;
    }
    *pcd+=*pcd2;
    pcl::io::savePCDFileBinary(path,*pcd);
}

void Pipe::debug(const std::vector<FullPointCloud::Ptr>& pcdlist,const std::string& path)
{
    std::stringstream stream("");
    unsigned int idx;
    for(idx=0;idx<pcdlist.size();++idx)
    {
        std::string currentpath;
        stream.clear();
        stream<<path<<idx<<".pcd";
        stream>>currentpath;
        if(pcdlist[idx]->size()>0)pcl::io::savePCDFileBinary(currentpath,*pcdlist[idx]);
    }
}

void Pipe::debug(const std::vector<cv::Mat>& depth,const std::string& path)
{
    std::stringstream stream("");
    cv::Mat tmp;
    unsigned int idx;
    for(idx=0;idx<depth.size();++idx)
    {
        std::string currentpath;
        stream.clear();
        stream<<path<<idx<<".png";
        stream>>currentpath;
        depth[idx].copyTo(tmp);
        cv::normalize(tmp,tmp,255.0,0.0,cv::NORM_MINMAX);
        cv::imwrite(currentpath,tmp);
    }
}

int Pipe::debug(FullPointCloud::Ptr& a,FullPointCloud::Ptr& b,bool& merge)
{
    QDir dir;
    int _argc = 1;
    char**_argv = new char*[1];
    std::string currentpath = dir.current().absolutePath().toStdString();
    char* path = new char[currentpath.size()];
    strcpy(path,currentpath.c_str());
    _argv[0] = path;
    QApplication app(_argc,_argv);
    CheckView view;
    view.addPointCloud(a,b);
    view.setResult(&merge);
    view.show();
    return app.exec();
}

void Pipe::getSegList(
        const FullPointCloud::Ptr&framefgCloud,
        cv::Mat&idmap,
        std::vector<FullPointCloud::Ptr>&Segs
        )
{
    unsigned int idx;
    int segIdx;
    int segMax = -1;
    if(framefgCloud->size()!=idmap.cols)
    {
        error("the cloud and idmap doesn't match");
        return;
    }
    for(idx=0;idx<framefgCloud->size();++idx)
    {
        if(segIdx>segMax)
        {
            segMax = segIdx;
        }
        segIdx = idmap.at<__int32>(0,idx);
        if(segIdx<0){
//            warn("-1 in seg");
            continue;
        }
        while(segIdx>Segs.size())
        {
            Segs.push_back(FullPointCloud::Ptr(new FullPointCloud));
        }
        Segs[segIdx-1]->push_back(framefgCloud->at(idx));
    }
    for(idx=0;idx<Segs.size();++idx)
    {
        Segs[idx]->height = 1;
        Segs[idx]->width = Segs[idx]->size();
    }
}

int Pipe::configure(Config&)
{
    Pipe::warn("Pipe::configure(const Config&)");
    return -1;
}

int Pipe::init(void)
{
    Pipe::warn("Pipe::init(void)");
    return -1;
}

int Pipe::work(void)
{
    Pipe::warn("Pipe::work(void)");
    return -1;
}

int Pipe::saveToData(void)
{
    Pipe::warn("Pipe::saveToData(void)");
    return -1;
}

void Pipe::outputFalseColorBar(unsigned int firstK,std::string path)
{
    QImage bar(64*firstK,64,QImage::Format_ARGB32);

    unsigned int x,y;
    unsigned int unitX,i;
    for( y = 0 ; y < bar.height() ; ++y )
        for( x = 0 ; x < bar.width() ; ++x )
        {
            i = x / 64;
            unitX = x - i*64;
            QRgb pix = qRgb( FalseColor[i][0], FalseColor[i][1], FalseColor[i][2] ); // unitY == y
            bar.setPixel(x,y,pix);
        }

    bar.save(QString::fromStdString(path));
}

void Pipe::doFalseColorOnPCD(unsigned int coloridx,FullPointCloud::Ptr&pcd)
{
    coloridx %= FalseColorNum;
    unsigned int idx;
    for(idx=0;idx<pcd->size();++idx)
    {
        pcd->at(idx).r = FalseColor[coloridx][0];
        pcd->at(idx).g = FalseColor[coloridx][1];
        pcd->at(idx).b = FalseColor[coloridx][2];
    }
}

void Pipe::doFalseColorOnPCD(cv::Mat& coloridx,FullPointCloud::Ptr&pcd)
{
    unsigned int idx;
    int cidx;
    for(idx=0;idx<coloridx.cols;++idx)
    {
        cidx = coloridx.at<int>(0,idx);
        if(cidx>=0)
        {
            pcd->at(idx).r = FalseColor[cidx][0];
            pcd->at(idx).g = FalseColor[cidx][1];
            pcd->at(idx).b = FalseColor[cidx][2];
        }else{
            pcd->at(idx).r = 0;
            pcd->at(idx).g = 0;
            pcd->at(idx).b = 0;
        }
    }
}

unsigned char Pipe::FalseColor[Pipe::FalseColorNum][3]={
    {0XFF,0X66,0X66},
    {0XFF,0XCC,0X66},
    {0XCC,0XFF,0X66},
    {0X66,0XFF,0X66},
    {0X66,0XFF,0XCC},
    {0X66,0XCC,0XFF},
    {0X66,0X66,0XFF},
    {0XCC,0X66,0XFF},
    {0XFF,0X66,0XCC},
    {0XFF,0X00,0X00},
    {0XFF,0X66,0X00},
    {0XFF,0XCC,0X00},
    {0XCC,0XFF,0X00},
    {0X66,0XFF,0X00},
    {0X00,0XFF,0X00},
    {0X00,0XFF,0X66},
    {0X00,0XFF,0XCC},
    {0X00,0XCC,0XFF},
    {0X00,0X66,0XFF},
    {0X00,0X00,0XFF},
    {0X66,0X00,0XFF},
    {0XCC,0X00,0XFF},
    {0XFF,0X00,0XCC},
    {0XFF,0X00,0X66}
};




