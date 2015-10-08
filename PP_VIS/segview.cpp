#include "segview.h"
#include "ui_segview.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include "region_growing.h"
SegView::SegView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SegView)
{
    ui->setupUi(this);
    currentFrame = 0;
    _PickedIndex.clear();
    ui->widget->setMinimumSize(320,240);
    ui->gridLayout_2->addWidget((QWidget*)(&widget));
    v.initCameraParameters();
    v.addCoordinateSystem(0.3);
    v.setBackgroundColor(1.0,1.0,1.0);
    v.registerPointPickingCallback<SegView>(&SegView::picking,*this,NULL);
    v.registerKeyboardCallback<SegView>(&SegView::key,*this,NULL);

    QString info;
    Pipe::loadData(_FrameKeyList,info,Pipe::_FrameListKey);
    Pipe::loadData(_IdMapKeyList,info,Pipe::_IdMapListKey);
    cloud = FullPointCloud::Ptr(new FullPointCloud);
    connect(ui->toolButton_1,SIGNAL(clicked(bool)),this,SLOT(last()));
    connect(ui->toolButton_2,SIGNAL(clicked(bool)),this,SLOT(next()));
    connect(ui->toolButton_3,SIGNAL(clicked(bool)),this,SLOT(reSeg()));
    connect(ui->toolButton_4,SIGNAL(clicked(bool)),this,SLOT(save()));
    connect(ui->toolButton_5,SIGNAL(clicked(bool)),this,SLOT(reset()));
}

void SegView::next(void)
{
    currentFrame++;
    reLoad();
    updatePointCloud();
}

void SegView::last(void)
{
    if(currentFrame==0)currentFrame=_FrameKeyList.size();
    currentFrame--;
    reLoad();
    updatePointCloud();
}

void SegView::save(void)
{
    Pipe::_PipeData.remove(_IdMapKeyList[currentFrame]);
    Pipe::addToData(idmap,_IdMapKeyList[currentFrame],idinfo);
}

void SegView::reSeg(void)
{
    Pipe::inform("reseg0");
    if(_PickedIndex.size()>=4)
    {
        reSegPlane();
    }
    if( _PickedIndex.size()==2 )
    {
        mergeSeg();
    }
    if( _PickedIndex.size() == 1)
    {
        segToOutlier();
    }
    updateSeg();
}

void SegView::reLoad(void)
{
    if(Pipe::_PipeData.empty())return;
    currentFrame %= _FrameKeyList.size();
    cloud->clear();
    Pipe::loadData( cloud , info , _FrameKeyList[currentFrame] );
    Pipe::loadData(idmap,idinfo,_IdMapKeyList[currentFrame]);
    setWindowTitle(info);
}

void SegView::mergeSeg()
{
    int segidx0 = idmap.at<int>(0,_PickedIndex[0]);
    int segidx1 = idmap.at<int>(0,_PickedIndex[1]);
    int tmp;
    if(segidx0==segidx1)return;
    if(segidx0 > segidx1)
    {
        tmp = segidx0;
        segidx0 = segidx1;
        segidx1 = tmp;
    }
    int idx;
    for(idx=0;idx<idmap.cols;++idx)
    {
        if( segidx1 == idmap.at<int>(0,idx) )
        {
            idmap.at<int>(0,idx) = segidx0;
        }
    }

    for(idx=0;idx<idmap.cols;++idx)
    {
        if( segidx1 < idmap.at<int>(0,idx) )
        {
            idmap.at<int>(0,idx) -= 1;
        }
    }

    int pidx;
    for(pidx=0;pidx<_PickedIndex.size();++pidx)
    {
        QString name;
        name = name.sprintf("%d",pidx);
        v.removeShape(name.toStdString());
    }
    _PickedIndex.clear();
}

void SegView::reset(void)
{
    reLoad();
    updatePointCloud();
}

void SegView::updatePointCloud(bool updateCam)
{
    FullPoint center;
    Pipe::getPCDCenter( cloud , center );

    if(updateCam)v.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgb(cloud);
    v.removeAllPointClouds();
    v.addPointCloud<FullPoint>( cloud , rgb , info.toStdString() );
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, info.toStdString());

    widget.SetRenderWindow(v.getRenderWindow());
    widget.update();
}

void SegView::picking(const pcl::visualization::PointPickingEvent & event, void *)
{
    if(cloud->empty())return;
    if(_PickedIndex.size()>=5)return;
    unsigned int idx = _PickedIndex.size();
    idx%=Pipe::FalseColorNum;
    if(-1==idmap.at<int>(0,event.getPointIndex()))
    {
        Pipe::inform("outlier picked");
        return;
    }
    _PickedIndex.push_back(event.getPointIndex());
    FullPoint p = cloud->at(_PickedIndex[idx]);
    QString name;
    name = name.sprintf("%d",idx);
    float r,g,b;
    r = float(Pipe::FalseColor[idx][0])/255.0;
    g = float(Pipe::FalseColor[idx][1])/255.0;
    b = float(Pipe::FalseColor[idx][2])/255.0;
    v.addSphere (p,0.05,r,g,b,name.toStdString());
}

void SegView::key(const pcl::visualization::KeyboardEvent & event,void*)
{
    if(_PickedIndex.empty())return;
    if("Delete"==event.getKeySym()&&event.keyDown())
    {
        unsigned int idx = _PickedIndex.size()-1;
        QString name;
        name = name.sprintf("%d",idx);
        v.removeShape(name.toStdString());
        _PickedIndex.pop_back();
    }
}

void SegView::updateSeg(void)
{
    FullPointCloud::Ptr cloudptr(new FullPointCloud);
    unsigned int idx;
    segmap.clear();
    for(idx=0;idx<cloud->size();++idx)
    {
        int segidx = idmap.at<int>(0,idx);
        cloudptr->push_back(cloud->at(idx));
        if(segidx==-1)
        {
            cloudptr->at(idx).r = 0;
            cloudptr->at(idx).g = 0;
            cloudptr->at(idx).b = 0;
        }else{
            while(segmap.size()<segidx)
            {
                segmap.push_back(std::vector<int>());
                //start from 1
            }
            segmap[segidx-1].push_back(idx);
            segidx -= 1;
            segidx %= Pipe::FalseColorNum;
            if( segidx < 0 || segidx >= Pipe::FalseColorNum){
                Pipe::inform("Wrong seg idx");
                std::cerr<<segidx<<std::endl;
            }
            cloudptr->at(idx).r = Pipe::FalseColor[segidx][0];
            cloudptr->at(idx).g = Pipe::FalseColor[segidx][1];
            cloudptr->at(idx).b = Pipe::FalseColor[segidx][2];
        }
    }
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgb(cloudptr);
    v.removeAllPointClouds();
    v.addPointCloud<FullPoint>( cloudptr , rgb , info.toStdString() );
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, info.toStdString());
    widget.update();
}

void SegView::reSegPlane(void)
{
    std::cerr<<"resegplane"<<std::endl;
    unsigned int segidx;
    unsigned int pidx;
    segidx = idmap.at<int>(0,_PickedIndex[0]);
    for(pidx=1;pidx<_PickedIndex.size();++pidx)
    {
        if( segidx!=idmap.at<int>(0,_PickedIndex[pidx])  )
        {
            std::cerr<<"selected points were not from same patch"<<std::endl;
            return ;
        }
    }

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    Eigen::Vector3f center;

    initPlane(coeff,center);
    FullPointCloud::Ptr patch = extractPatchCloud(segidx);
    FullPointCloud::Ptr plane = extractPlaneCloud(patch,coeff,center);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<FullPoint> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (plane);
    seg.segment (*inliers, *coeff);

    std::vector<int> reseg;
//    cloud = patch;
    reSegByPlane(patch,coeff,reseg);
    updateIdMap(segidx,reseg);

    for(pidx=0;pidx<_PickedIndex.size();++pidx)
    {
        QString name;
        name = name.sprintf("%d",pidx);
        v.removeShape(name.toStdString());
    }
    _PickedIndex.clear();
}

void SegView::updateIdMap(unsigned int idx,std::vector<int>& reseg)
{
    unsigned int sidx;
    for(sidx=0;sidx<reseg.size();++sidx)
    {
        unsigned int pidx = segmap[idx-1][sidx];
        if( reseg[sidx] > 0 )
        {
            idmap.at<int>( 0 , pidx ) = segmap.size() + reseg[sidx];
        }
    }
}

void SegView::initPlane(pcl::ModelCoefficients::Ptr coeff,Eigen::Vector3f& center)
{
    Eigen::MatrixXf p(3,_PickedIndex.size()-1);

    unsigned int idx;
    for(idx=0;idx<_PickedIndex.size()-1;++idx)
    {
        FullPoint pp = cloud->at(_PickedIndex[idx]);
        p(0,idx) = pp.x;
        p(1,idx) = pp.y;
        p(2,idx) = pp.z;
    }

    center = p.rowwise().mean();

    Eigen::Vector3f v01;
    Eigen::Vector3f v02;
    Eigen::Vector3f n;
    v01 = p.col(1) - p.col(0);
    v02 = p.col(2) - p.col(0);
    n = v01.cross(v02);

    coeff->values.resize(3);

    coeff->values[0] = n(0);
    coeff->values[1] = n(1);
    coeff->values[2] = n(2);
    coeff->values[3] =  -( n.dot(center) );

}

FullPointCloud::Ptr SegView::extractPlaneCloud(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,Eigen::Vector3f& center)
{
    FullPointCloud::Ptr plane(new FullPointCloud);
    unsigned int idx;
    Eigen::Vector3f n;
    n(0) = coeff->values[0];
    n(1) = coeff->values[1];
    n(2) = coeff->values[2];
    Eigen::Vector3f p;
    for(idx=0;idx<patch->size();++idx)
    {
        p(0) = patch->at(idx).x;
        p(1) = patch->at(idx).y;
        p(2) = patch->at(idx).z;
        p = p - center;
        if( ( abs(n.dot(p)) / n.norm() ) < 0.05 )
        {
            plane->push_back(patch->at(idx));
        }
    }
    return plane;
}

FullPointCloud::Ptr SegView::extractPatchCloud(unsigned int segidx)
{
    FullPointCloud::Ptr ptr(new FullPointCloud);
    unsigned int idx;
    for(idx=0;idx<segmap[segidx-1].size();++idx)
    {
        ptr->push_back(cloud->at(segmap[segidx-1][idx]));
    }

    return ptr;
}

void SegView::reSegByPlane(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,std::vector<int>& reseg)
{
    reseg.clear();
    biCut(patch,coeff,reseg);
    if(abs(coeff->values[2])>10*abs(coeff->values[0]) && abs(coeff->values[2]) > 10*abs(coeff->values[1]) )
    {
        regionCut(patch,coeff,reseg);
    }
}

void SegView::regionCut(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,std::vector<int>& reseg)
{
    std::cerr<<"Region Cut:"<<std::endl;
    FullPointCloud::Ptr cutout(new FullPointCloud);
    int idx;
    for(idx=0;idx<patch->size();++idx)
    {
        if(reseg[idx]>0)
        {
            cutout->push_back(patch->at(idx));
        }
    }
    std::cerr<<"size:"<<std::endl;
    std::cerr<<cutout->size()<<std::endl;
    if(cutout->size()<60)return;
    pcl::search::Search<FullPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<FullPoint> > (new pcl::search::KdTree<FullPoint>);
    pcl::NormalEstimationOMP<FullPoint,FullPoint> est;
    est.setSearchMethod (tree);
    est.setViewPoint(0,0,0);
    est.setKSearch(10);
    est.setInputCloud(cutout);
    est.compute(*cutout);

    pcl::RegionGrowing<FullPoint,FullPoint> reg;

    reg.setMinClusterSize(30);
    reg.setMaxClusterSize(std::numeric_limits<int>::max());
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(10);
    reg.setSmoothnessThreshold( 180.0 / 180.0 * M_PI );
    reg.setCurvatureThreshold( std::numeric_limits<float>::max() );

    reg.setInputCloud(cutout);
    reg.setInputNormals(cutout);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    std::cerr<<"cluster size:"<<clusters.size()<<std::endl;
    std::vector<int> lbl(cutout->size(),-1);
    if(clusters.size()==0)return;
    idx = clusters.size() - 1;
    while( idx >= 0 )
    {
        std::vector<int>::iterator iter;
        iter = clusters[idx].indices.begin();
        for(;iter!=clusters[idx].indices.end();++iter)
        {
            lbl[*iter]=idx;
        }
        --idx;
    }
    int lidx=0;
    for(idx=0;idx<reseg.size();++idx)
    {
        if(reseg[idx]>0)
        {
            if(-1==lbl[lidx])
            {
                reseg[idx]=0;
            }else{
                reseg[idx] += lbl[lidx];
            }
            ++lidx;
        }
    }
}

void SegView::biCut(FullPointCloud::Ptr patch,pcl::ModelCoefficients::Ptr coeff,std::vector<int>& reseg)
{
    unsigned int idx;
    Eigen::Vector3f n;
    Eigen::Vector3f c;
    Eigen::Vector3f last;
    n(0) = coeff->values[0];
    n(1) = coeff->values[1];
    n(2) = coeff->values[2];
    // a point on plane
    c(0) = 0;
    c(1) = 0;
    c(2) = - coeff->values[3] / coeff->values[2];
    // the choice of side
    last(0) = cloud->at(_PickedIndex.back()).x;
    last(1) = cloud->at(_PickedIndex.back()).y;
    last(2) = cloud->at(_PickedIndex.back()).z;
    last = last - c;

    std::cerr<<"n"<<std::endl;
    std::cerr<<n<<std::endl;
    std::cerr<<last<<std::endl;
    float side = last.dot(n);

    Eigen::Vector3f current;

    unsigned int cnt = 0;
    for(idx=0;idx<patch->size();++idx)
    {
        current(0) = patch->at(idx).x;
        current(1) = patch->at(idx).y;
        current(2) = patch->at(idx).z;
        current = current - c;

        float dir = current.dot(n);

        dir /= n.norm();

        if ( ( ( dir >= -0.01 && side >= 0 ) || ( dir <= 0.01 && side <=0 ) )  )
        {
            reseg.push_back(0);
        }else{
            cnt ++;
            reseg.push_back(1);
        }
    }
}

void SegView::segToOutlier(void)
{
    int segidx0 = idmap.at<int>(0,_PickedIndex[0]);
    if(segidx0==-1)return;
    int idx;
    for(idx=0;idx<idmap.cols;++idx)
    {
        if( segidx0 == idmap.at<int>(0,idx) )
        {
            idmap.at<int>(0,idx) = -1;
        }
    }

    for(idx=0;idx<idmap.cols;++idx)
    {
        if( segidx0 < idmap.at<int>(0,idx) )
        {
            idmap.at<int>(0,idx) -= 1;
        }
    }
    int pidx;
    for(pidx=0;pidx<_PickedIndex.size();++pidx)
    {
        QString name;
        name = name.sprintf("%d",pidx);
        v.removeShape(name.toStdString());
    }
    _PickedIndex.clear();
}

SegView::~SegView()
{
    delete ui;
}
