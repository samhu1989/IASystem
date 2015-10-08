#include "icpwidget.h"
#include "ui_icpwidget.h"
#include "objview.h"
#include "icpdialog.h"
#include <QFileDialog>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_lib_io.h>
ICPWidget::ICPWidget(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::ICPWidget)
{
    ui->setupUi(this);

    v.initCameraParameters();
    v.setBackgroundColor(1.0,1.0,1.0);
    v.addCoordinateSystem(0.3);
    v.registerPointPickingCallback<ICPWidget>(&ICPWidget::pick,*this,NULL);
    v.registerKeyboardCallback<ICPWidget>(&ICPWidget::key,*this,NULL);

    widget.SetRenderWindow(v.getRenderWindow());

    ui->frameView->layout()->addWidget(&widget);
    ui->tools->setCurrentIndex(0);

    QString dataInfo;
    Pipe::loadData(_FrameKeyList,dataInfo,Pipe::_FrameListKey);
    Pipe::loadData(_IdMapKeyList,dataInfo,Pipe::_IdMapListKey);
    frameCloud = FullPointCloud::Ptr(new FullPointCloud);
    segCloud = FullPointCloud::Ptr(new FullPointCloud);
    currentFrame = 0;
    currentState = PICK_FRAME;
    currentObjIndex = -1;

    connect(ui->nextFrame,SIGNAL(clicked()),this,SLOT(nextFrame()));
    connect(ui->lastFrame,SIGNAL(clicked()),this,SLOT(lastFrame()));
    connect(ui->loadFrame,SIGNAL(clicked()),this,SLOT(reLoadFrameWithView()));
    connect(ui->addObj,SIGNAL(clicked()),this,SLOT(addObj()));
    connect(ui->delObj,SIGNAL(clicked()),this,SLOT(delObj()));
    connect(ui->icpObj,SIGNAL(clicked()),this,SLOT(icpObj()));

    connect(ui->tools,SIGNAL(currentChanged(int)),this,SLOT(changeState(int)));
    connect(ui->outObj,SIGNAL(clicked()),this,SLOT(outputObj()));
    connect(ui->outFrame,SIGNAL(clicked()),this,SLOT(outputFrame()));
}

void ICPWidget::nextFrame(void)
{
    currentFrame++;
    reLoadFrame();
    updateShow();
}

void ICPWidget::lastFrame(void)
{
    if(currentFrame<=0)currentFrame=_FrameKeyList.size();
    currentFrame--;
    reLoadFrame();
    updateShow();
}

void ICPWidget::reLoadFrame(void)
{
    if(Pipe::_PipeData.empty())return;
    currentFrame %= _FrameKeyList.size();
    frameCloud->clear();
    Pipe::loadData( frameCloud , info , _FrameKeyList[currentFrame] );
    Pipe::loadData( idmap , idinfo , _IdMapKeyList[currentFrame]);
    generateSeg();
    //clear current layout
    alignedObjIndex.clear();
    alignedObjLayout.clear();
    ui->statusText->setText(info);
}

void ICPWidget::reLoadFrameWithView(void)
{
    updateShow(true);
}

void ICPWidget::updateShow(bool updateCam)
{
    switch(currentState)
    {
    case PICK_FRAME:
        showCloud = frameCloud;
        break;
    case PICK_PATCH:
        showCloud = segCloud;
        break;
    }

    FullPoint center;
    Pipe::getPCDCenter( showCloud , center );
    if(updateCam)v.setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    v.removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgb(showCloud);
    v.addPointCloud<FullPoint>( showCloud , rgb , info.toStdString() );
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, info.toStdString());

    widget.SetRenderWindow(v.getRenderWindow());
    widget.update();
}

void ICPWidget::init(void)
{
    reLoadFrame();
}

void ICPWidget::pick(const pcl::visualization::PointPickingEvent & event, void*)
{
    if(currentState!=PICK_PATCH)return;
    if(segCloud->empty())return;
//    if(_PickedIndex.size()>=5)return;
    unsigned int idx = _PickedIndex.size();
    idx%=Pipe::FalseColorNum;
    if(-1==idmap.at<int>(0,event.getPointIndex()))
    {
        Pipe::inform("outlier picked");
        return;
    }
    _PickedIndex.push_back(event.getPointIndex());
    FullPoint p = segCloud->at(_PickedIndex[idx]);
    QString name;
    name = name.sprintf("%d",idx);
    float r,g,b;
    r = float(Pipe::FalseColor[idx][0])/255.0;
    g = float(Pipe::FalseColor[idx][1])/255.0;
    b = float(Pipe::FalseColor[idx][2])/255.0;
    v.addSphere (p,0.05,r,g,b,name.toStdString());
    widget.update();
}

void ICPWidget::key(const pcl::visualization::KeyboardEvent& event, void*)
{
    if(_PickedIndex.empty())return;
    if("Delete"==event.getKeySym()&&event.keyDown())
    {
        unsigned int idx = _PickedIndex.size()-1;
        QString name;
        name = name.sprintf("%d",idx);
        v.removeShape(name.toStdString());
        _PickedIndex.pop_back();
        widget.update();
    }
}

void ICPWidget::generateSeg(void)
{
    unsigned int idx;
    segmap.clear();
    segCloud->clear();

    for(idx=0;idx<frameCloud->size();++idx)
    {
        int segidx = idmap.at<int>(0,idx);
        segCloud->push_back(frameCloud->at(idx));
        if(segidx==-1)
        {
            segCloud->at(idx).r = 0;
            segCloud->at(idx).g = 0;
            segCloud->at(idx).b = 0;
        }else{
            while(segmap.size()<segidx)
            {
                segmap.push_back(std::vector<int>());
            }
            segmap[segidx-1].push_back(idx);
            segidx -= 1;
            segidx %= Pipe::FalseColorNum;
            if( segidx < 0 || segidx >= Pipe::FalseColorNum){
                Pipe::inform("Wrong seg idx");
                std::cerr<<segidx<<std::endl;
            }
            segCloud->at(idx).r = Pipe::FalseColor[segidx][0];
            segCloud->at(idx).g = Pipe::FalseColor[segidx][1];
            segCloud->at(idx).b = Pipe::FalseColor[segidx][2];
        }
    }
}

void ICPWidget::changeState(int idx)
{
    switch(idx)
    {
    case 0:currentState = PICK_FRAME;
        break;
    case 1:currentState = PICK_PATCH;
        break;
    }
    updateShow();
}

void ICPWidget::addObj(void)
{
    if( _PickedIndex.empty() )return;
    FullPointCloud::Ptr patch(new FullPointCloud);

    int currentPatch;
    currentPatch = idmap.at<int>(0,_PickedIndex.back());
    if(currentPatch<1)return;
    std::vector<int>& idxlst = segmap[currentPatch-1];
    unsigned int idx;
    for(idx=0;idx<idxlst.size();++idx)
    {
        patch->push_back(frameCloud->at(idxlst[idx]));
    }
    patch->width = patch->size();
    patch->height = 1;

    _ObjList.push_back(GeoObj::Ptr(new GeoObj(patch,int(_ObjList.size()))));

    ObjView* w = new ObjView(_ObjList.back(),NULL);
    ui->scrollLayout->addWidget(w);
    connect(w,SIGNAL(select(int)),this,SLOT(selectObj(int)));
    connect(w,SIGNAL(deSelect(int)),this,SLOT(deselectObj(int)));
    connect(this,SIGNAL(currentObj(int)),w,SLOT(acceptSelect(int)));
    connect(this,SIGNAL(deleteObj(int)),w,SLOT(acceptDelete(int)));

    v.removeAllShapes();
    _PickedIndex.clear();
}

void ICPWidget::selectObj(int idx)
{
    std::cerr<<"selected:"<<idx<<std::endl;
    currentObjIndex = idx;
    emit currentObj(currentObjIndex);
}

void ICPWidget::deselectObj(int)
{
    currentObjIndex = -1;
}

void ICPWidget::delObj(void)
{
    if(-1==currentObjIndex)return;
    emit deleteObj(currentObjIndex);
    _ObjList.erase(_ObjList.begin()+currentObjIndex);
    std::vector<GeoObj::Ptr>::iterator iter;
    int index = 0;
    for(iter=_ObjList.begin();iter!=_ObjList.end();++iter)
    {
        (*iter)->setIndex(index);
        ++index;
    }
    currentObjIndex = -1;
}

void ICPWidget::icpObj(void)
{
    if(-1==currentObjIndex)return;
    if(_PickedIndex.empty())return;
    //extract patch
    FullPointCloud::Ptr patch(new FullPointCloud);
    int currentPatch;
    currentPatch = idmap.at<int>(0,_PickedIndex.back());
    if(currentPatch<1)return;
    std::vector<int>& idxlst = segmap[currentPatch-1];
    unsigned int idx;
    for(idx=0;idx<idxlst.size();++idx)
    {
        patch->push_back(frameCloud->at(idxlst[idx]));
    }
    patch->width = patch->size();
    patch->height = 1;
    //extract obj model
    FullPointCloud::Ptr obj = _ObjList[currentObjIndex]->getCloud();
    ICPDialog dialog;
    //do icp
    Eigen::Matrix4f result;
    dialog.setA(patch);
    dialog.setB(obj);
    dialog.reset();
    if(0==dialog.exec())
    {
        dialog.getT(result);
    }else{
        Pipe::inform("not working in icp");
        return;
    }
    alignedObjIndex.push_back(currentObjIndex);
    alignedObjLayout.push_back(
                transformLayout(
                    _ObjList[currentObjIndex]->getBox(),
                    result
                )
                );
}

void ICPWidget::outputObj(void)
{
    QFileDialog fd(this,tr("Choose Output Path"), "./","" );
    fd.setFileMode(QFileDialog::DirectoryOnly);
    fd.setViewMode(QFileDialog::Detail);
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }else return;
    if(fileNamesList.empty())return;
    currentPath = ( fileNamesList.back() + "/" ).toStdString() ;
    outputObjBox();
    outputObjModel();
}

void ICPWidget::outputObjBox(void)
{
    std::fstream objfile;
    std::stringstream stream("");
    std::string objfilename;

    unsigned int idx=0,c;
    unsigned int cnt = 1;

    std::vector<GeoObj::Ptr>::iterator iter;
    for(iter=_ObjList.begin();iter!=_ObjList.end();++iter)
    {
        GeoObj::Ptr objptr = *iter;
        stream.clear();
        stream<<currentPath<<"obj_box"<<idx<<"_"<<objptr->getName().toStdString();
        stream>>objfilename;
        objfile.open(objfilename+".obj",objfile.out);
        std::cerr<<objfilename<<std::endl;
        objfile<<"#obj "<<idx<<std::endl;
        cv::Mat layout = objptr->getBox();
        for(c=0;c<layout.cols;++c)
        {
            objfile<<"v "<<layout.at<float>(0,c)
                       <<" "<<layout.at<float>(1,c)
                       <<" "<<layout.at<float>(2,c)
                       <<std::endl;
        }
        objfile<<"f "<<cnt+0<<" "<<cnt+1<<" "<<cnt+2<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+4<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+7<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+5<<" "<<cnt+1<<std::endl;
        objfile<<"f "<<cnt+1<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+2<<std::endl;
        objfile<<"f "<<cnt+2<<" "<<cnt+6<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        objfile.close();
        ++idx;
    }
}

void ICPWidget::outputObjModel(void)
{
    FullPointCloud::Ptr obj(new FullPointCloud);
    std::stringstream stream("");
    std::string objfilename;
    QString info;
    obj->clear();
    Pipe::loadData(obj,info,Pipe::_BGKey);

    stream.clear();
    stream<<currentPath<<"obj"<<0<<"_background";
    stream>>objfilename;

    pcl::Poisson<FullPoint> poisson;
    pcl::io::savePCDFileBinary(objfilename+".pcd",*obj);
    pcl::PolygonMesh mesh;
    poisson.setInputCloud(obj);
    poisson.performReconstruction(mesh);
    pcl::io::savePolygonFile(objfilename+".ply",mesh);

    unsigned int idx = 1;
    foreach(GeoObj::Ptr objptr,_ObjList)
    {
        stream.clear();
        stream<<currentPath<<"obj"<<idx<<"_"<<objptr->getName().toStdString();
        stream>>objfilename;
        pcl::io::savePCDFileBinary(objfilename+".pcd",*(objptr->getCloud()));
        pcl::PolygonMesh mesh;
        poisson.setInputCloud(objptr->getCloud());
        poisson.performReconstruction(mesh);
        pcl::io::savePolygonFile(objfilename+".ply",mesh);
        ++idx;
    }
}

void ICPWidget::outputFrame(void)
{
    QFileDialog fd(this,tr("Choose Output Path"), "./","" );
    fd.setFileMode(QFileDialog::DirectoryOnly);
    fd.setViewMode(QFileDialog::Detail);
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }else return;
    if(fileNamesList.empty())return;
    currentPath = ( fileNamesList.back() + "/" ).toStdString() ;
    outputFrame(
        currentFrame,
        alignedObjLayout,
        alignedObjIndex
    );
}

void ICPWidget::outputFrame(
        unsigned int FrameIdx,
        const std::vector<cv::Mat>&layouts,
        const std::vector<unsigned int>&objIdx
        )
{
    std::fstream objfile;
    std::stringstream stream("");
    std::string objfilename;
    stream<<currentPath<<"scene_"<<FrameIdx<<".obj";
    stream>>objfilename;
    objfile.open(objfilename,objfile.out);
    unsigned int idx,c;
    unsigned int cnt = 1;
    Pipe::inform("Output Scene");
    std::cerr<<objfilename<<std::endl;
    for(idx=0;idx<layouts.size();++idx)
    {
        objfile<<"#obj "<<objIdx[idx]<<std::endl;
        cv::Mat layout = layouts[idx];
        for(c=0;c<layout.cols;++c)
        {
            objfile<<"v "<<layout.at<float>(0,c)
                       <<" "<<layout.at<float>(1,c)
                       <<" "<<layout.at<float>(2,c)
                       <<std::endl;
        }
        objfile<<"f "<<cnt+0<<" "<<cnt+1<<" "<<cnt+2<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+4<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+7<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        objfile<<"f "<<cnt+0<<" "<<cnt+4<<" "<<cnt+5<<" "<<cnt+1<<std::endl;
        objfile<<"f "<<cnt+1<<" "<<cnt+5<<" "<<cnt+6<<" "<<cnt+2<<std::endl;
        objfile<<"f "<<cnt+2<<" "<<cnt+6<<" "<<cnt+7<<" "<<cnt+3<<std::endl;
        cnt+=8;
    }
    objfile.close();
}

cv::Mat ICPWidget::transformLayout(cv::Mat&layout,Eigen::Matrix4f& T)
{
    cv::Mat result;
    cv::Mat t;
    layout.copyTo(result);
    Pipe::getMat(T,t);
    Pipe::transformOBB(t,result);
    return result;
}


ICPWidget::~ICPWidget()
{
    delete ui;
}
