#include "objview.h"
#include "ui_objview.h"

ObjView::ObjView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ObjView)
{
    ui->setupUi(this);
}

ObjView::ObjView(GeoObj::Ptr ptr,QWidget *parent):
    QWidget(parent),
    ui(new Ui::ObjView)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose);
    setFixedHeight(200);
    _ObjPtr = GeoObj::Ptr(ptr);
    _ObjPtr->setName(ui->nameline->text());
    ui->gridLayout_2->addWidget(&w);
    v = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer());
    v->setBackgroundColor(0.8,0.8,0.8);
    v->addCoordinateSystem(0.3);

    FullPointCloud::Ptr cloud = _ObjPtr->getCloud();
    FullPoint center;
    Pipe::getPCDCenter( cloud , center );

    v->setCameraPose(0,0,0,center.x,center.y,center.z,0,0,1);
    pcl::visualization::PointCloudColorHandlerRGBField<FullPoint> rgb(cloud);
    v->removeAllPointClouds();
    v->addPointCloud<FullPoint>( cloud , rgb , "obj" );
    v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "obj");

    w.SetRenderWindow(v->getRenderWindow());
    w.update();

    connect(ui->nameline,SIGNAL(textChanged(QString)),this,SLOT(changeName(QString)));
    connect(ui->radioButton,SIGNAL(toggled(bool)),this,SLOT(informSelect(bool)));
}

void ObjView::informSelect(bool checked)
{
    if(checked)emit select(_ObjPtr->index());
    else emit deSelect(_ObjPtr->index());
}

void ObjView::acceptSelect(int i)
{
    if(i!=_ObjPtr->index())
    {
        disconnect(ui->radioButton,SIGNAL(toggled(bool)),this,SLOT(informSelect(bool)));
        ui->radioButton->setChecked(false);
        connect(ui->radioButton,SIGNAL(toggled(bool)),this,SLOT(informSelect(bool)));
    }
}

void ObjView::acceptDelete(int i)
{
    if(i==_ObjPtr->index())close();
}

void ObjView::changeName(QString name)
{
    _ObjPtr->setName(name);
}

ObjView::~ObjView()
{
    std::cerr<<"delete:"<<_ObjPtr->index()<<std::endl;
    delete ui;
}
