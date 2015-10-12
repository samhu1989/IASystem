#include "oversegview.h"
#include "ui_oversegview.h"
#include <QFileDialog>
OverSegView::OverSegView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::OverSegView)
{
    ui->setupUi(this);
    v.setBackgroundColor(0.7,0.7,0.7);
    ui->w->setMinimumSize(320,240);
    v.addCoordinateSystem(0.3);
    ui->viewGridLayout->addWidget(&widget);
    widget.SetRenderWindow(v.getRenderWindow());
    connect( ui->loadFromFile,SIGNAL(clicked()),this,SLOT(showFromFile()) );
}

void OverSegView::showFromFile(void)
{
    QFileDialog fd(this,tr("Load Configure"), "./XK/overseg/","" );
    fd.setFileMode(QFileDialog::AnyFile);
    fd.setViewMode(QFileDialog::Detail);
    fd.setAcceptMode(QFileDialog::AcceptOpen);
    fd.setNameFilter(QString(tr("Configure File(*.config)")));
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }
    if(fileNamesList.empty())return;
    XKCommon::SuperVoxelClusters clusters;
    XKCommon::SuperVoxelAdjacency adjacency;
    QFile file(fileNamesList.front());
    file.open(QFile::ReadOnly);
    QDataStream stream(&file);
    XKCommon::loadClusters(stream,clusters);
    XKCommon::loadAdjacency(stream,adjacency);
    showSuperVoxel(clusters,adjacency);
    file.close();
}

void OverSegView::showFromProc(void)
{
    ;
}

void OverSegView::showSuperVoxel(
        SuperVoxelClusters&,
        SuperVoxelAdjacency&
        )
{
    ;
}

OverSegView::~OverSegView()
{
    delete ui;
}
