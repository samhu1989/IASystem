#include "oversegview.h"
#include "ui_oversegview.h"
#include <QFileDialog>
#include <QMessageBox>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyLine.h>
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
    QFileDialog fd(this,tr("Load Supervoxel"), "./XK/overseg/","" );
    fd.setFileMode(QFileDialog::AnyFile);
    fd.setViewMode(QFileDialog::Detail);
    fd.setAcceptMode(QFileDialog::AcceptOpen);
    fd.setNameFilter(QString(tr("Supervoxel(*.svx)")));
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
        SuperVoxelClusters& vox,
        SuperVoxelAdjacency& adj
        )
{
    addVoxelCentroidCloudToViewer(vox);
    addVoxelColoredCloudToViewer(vox);
//    std::multimap<uint32_t,uint32_t>::iterator label_itr = adj.begin ();
//    for ( ; label_itr != adj.end (); )
//    {
//        //First get the label
//        uint32_t supervoxel_label = label_itr->first;
//        //Now get the supervoxel corresponding to the label
//        pcl::Supervoxel<ColorPoint>::Ptr supervoxel = vox.at (supervoxel_label);

//        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
//        ColorPointCloud adjacent_supervoxel_centers;
//        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = adj.equal_range (supervoxel_label).first;
//        for ( ; adjacent_itr!=adj.equal_range(supervoxel_label).second; ++adjacent_itr)
//        {
//            pcl::Supervoxel<ColorPoint>::Ptr neighbor_supervoxel = vox.at(adjacent_itr->second);
//            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
//        }
//        adjacent_supervoxel_centers.width = adjacent_supervoxel_centers.size();
//        adjacent_supervoxel_centers.height = 1;
//        //Now we make a name for this polygon
//        std::stringstream ss;
//        ss << "supervoxel_" << supervoxel_label;
//        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
//        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str() );
//        //Move iterator forward to next label
//        label_itr = adj.upper_bound (supervoxel_label);
//    }
    widget.SetRenderWindow(v.getRenderWindow());
    widget.update();
    QMessageBox::information(this,tr("Loaded"),tr("The Supervoxel is ready"));
}

void OverSegView::addVoxelCentroidCloudToViewer(
        SuperVoxelClusters& vox
        )
{
    ColorPointCloud::Ptr voxel_centroid_cloud(new ColorPointCloud);
    SuperVoxelClusters::iterator iter;
    for(iter=vox.begin();iter!=vox.end();++iter)
    {
        SuperVoxel::Ptr voxel = iter->second;
        voxel_centroid_cloud->push_back(voxel->centroid_);
    }
    voxel_centroid_cloud->width = voxel_centroid_cloud->size();
    voxel_centroid_cloud->height = 1;
    pcl::visualization::PointCloudColorHandlerCustom<ColorPoint>color(voxel_centroid_cloud,0,0,255);
    v.addPointCloud (voxel_centroid_cloud,color,"voxel centroids");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5.0, "voxel centroids");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "voxel centroids");
}

void OverSegView::addVoxelColoredCloudToViewer(
        SuperVoxelClusters& vox
        )
{
    ColorPointCloud::Ptr voxel_colored_cloud(new ColorPointCloud);
    SuperVoxelClusters::iterator iter;
    for(iter=vox.begin();iter!=vox.end();++iter)
    {
        SuperVoxel::Ptr voxel = iter->second;
        ColorPointCloud::Ptr colored_cloud(new ColorPointCloud);
        generateColoredCloud(iter->first,*voxel,*colored_cloud);
        *voxel_colored_cloud += *colored_cloud;
    }
    voxel_colored_cloud->width = voxel_colored_cloud->size();
    voxel_colored_cloud->height = 1;
    pcl::visualization::PointCloudColorHandlerRGBField<ColorPoint>color(voxel_colored_cloud);
    v.addPointCloud (voxel_colored_cloud,color,"voxel label");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel label");
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel label");
}

void OverSegView::generateColoredCloud(
        uint32_t label,
        SuperVoxel&vox,
        ColorPointCloud&cloud
        )
{
    cloud = *vox.voxels_;
    int idx = label;
    idx%=Pipe::FalseColorNum;
    ColorPointCloud::iterator iter;
    for(iter=cloud.begin();iter!=cloud.end();++iter)
    {
        iter->r = Pipe::FalseColor[idx][0];
        iter->g = Pipe::FalseColor[idx][1];
        iter->b = Pipe::FalseColor[idx][2];
    }
}

//void
//OverSegView::addSupervoxelConnectionsToViewer (ColorPoint &supervoxel_center,
//                                  ColorPointCloud &adjacent_supervoxel_centers,
//                                  const std::string& supervoxel_name)
//{
//    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
//    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

//    //Iterate through all adjacent points, and add a center point to adjacent point pair
//    ColorPointCloud::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
//    for ( ;adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
//    {
//        points->InsertNextPoint (supervoxel_center.data);
//        points->InsertNextPoint (adjacent_itr->data);
//    }
//    // Create a polydata to store everything in
//    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
//    // Add the points to the dataset
//    polyData->SetPoints(points);
//    polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
//    for(unsigned int i = 0; i < points->GetNumberOfPoints(); i++ )
//        polyLine->GetPointIds()->SetId(i,i);
//    cells->InsertNextCell(polyLine);
//    // Add the lines to the dataset
//    polyData->SetLines(cells);
//    if(!v.addModelFromPolyData(polyData,supervoxel_name))
//    {
//        Pipe::error("Failed to add Model");
//        std::cerr<<"Failed to add "<<supervoxel_name<<std::endl;
//    }
//    v.setShapeRenderingProperties(
//                pcl::visualization::PCL_VISUALIZER_COLOR,
//                0.0,0.0,1.0,
//                supervoxel_name
//                );
//    v.setShapeRenderingProperties(
//                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
//                5.0,
//                supervoxel_name
//                );
//}

OverSegView::~OverSegView()
{
    delete ui;
}
