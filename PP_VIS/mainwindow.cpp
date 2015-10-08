#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include "pipe.h"
#include "icpwidget.h"
#include "formattools.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    segview = NULL;
    datalist = NULL;
    fview = NULL;

    connect(ui->actionOpen,SIGNAL(triggered(bool)),this,SLOT(loadDump()));
    connect(ui->actionSegment,SIGNAL(triggered(bool)),this,SLOT(startSeg()));
    connect(ui->actionSave,SIGNAL(triggered(bool)),this,SLOT(saveDump()));
    connect(ui->actionViewFeature,SIGNAL(triggered(bool)),this,SLOT(startFeature()));
    connect(ui->actionMapping,SIGNAL(triggered()),this,SLOT(domap()));
    connect(ui->actionICP,SIGNAL(triggered()),this,SLOT(doicp()));
    connect(ui->actionFormatTool,SIGNAL(triggered()),this,SLOT(doformat()));
}

void MainWindow::loadDump(void)
{
    QFileDialog fd(this,tr("Choose Dump"), "./dump/","" );
    fd.setFileMode(QFileDialog::ExistingFile);
    fd.setViewMode(QFileDialog::Detail);
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }
    if(fileNamesList.empty())return;
    Pipe::loadPipeDataFrom(fileNamesList[0].toStdString());
    datalist = new DataList();
    datalist->setAttribute(Qt::WA_DeleteOnClose,true);
    ui->mdiArea->addSubWindow(datalist);
    connect(datalist,SIGNAL(destroyed(QObject*)),this,SLOT(reset(QObject*)));
    connect(datalist,SIGNAL(passToMdi(QWidget*)),this,SLOT(showInMdi(QWidget*)));
    datalist->show();
}

void MainWindow::saveDump(void)
{
    QFileDialog fd(this,tr("Saving Dump"), "./dump/","" );
    fd.setFileMode(QFileDialog::AnyFile);
    fd.setViewMode(QFileDialog::Detail);
    fd.setAcceptMode(QFileDialog::AcceptSave);
    fd.setNameFilter(QString(tr("Dump File (*.IA.dump)")));
    QStringList fileNamesList;
    if(fd.exec()) // ok
    {
        fileNamesList=fd.selectedFiles();
    }
    if(fileNamesList.empty())return;
    QString fname = fileNamesList[0];
    if(!fname.endsWith("IA.dump"))
    {
        fname += ".IA.dump";
    }
    if(!Pipe::_PipeData.empty())Pipe::dumpTo( fname.toStdString() );
}

void MainWindow::startSeg(void)
{
    if(segview!=NULL)return;
    if(Pipe::_PipeData.empty())return;
    QList<quint32> idmaplist;
    QString info;
    Pipe::loadData(idmaplist,info,Pipe::_IdMapListKey);
    if(idmaplist.empty())return;
    segview = new SegView();
    segview->setAttribute(Qt::WA_DeleteOnClose,true);
    ui->mdiArea->addSubWindow(segview);
    connect(segview,SIGNAL(destroyed(QObject*)),this,SLOT(reset(QObject*)));
//    connect(datalist,SIGNAL(passToMdi(QWidget*)),this,SLOT(showInMdi(QWidget*)));
    segview->show();
    segview->reLoad();
    segview->updatePointCloud(true);
}

void MainWindow::startFeature(void)
{
    if(fview!=NULL)return;
    if(Pipe::_PipeData.empty())return;
    QList<quint32> idmaplist;
    QString info;
    Pipe::loadData(idmaplist,info,Pipe::_IdMapListKey);
    if(idmaplist.empty())return;
    fview = new FeatureView();
    fview->setAttribute(Qt::WA_DeleteOnClose,true);
    ui->mdiArea->addSubWindow(fview);
    connect(fview,SIGNAL(destroyed(QObject*)),this,SLOT(reset(QObject*)));
    connect(fview,SIGNAL(passToMdi(QWidget*)),this,SLOT(showInMdi(QWidget*)));
    fview->show();
    fview->reLoad();
    fview->updatePointCloud(true);
}

void MainWindow::doicp(void)
{
    if(Pipe::_PipeData.empty())return;
    ICPWidget* w  = new ICPWidget();
    showInMdi((QWidget*)w);
    w->init();
}

void MainWindow::domap(void)
{
//    Mapper* w = new Mapper();
//    connect(this,SIGNAL(destroyed()),w,SLOT(deleteLater()));
//    showInMdi((QWidget*)(w));
}

void MainWindow::doformat(void)
{
    FormatTools* w = new FormatTools();
    showInMdi((QWidget*)(w));
}

void MainWindow::showInMdi(QWidget* w)
{
    w->setAttribute(Qt::WA_DeleteOnClose,true);
    ui->mdiArea->addSubWindow(w);
    connect(this,SIGNAL(destroyed()),w,SLOT(deleteLater()));
    w->show();
}

void MainWindow::reset(QObject* o)
{
    if(o==(QObject*)datalist)datalist=NULL;
    if(o==(QObject*)segview)segview=NULL;
    if(o==(QObject*)fview)fview=NULL;
}

void MainWindow::keyPressEvent(QKeyEvent*e)
{
    QMainWindow::keyPressEvent(e);
}

MainWindow::~MainWindow()
{
    if(fview)delete fview;
    if(segview) delete segview;
    if(datalist) delete datalist;
    delete ui;
}
