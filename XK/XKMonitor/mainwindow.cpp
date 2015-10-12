#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDataStream>
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include "xkcommon.h"
#include "oversegview.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    _Server = new QLocalServer();
    _Server->removeServer("XK");
    _Server->listen("XK");
    _Config = NULL;
    connect(_Server,SIGNAL(newConnection()),this,SLOT(newSocket()));
    connect(ui->actionLoadConfig,SIGNAL(triggered()),this,SLOT(loadConfig()));
    connect(ui->actionOverSeg,SIGNAL(triggered()),this,SLOT(triggerOverSeg()));
    connect(ui->actionOverSegView,SIGNAL(triggered()),this,SLOT(viewOverSeg()));
}

MainWindow::~MainWindow()
{
    _Server->close();
    _Server->deleteLater();
    _Server = NULL;
    delete ui;
}

void MainWindow::newSocket(void)
{
    QLocalSocket* socket = _Server->nextPendingConnection();
    connect(socket,SIGNAL(readyRead()),this,SLOT(recieve()));
}

void MainWindow::send(void)
{
    ;
}

void MainWindow::recieve(void)
{
    std::cerr<<"recieved"<<std::endl;
    QLocalSocket* socket = static_cast<QLocalSocket*>(sender());
    QByteArray data = socket->readAll();
    QDataStream stream(&data,QIODevice::ReadWrite);
    QString type;
    stream>>type;
    respond(socket,type,data);
}

void MainWindow::respond(QLocalSocket*socket,QString&type,QByteArray&data)
{
    if(type=="Name")
    {
        QDataStream stream(&data,QIODevice::ReadWrite);
        QString name;
        stream>>name;
        std::cerr<<name.toStdString()<<std::endl;
        stream>>name;
        std::cerr<<name.toStdString()<<std::endl;
        _Sockets[name]=socket;
    }
}

void MainWindow::loadConfig(void)
{
    if(_Config!=NULL)
    {
        delete _Config;
        _Config = NULL;
    }
    QFileDialog fd(this,tr("Load Configure"), "./config/","" );
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
    QString fname = fileNamesList[0];
    _Config = new Config(fname.toStdString());
}

void MainWindow::finishOverSeg(int, QProcess::ExitStatus)
{
    QProcess* p = static_cast<QProcess*>(sender());
    if(p)
    {
        Pipe::inform("Done Work in");
        std::cerr<<p->nativeArguments().toStdString()<<std::endl;
        std::cerr<<"============================================"<<std::endl;
    }
}

void MainWindow::triggerOverSeg(void)
{
    if(!_Config)
    {
        QMessageBox::warning(this,tr("No Configuration"),tr("Please Load Configuration First"));
        return;
    }
    QProcess* p = new QProcess();
    connect(p,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(finishOverSeg(int,QProcess::ExitStatus)));
    connect(p,SIGNAL(readyReadStandardOutput()),this,SLOT(printProcessStdCout()));
    connect(p,SIGNAL(readyReadStandardError()),this,SLOT(printProcessStdCerr()));
    connect(p,SIGNAL(error(QProcess::ProcessError)),this,SLOT(printProcessError(QProcess::ProcessError)));
    connect(this,SIGNAL(destroyed()),p,SLOT(kill()));
    connect(this,SIGNAL(destroyed()),p,SLOT(deleteLater()));
    Pipe::inform("Starting Over-Segment");
    QString arguments;
    QStringList keys;
    keys<<"oversegin"<<"oversegout"<<"oversegsuffix"<<"oversegprefix";
    keys<<"voxelres"<<"seedres"<<"color_w"<<"spatial_w"<<"norm_w";
    arguments = XKCommon::getNativeArguments(*_Config,keys);
    p->setNativeArguments(arguments);
    p->start("./bin/XKOverSeg.exe");
}

void MainWindow::printProcessStdCout(void)
{
    QProcess* p = static_cast<QProcess*>(sender());
    if(p)std::cout<<QString(p->readAllStandardOutput()).toStdString();
}

void MainWindow::printProcessStdCerr(void)
{
    QProcess* p = static_cast<QProcess*>(sender());
    if(p)std::cerr<<QString(p->readAllStandardError()).toStdString();
}

void MainWindow::printProcessError(QProcess::ProcessError e)
{
    QProcess* p = static_cast<QProcess*>(sender());
    if(p)
    {
        Pipe::inform("Error in");
        std::cerr<<p->nativeArguments().toStdString()<<std::endl;
        std::cerr<<p->errorString().toStdString()<<std::endl;
    }else{
        Pipe::error("Unknown Error");
    }
}

void MainWindow::view(QWidget*w)
{
    emit closeView();
    w->setAttribute(Qt::WA_DeleteOnClose,true);
    connect(this,SIGNAL(closeView()),w,SLOT(close()));
    connect(w,SIGNAL(destroyed(QObject*)),this,SLOT(informViewClosed(QObject*)));
    ui->gridLayout->addWidget(w);
}

void MainWindow::viewOverSeg(void)
{
    OverSegView* w = new OverSegView(NULL);
    w->setObjectName(QString("OversegView"));
    view((QWidget*)w);
}

void MainWindow::informViewClosed(QObject* o)
{
    Pipe::inform("View Closed:");
    std::cerr<<o->objectName().toStdString()<<std::endl;
}
