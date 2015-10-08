#include "pp_vis.h"
#include <QDir>
#include <QApplication>
#include "mainwindow.h"

Pipe* createPipe(void)
{
    return new PP_VIS();
}

PP_VIS::PP_VIS()
{
    ;
}

PP_VIS::~PP_VIS()
{
    delete[] _argv[0];
    delete[] _argv;
    inform("Deconstruct PP_VIS");
}

int PP_VIS::configure(Config& config)
{
    return 0;
}

int PP_VIS::init(void)
{
    QDir dir;
    _argc = 1;
    _argv = new char*[1];
    std::string currentpath = dir.current().absolutePath().toStdString();
    char* path = new char[currentpath.size()];
    strcpy(path,currentpath.c_str());
    _argv[0] = path;
    return 0;
}

int PP_VIS::work(void)
{
    QApplication a(_argc,_argv);
    MainWindow w;
    w.show();
    return a.exec();
}

int PP_VIS::saveToData(void)
{
    inform("Should have saved interactively");
    return 0;
}
