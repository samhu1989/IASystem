#include "pipeline.h"
#include "pipe.h"
#include <QLibrary>
Pipeline::Pipeline()
{

}

Pipeline::~Pipeline()
{

}

Pipe* Pipeline::fetchPipe(const std::string& fileName)
{
    QLibrary lib(QString::fromStdString(fileName));
    if(lib.load())
    {
       Pipe::Fetcher func = (Pipe::Fetcher)lib.resolve("createPipe");
       if(func)
       {
           Pipe* pipe = func();
           return pipe;
       }else{
           Pipe::error(lib.errorString().toStdString());
       }
    }else{
        Pipe::error(lib.errorString().toStdString());
    }
    return NULL;
}

void Pipeline::configure(Config& config)
{

}

int Pipeline::exec(void)
{
    Pipe::warn("Excuting Empty Pipeline");
    return -1;
}
