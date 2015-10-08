#include <QLibrary>
#include <QLibraryInfo>
#include <string>
#include <iostream>
#include "configure.h"
#include "pipe.h"
#include "pipeline.h"
int main(int argc, char *argv[])
{
    if( argc < 2 || argc%2!=0 )
    {
        std::cerr<<"<Usage>:"<<std::endl;
        std::cerr<<argv[0]<<" [Config File] <addtional_key0 addtional_value0>..."<<std::endl;
    }else{
        std::string configName = std::string(argv[1]);
        Config config(configName);
        unsigned int idx;
        for(idx=2;idx<argc;idx+=2)
        {
            std::string key(argv[idx]);
            std::string value(argv[idx+1]);
//            std::cerr<<key<<"->"<<value<<std::endl;
            config.add(key,value);
        }
        Pipe::_Date = Pipe::_Date.fromString("01_01_01","yy_MM_dd");
        Pipe::_Time = Pipe::_Time.fromString("12_00_00","hh_mm_ss");
        if( config.has("PPL_Lib") )
        {
            QLibrary lib(QString::fromStdString(config.getString("PPL_Lib")));
            if(lib.load())
            {
               Pipeline::Fetcher func = (Pipeline::Fetcher)lib.resolve("createPipeline");
               if(func)
               {
                   Pipeline* pipeline = func();
                   pipeline->configure(config);
                   int r = pipeline->exec();
                   delete pipeline;
                   return r;
               }else{
                   Pipe::error(lib.errorString().toStdString());
               }
            }else{
                Pipe::error(lib.errorString().toStdString());
            }
        }
    }
    return -1;
}
