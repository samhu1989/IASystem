#include "ppl_unit.h"
#include "pipeline.h"
#include "configure.h"

Pipeline* createPipeline(void)
{
    return new PPL_Unit();
}

PPL_Unit::PPL_Unit()
{
    p = NULL;
    isBroke = false;
    isDumping = false;
    _DumpPath = "";
}

void PPL_Unit::configure(Config &config)
{
    if(config.has("PP0_Lib"))
    {
        p = fetchPipe(config.getString("PP0_Lib"));
        if(p)
        {
            if(0!=p->configure(config)){isBroke = true;}
        }else{
            isBroke = true;
        }
    }else{
        NECESSARY_CONFIG();
    }

    if(config.has("PP0_Dump"))
    {
        if(config.getInt("PP0_Dump"))
        {
            isDumping = true;
        }
    }

    if(config.has("PP0_Dump_Path"))
    {
        _DumpPath = config.getString("PP0_Dump_Path");
    }

}

int PPL_Unit::exec(void)
{
    if(!isBroke)
    {
        if(0==p->init())
        if(0==p->work())
        if(0==p->saveToData())
        if(isDumping)
        {
            if(_DumpPath.empty())
            {
                p->dump();
            }else{
                p->dumpTo(_DumpPath);
            }
        }
        delete p;
        return 0;
    }
    return -1;
}
