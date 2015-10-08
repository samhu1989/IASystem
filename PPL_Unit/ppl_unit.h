#ifndef PPL_UNIT_H
#define PPL_UNIT_H

#include "ppl_unit_global.h"
#include "pipeline.h"
#include "configure.h"
#include "pipe.h"
class PPL_UNITSHARED_EXPORT PPL_Unit: public Pipeline
{
public:
    PPL_Unit();
    void configure(Config&);
    int exec();
private:
    Pipe* p;
    bool isDumping;
    bool isBroke;
    std::string _DumpPath;
};

extern "C" PPL_UNITSHARED_EXPORT Pipeline* createPipeline(void);

#endif // PPL_UNIT_H
