#ifndef PIPELINE_H
#define PIPELINE_H
#include "common_global.h"
#include "configure.h"
#include "pipe.h"
class COMMONSHARED_EXPORT Pipeline
{
public:
    typedef Pipeline*(*Fetcher)(void);
    Pipeline();
    ~Pipeline();

    static Pipe* fetchPipe(const std::string&);

    virtual void configure(Config&);
    virtual int exec(void);

};

#endif // PIPELINE_H
