#ifndef PP_VIS_H
#define PP_VIS_H
#include "pp_vis_global.h"
#include <string>
#include "pipe.h"
class PP_VISSHARED_EXPORT PP_VIS:public Pipe
{

public:
    PP_VIS();
    virtual ~PP_VIS();

    std::string name(){return "PP_VIS";}
    __int32 version(){return 0;}

    int configure(Config&);
    int init(void);
    int work(void);
    int saveToData(void);

protected:

private:

    int _argc;
    char** _argv;
};
extern "C" PP_VISSHARED_EXPORT Pipe* createPipe(void);
#endif // PP_VIS_H
