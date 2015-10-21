#ifndef ELM_H
#define ELM_H
#include <iostream>
namespace ELM
{
    typedef struct MatrixMemory{
        int N;
        double* data;
        MatrixMemory():N(0),data(NULL){}
        MatrixMemory(const MatrixMemory&m):N(m.N),data(m.data){}
        MatrixMemory(int n):N(n){data = new double[N];}
        ~MatrixMemory(){delete[] data;}
    }MatrixMemory;

    class OSELM
    {
    public:
        void learnInitial(void);
        void learnSequential(void);
        void test(void);
    private:
        MatrixMemory _InputWeight;
        MatrixMemory _Bias;
        MatrixMemory _Beta;
    };
}

#endif // ELM_H
