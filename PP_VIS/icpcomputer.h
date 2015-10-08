#ifndef ICPCOMPUTER_H
#define ICPCOMPUTER_H
#include <QThread>
#include "pipe.h"
class ICPComputer : public QThread
{
public:
    ICPComputer();
    void setCorr(std::vector<int>&pickeda,std::vector<int>&pickedb);
    void setA(FullPointCloud::Ptr a);
    void setB(FullPointCloud::Ptr b);
    void run(void);
    void getT(Eigen::Matrix4f& T);
    ~ICPComputer();
protected:
    void initCenter(void);
    void initManual(void);
private:
    FullPointCloud::Ptr _A;
    std::vector<int> _PickedA;
    FullPointCloud::Ptr _B;
    std::vector<int> _PickedB;
    float _T[16];
};

#endif // ICPCOMPUTER_H
