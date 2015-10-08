#include <QCoreApplication>
#include "xkoverseg.h"
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    XKOverSeg overseg;
    overseg.init();
    return a.exec();
}
