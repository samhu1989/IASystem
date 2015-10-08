#-------------------------------------------------
#
# Project created by QtCreator 2015-01-13T20:40:52
#
#-------------------------------------------------

QT       += core opengl printsupport

TARGET = Common
TEMPLATE = lib
DEFINES += COMMON_LIBRARY
QMAKE_CXXFLAGS+= -openmp
QMAKE_LFLAGS +=  -openmp
SOURCES += \
    pipe.cpp \
    configure.cpp \
    pipeline.cpp \
    icp.cpp \
    checkview.cpp \
    gicp6d.cpp \
    histogram.cpp \
    featuredialog.cpp \
    icpmixedinitview.cpp \
    formattools.cpp \
    icpviewer.cpp

HEADERS +=\
        common_global.h \
    pipe.h \
    configure.h \
    pipeline.h \
    icp.h \
    icp.hpp \
    checkview.h \
    bfgs.h \
    gicp6d.h \
    histogram.h \
    featuredialog.h \
    icpmixedinitview.h \
    formattools.h \
    nanoflann.hpp \
    SparseICP.h \
    icpviewer.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

CONFIG(release, debug|release): DESTDIR = $$OUT_PWD/../../IASystem_RunTime/bin

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/include

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Eigen/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Eigen/include

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/include/pcl-1.6
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/include/pcl-1.6

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/VTK/include/vtk-5.8
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/VTK/include/vtk-5.8

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-1_49
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-gd-1_49

INCLUDEPATH += $$PWD/../../../../../../opencv/build/include
DEPENDPATH += $$PWD/../../../../../../opencv/build/include

win32: LIBS += -L$$PWD/../../../../../../opencv/build/x86/vc10/lib/ -lopencv_core249 -lopencv_highgui249 -lopencv_imgproc

win32: LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/lib/ -lflann_cpp_s

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/lib/ -lpcl_common_release \
-lpcl_search_release -lpcl_kdtree_release -lpcl_io_release -lpcl_visualization_release -lpcl_features_release -lpcl_filters_release \
-lpcl_segmentation_release

FORMS += \
    checkview.ui \
    featuredialog.ui \
    icpmixedinitview.ui \
    formattools.ui \
    icpviewer.ui

win32:CONFIG(release, debug|release): LIBS += -lAdvapi32

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/VTK/lib/vtk-5.8/  \
-lvtkGraphics -lvtkverdict -lvtkImaging  -lvtkIO \
-lvtkCommon -lvtkRendering \
-lvtkpng -lvtktiff -lvtkzlib -lvtkjpeg -lvtkexpat -lvtkftgl -lvtkmetaio  \
-lvtkFiltering -lvtkDICOMParser -lvtkNetCDF_cxx -lvtksqlite \
-lvtkfreetype -lvtkexoIIc -lvtkNetCDF -lvtkHybrid -lQVTK -lvtkWidgets -lvtksys

