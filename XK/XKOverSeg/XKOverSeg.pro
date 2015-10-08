#-------------------------------------------------
#
# Project created by QtCreator 2015-09-23T11:37:40
#
#-------------------------------------------------

QT       += core network

QT       -= gui

TARGET = XKOverSeg
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

CONFIG(release, debug|release): DESTDIR = $$OUT_PWD/../../../IASystem_RunTime/bin

SOURCES += main.cpp \
    xkoverseg.cpp

HEADERS += \
    octree/boost.h \
    octree/octree_pointcloud_adjacency.h \
    octree/octree_pointcloud_adjacency_container.h \
    octree/octree_pointcloud.h \
    octree/octree_base.h \
    octree/octree_container.h \
    octree/octree_iterator.h \
    octree/octree_key.h \
    octree/octree_nodes.h \
    octree/impl/octree_base.hpp \
    octree/impl/octree_iterator.hpp \
    octree/impl/octree_pointcloud.hpp \
    octree/impl/octree_pointcloud_adjacency.hpp \
    octree/impl/octree_search.hpp\
    xkoverseg.h \
    supervoxel_clustering.h \
    supervoxel_clustering.hpp \
    octree/octree_search.h

win32:CONFIG(release, debug|release): LIBS += -L$$DESTDIR/ -lCommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$DESTDIR/ -lCommon

INCLUDEPATH += $$PWD/../../Common
DEPENDPATH += $$PWD/../../Common

INCLUDEPATH += $$PWD/../../../../../../../opencv/build/include
DEPENDPATH += $$PWD/../../../../../../../opencv/build/include

win32: LIBS += -L$$PWD/../../../../../../../opencv/build/x86/vc10/lib/ -lopencv_core249 -lopencv_highgui249 -lopencv_imgproc

INCLUDEPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/Boost/include
DEPENDPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/Boost/include

INCLUDEPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/Eigen/include
DEPENDPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/Eigen/include

INCLUDEPATH += $$PWD/../../../../../../../PCL_1.6.0/include/pcl-1.6
DEPENDPATH += $$PWD/../../../../../../../PCL_1.6.0/include/pcl-1.6

INCLUDEPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/FLANN/include
DEPENDPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/FLANN/include

INCLUDEPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/VTK/include/vtk-5.8
DEPENDPATH += $$PWD/../../../../../../../PCL_1.6.0/3rdParty/VTK/include/vtk-5.8

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-1_49
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-gd-1_49

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../PCL_1.6.0/lib/ -lpcl_common_release \
-lpcl_search_release -lpcl_kdtree_release -lpcl_io_release -lpcl_visualization_release -lpcl_features_release -lpcl_filters_release \
-lpcl_segmentation_release
