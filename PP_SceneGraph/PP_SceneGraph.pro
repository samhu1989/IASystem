#-------------------------------------------------
#
# Project created by QtCreator 2015-04-26T19:48:35
#
#-------------------------------------------------

QT       += gui

TARGET = PP_SceneGraph
TEMPLATE = lib

DEFINES += PP_SCENEGRAPH_LIBRARY

SOURCES += pp_scenegraph.cpp

HEADERS += pp_scenegraph.h\
        pp_scenegraph_global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

CONFIG(release, debug|release): DESTDIR = $$OUT_PWD/../../IASystem_RunTime/bin

win32:CONFIG(release, debug|release): LIBS += -L$$DESTDIR/ -lCommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$DESTDIR/ -lCommon

INCLUDEPATH += $$PWD/../Common
DEPENDPATH += $$PWD/../Common

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/include

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Eigen/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Eigen/include

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/include/pcl-1.6
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/include/pcl-1.6

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/VTK/include/vtk-5.8
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/VTK/include/vtk-5.8

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-1_49
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-gd-1_49

INCLUDEPATH += $$PWD/../../../../../../opencv/build/include
DEPENDPATH += $$PWD/../../../../../../opencv/build/include

win32: LIBS += -L$$PWD/../../../../../../opencv/build/x86/vc10/lib/ -lopencv_core249 \
-lopencv_highgui -lopencv_imgproc

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/lib/ -lpcl_common_release \
-lpcl_filters_release -lpcl_search_release -lpcl_features_release -lpcl_kdtree_release -lpcl_io_release -lpcl_segmentation_release \
-lpcl_surface_release
win32: LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/lib/ -lflann_cpp_s

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include
