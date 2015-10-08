#-------------------------------------------------
#
# Project created by QtCreator 2015-05-18T23:25:48
#
#-------------------------------------------------

QT       -= gui

TARGET = PP_MultiMergePlane
TEMPLATE = lib

DEFINES += PP_MULTIMERGEPLANE_LIBRARY

SOURCES += pp_multimergeplane.cpp

HEADERS += pp_multimergeplane.h\
        pp_multimergeplane_global.h \
    region_growing.h \
    region_growing.hpp

unix {
    target.path = /usr/lib
    INSTALLS += target
}

CONFIG(release, debug|release): DESTDIR = $$OUT_PWD/../../IASystem_RunTime/bin

win32:CONFIG(release, debug|release): LIBS += -L$$DESTDIR/ -lCommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$DESTDIR/ -lCommon

INCLUDEPATH += $$PWD/../Common
DEPENDPATH += $$PWD/../Common

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/lib/ -lpcl_common_release -lpcl_search_release -lpcl_features_release -lpcl_kdtree_release
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/lib/ -lpcl_common_debug

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/include/pcl-1.6
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/include/pcl-1.6

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-1_49
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ -lboost_date_time-vc100-mt-gd-1_49

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/include

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Eigen/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Eigen/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/lib/ -lpcl_common_release \
-lpcl_search_release -lpcl_features_release -lpcl_kdtree_release -lpcl_filters_release -lpcl_io_release -lpcl_segmentation_release
else:win32:CONFIG(debug, debug|release): LIBS += -lpcl_io_debug

win32: LIBS += -L$$PWD/../../../../../../opencv/build/x86/vc10/lib/ -lopencv_core249

INCLUDEPATH += $$PWD/../../../../../../opencv/build/include
DEPENDPATH += $$PWD/../../../../../../opencv/build/include

win32: LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/lib/ -lflann_cpp_s

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include

