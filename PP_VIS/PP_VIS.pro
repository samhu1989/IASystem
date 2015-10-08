#-------------------------------------------------
#
# Project created by QtCreator 2015-05-21T20:39:24
#
#-------------------------------------------------

QT       += widgets opengl

TARGET = PP_VIS
TEMPLATE = lib

DEFINES += PP_VIS_LIBRARY

SOURCES += pp_vis.cpp \
    mainwindow.cpp \
    datalist.cpp \
    datalistmodel.cpp \
    pcdviewer.cpp \
    segview.cpp \
    featureview.cpp \
    patchcharts.cpp \
    icpwidget.cpp \
    geoobj.cpp \
    objview.cpp \
    icpdialog.cpp \
    icpcomputer.cpp

HEADERS += pp_vis.h\
        pp_vis_global.h \
    mainwindow.h \
    datalist.h \
    datalistmodel.h \
    pcdviewer.h \
    segview.h \
    region_growing.h \
    region_growing.hpp \
    featureview.h \
    patchcharts.h \
    icpwidget.h \
    geoobj.h \
    objview.h \
    icpdialog.h \
    icpcomputer.h

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

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/include

win32:CONFIG(release, debug|release): LIBS += -lAdvapi32

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/VTK/lib/vtk-5.8/  \
-lvtkGraphics -lvtkverdict -lvtkImaging  -lvtkIO \
-lvtkCommon -lvtkRendering \
-lvtkpng -lvtktiff -lvtkzlib -lvtkjpeg -lvtkexpat -lvtkftgl -lvtkmetaio  \
-lvtkFiltering -lvtkDICOMParser -lvtkNetCDF_cxx -lvtksqlite \
-lvtkfreetype -lvtkexoIIc -lvtkNetCDF -lvtkHybrid -lQVTK -lvtkWidgets -lvtksys

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Boost/lib/ \
-lboost_system-vc100-mt-1_49 \
-lboost_filesystem-vc100-mt-1_49 \
-lboost_thread-vc100-mt-1_49 \
-lboost_date_time-vc100-mt-1_49 \
-lboost_iostreams-vc100-mt-1_49 \
-lboost_mpi-vc100-mt-1_49 \
-lboost_serialization-vc100-mt-1_49

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/lib/ \
-lpcl_visualization_release -lpcl_apps_release -lpcl_common_release \
-lpcl_octree_release  -lpcl_io_release  -lpcl_kdtree_release \
-lpcl_search_release  -lpcl_sample_consensus_release -lpcl_filters_release \
-lpcl_features_release  -lpcl_segmentation_release \
-lpcl_visualization_release  -lpcl_surface_release -lpcl_registration_release \
-lpcl_keypoints_release -lpcl_tracking_release

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/Qhull/lib/ -lqhullstatic

INCLUDEPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Qhull/include
DEPENDPATH += $$PWD/../../../../../../PCL_1.6.0/3rdParty/Qhull/include

win32: LIBS += -L$$PWD/../../../../../../PCL_1.6.0/3rdParty/FLANN/lib/ -lflann_cpp_s

INCLUDEPATH += $$PWD/../../../../../../opencv/build/include
DEPENDPATH += $$PWD/../../../../../../opencv/build/include

win32: LIBS += -L$$PWD/../../../../../../opencv/build/x86/vc10/lib/ -lopencv_core249

win32: LIBS += -llibcoinglpk -llibClp -llibDecomp -llibCoinUtils -llibcoinlapack -llibcoinblas -llibOS \
-llibOsi -llibOsiClp

FORMS += \
    mainwindow.ui \
    datalist.ui \
    pcdviewer.ui \
    segview.ui \
    featureview.ui \
    patchcharts.ui \
    mapper.ui \
    icpwidget.ui \
    objview.ui \
    icpdialog.ui

RESOURCES += \
    rc.qrc
