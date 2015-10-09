#-------------------------------------------------
#
# Project created by QtCreator 2015-09-23T11:07:51
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = XKMonitor
TEMPLATE = app
CONFIG   += console
CONFIG   -= app_bundle
CONFIG(release, debug|release): DESTDIR = $$OUT_PWD/../../../IASystem_RunTime/bin

SOURCES += main.cpp\
        mainwindow.cpp \
    oversegview.cpp

HEADERS  += mainwindow.h \
    oversegview.h

FORMS    += mainwindow.ui \
    oversegview.ui

RESOURCES += \
    rc.qrc

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


