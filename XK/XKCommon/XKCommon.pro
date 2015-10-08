#-------------------------------------------------
#
# Project created by QtCreator 2015-09-23T16:00:51
#
#-------------------------------------------------

QT       -= gui

TARGET = XKCommon
TEMPLATE = lib

DEFINES += XKCOMMON_LIBRARY

SOURCES += xkcommon.cpp

HEADERS += xkcommon.h\
        xkcommon_global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
