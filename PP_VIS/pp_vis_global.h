#ifndef PP_VIS_GLOBAL_H
#define PP_VIS_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(PP_VIS_LIBRARY)
#  define PP_VISSHARED_EXPORT Q_DECL_EXPORT
#else
#  define PP_VISSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // PP_VIS_GLOBAL_H
