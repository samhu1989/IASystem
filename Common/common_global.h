#ifndef COMMON_GLOBAL_H
#define COMMON_GLOBAL_H

#if defined(COMMON_LIBRARY)
#  define COMMONSHARED_EXPORT __declspec( dllexport )
#else
#  define COMMONSHARED_EXPORT __declspec( dllimport )
#endif

#endif // COMMON_GLOBAL_H
