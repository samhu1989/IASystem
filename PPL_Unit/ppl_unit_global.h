#ifndef PPL_UNIT_GLOBAL_H
#define PPL_UNIT_GLOBAL_H


#if defined(PPL_UNIT_LIBRARY)
#  define PPL_UNITSHARED_EXPORT __declspec( dllexport )
#else
#  define PPL_UNITSHARED_EXPORT __declspec( dllimport )
#endif

#endif // PPL_UNIT_GLOBAL_H
