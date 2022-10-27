#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define SbsController_DLLIMPORT __declspec(dllimport)
#  define SbsController_DLLEXPORT __declspec(dllexport)
#  define SbsController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define SbsController_DLLIMPORT __attribute__((visibility("default")))
#    define SbsController_DLLEXPORT __attribute__((visibility("default")))
#    define SbsController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define SbsController_DLLIMPORT
#    define SbsController_DLLEXPORT
#    define SbsController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef SbsController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define SbsController_DLLAPI
#  define SbsController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef SbsController_EXPORTS
#    define SbsController_DLLAPI SbsController_DLLEXPORT
#  else
#    define SbsController_DLLAPI SbsController_DLLIMPORT
#  endif // SbsController_EXPORTS
#  define SbsController_LOCAL SbsController_DLLLOCAL
#endif // SbsController_STATIC