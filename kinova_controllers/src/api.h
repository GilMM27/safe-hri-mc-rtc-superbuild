#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define KinovaHoldController_DLLIMPORT __declspec(dllimport)
#  define KinovaHoldController_DLLEXPORT __declspec(dllexport)
#  define KinovaHoldController_DLLLOCAL
#else
#  if __GNUC__ >= 4
#    define KinovaHoldController_DLLIMPORT __attribute__((visibility("default")))
#    define KinovaHoldController_DLLEXPORT __attribute__((visibility("default")))
#    define KinovaHoldController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
#    define KinovaHoldController_DLLIMPORT
#    define KinovaHoldController_DLLEXPORT
#    define KinovaHoldController_DLLLOCAL
#  endif
#endif

#ifdef KinovaHoldController_STATIC
#  define KinovaHoldController_DLLAPI
#else
#  ifdef KinovaHoldController_EXPORTS
#    define KinovaHoldController_DLLAPI KinovaHoldController_DLLEXPORT
#  else
#    define KinovaHoldController_DLLAPI KinovaHoldController_DLLIMPORT
#  endif
#endif
