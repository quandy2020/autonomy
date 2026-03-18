#pragma once

// Visibility control macros for aviz rendering components.
//
// This follows the common pattern used in ROS and other C++ projects to
// control symbol visibility when building shared libraries. For our usage
// in the aviz executable, these mainly serve as documentation and can be
// safely defined to default visibility on GCC/Clang and left empty on
// other toolchains.

#if defined _WIN32 || defined __CYGWIN__

#ifdef AVIZ_RENDERING_BUILDING_LIBRARY
#ifdef __GNUC__
#define AVIZ_RENDERING_PUBLIC __attribute__((dllexport))
#else
#define AVIZ_RENDERING_PUBLIC __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define AVIZ_RENDERING_PUBLIC __attribute__((dllimport))
#else
#define AVIZ_RENDERING_PUBLIC __declspec(dllimport)
#endif
#endif

#define AVIZ_RENDERING_LOCAL

#else  // non-Windows (GCC/Clang)

#if __GNUC__ >= 4
#define AVIZ_RENDERING_PUBLIC __attribute__((visibility("default")))
#define AVIZ_RENDERING_LOCAL __attribute__((visibility("hidden")))
#else
#define AVIZ_RENDERING_PUBLIC
#define AVIZ_RENDERING_LOCAL
#endif

#endif
