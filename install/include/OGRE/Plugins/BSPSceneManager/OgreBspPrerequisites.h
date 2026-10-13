
#ifndef _OgreBspPluginExport_H
#define _OgreBspPluginExport_H

#ifdef PLUGIN_BSPSCENEMANAGER_STATIC_DEFINE
#  define _OgreBspPluginExport
#  define PLUGIN_BSPSCENEMANAGER_NO_EXPORT
#else
#  ifndef _OgreBspPluginExport
#    ifdef Plugin_BSPSceneManager_EXPORTS
        /* We are building this library */
#      define _OgreBspPluginExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgreBspPluginExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef PLUGIN_BSPSCENEMANAGER_NO_EXPORT
#    define PLUGIN_BSPSCENEMANAGER_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef PLUGIN_BSPSCENEMANAGER_DEPRECATED
#  define PLUGIN_BSPSCENEMANAGER_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef PLUGIN_BSPSCENEMANAGER_DEPRECATED_EXPORT
#  define PLUGIN_BSPSCENEMANAGER_DEPRECATED_EXPORT _OgreBspPluginExport PLUGIN_BSPSCENEMANAGER_DEPRECATED
#endif

#ifndef PLUGIN_BSPSCENEMANAGER_DEPRECATED_NO_EXPORT
#  define PLUGIN_BSPSCENEMANAGER_DEPRECATED_NO_EXPORT PLUGIN_BSPSCENEMANAGER_NO_EXPORT PLUGIN_BSPSCENEMANAGER_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef PLUGIN_BSPSCENEMANAGER_NO_DEPRECATED
#    define PLUGIN_BSPSCENEMANAGER_NO_DEPRECATED
#  endif
#endif

#endif /* _OgreBspPluginExport_H */
