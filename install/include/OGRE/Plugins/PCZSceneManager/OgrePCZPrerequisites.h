
#ifndef _OgrePCZPluginExport_H
#define _OgrePCZPluginExport_H

#ifdef PLUGIN_PCZSCENEMANAGER_STATIC_DEFINE
#  define _OgrePCZPluginExport
#  define PLUGIN_PCZSCENEMANAGER_NO_EXPORT
#else
#  ifndef _OgrePCZPluginExport
#    ifdef Plugin_PCZSceneManager_EXPORTS
        /* We are building this library */
#      define _OgrePCZPluginExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgrePCZPluginExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef PLUGIN_PCZSCENEMANAGER_NO_EXPORT
#    define PLUGIN_PCZSCENEMANAGER_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef PLUGIN_PCZSCENEMANAGER_DEPRECATED
#  define PLUGIN_PCZSCENEMANAGER_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef PLUGIN_PCZSCENEMANAGER_DEPRECATED_EXPORT
#  define PLUGIN_PCZSCENEMANAGER_DEPRECATED_EXPORT _OgrePCZPluginExport PLUGIN_PCZSCENEMANAGER_DEPRECATED
#endif

#ifndef PLUGIN_PCZSCENEMANAGER_DEPRECATED_NO_EXPORT
#  define PLUGIN_PCZSCENEMANAGER_DEPRECATED_NO_EXPORT PLUGIN_PCZSCENEMANAGER_NO_EXPORT PLUGIN_PCZSCENEMANAGER_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef PLUGIN_PCZSCENEMANAGER_NO_DEPRECATED
#    define PLUGIN_PCZSCENEMANAGER_NO_DEPRECATED
#  endif
#endif

#endif /* _OgrePCZPluginExport_H */
