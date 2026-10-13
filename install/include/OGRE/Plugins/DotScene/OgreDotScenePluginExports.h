
#ifndef _OgreDotScenePluginExport_H
#define _OgreDotScenePluginExport_H

#ifdef PLUGIN_DOTSCENE_STATIC_DEFINE
#  define _OgreDotScenePluginExport
#  define PLUGIN_DOTSCENE_NO_EXPORT
#else
#  ifndef _OgreDotScenePluginExport
#    ifdef Plugin_DotScene_EXPORTS
        /* We are building this library */
#      define _OgreDotScenePluginExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgreDotScenePluginExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef PLUGIN_DOTSCENE_NO_EXPORT
#    define PLUGIN_DOTSCENE_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef PLUGIN_DOTSCENE_DEPRECATED
#  define PLUGIN_DOTSCENE_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef PLUGIN_DOTSCENE_DEPRECATED_EXPORT
#  define PLUGIN_DOTSCENE_DEPRECATED_EXPORT _OgreDotScenePluginExport PLUGIN_DOTSCENE_DEPRECATED
#endif

#ifndef PLUGIN_DOTSCENE_DEPRECATED_NO_EXPORT
#  define PLUGIN_DOTSCENE_DEPRECATED_NO_EXPORT PLUGIN_DOTSCENE_NO_EXPORT PLUGIN_DOTSCENE_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef PLUGIN_DOTSCENE_NO_DEPRECATED
#    define PLUGIN_DOTSCENE_NO_DEPRECATED
#  endif
#endif

#endif /* _OgreDotScenePluginExport_H */
