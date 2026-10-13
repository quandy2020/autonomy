
#ifndef _OgreOctreeZonePluginExport_H
#define _OgreOctreeZonePluginExport_H

#ifdef PLUGIN_OCTREEZONE_STATIC_DEFINE
#  define _OgreOctreeZonePluginExport
#  define PLUGIN_OCTREEZONE_NO_EXPORT
#else
#  ifndef _OgreOctreeZonePluginExport
#    ifdef Plugin_OctreeZone_EXPORTS
        /* We are building this library */
#      define _OgreOctreeZonePluginExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgreOctreeZonePluginExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef PLUGIN_OCTREEZONE_NO_EXPORT
#    define PLUGIN_OCTREEZONE_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef PLUGIN_OCTREEZONE_DEPRECATED
#  define PLUGIN_OCTREEZONE_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef PLUGIN_OCTREEZONE_DEPRECATED_EXPORT
#  define PLUGIN_OCTREEZONE_DEPRECATED_EXPORT _OgreOctreeZonePluginExport PLUGIN_OCTREEZONE_DEPRECATED
#endif

#ifndef PLUGIN_OCTREEZONE_DEPRECATED_NO_EXPORT
#  define PLUGIN_OCTREEZONE_DEPRECATED_NO_EXPORT PLUGIN_OCTREEZONE_NO_EXPORT PLUGIN_OCTREEZONE_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef PLUGIN_OCTREEZONE_NO_DEPRECATED
#    define PLUGIN_OCTREEZONE_NO_DEPRECATED
#  endif
#endif

#endif /* _OgreOctreeZonePluginExport_H */
