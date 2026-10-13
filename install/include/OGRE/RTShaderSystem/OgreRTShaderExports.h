
#ifndef _OgreRTSSExport_H
#define _OgreRTSSExport_H

#ifdef OGRERTSHADERSYSTEM_STATIC_DEFINE
#  define _OgreRTSSExport
#  define OGRERTSHADERSYSTEM_NO_EXPORT
#else
#  ifndef _OgreRTSSExport
#    ifdef OgreRTShaderSystem_EXPORTS
        /* We are building this library */
#      define _OgreRTSSExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgreRTSSExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef OGRERTSHADERSYSTEM_NO_EXPORT
#    define OGRERTSHADERSYSTEM_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef OGRERTSHADERSYSTEM_DEPRECATED
#  define OGRERTSHADERSYSTEM_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef OGRERTSHADERSYSTEM_DEPRECATED_EXPORT
#  define OGRERTSHADERSYSTEM_DEPRECATED_EXPORT _OgreRTSSExport OGRERTSHADERSYSTEM_DEPRECATED
#endif

#ifndef OGRERTSHADERSYSTEM_DEPRECATED_NO_EXPORT
#  define OGRERTSHADERSYSTEM_DEPRECATED_NO_EXPORT OGRERTSHADERSYSTEM_NO_EXPORT OGRERTSHADERSYSTEM_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef OGRERTSHADERSYSTEM_NO_DEPRECATED
#    define OGRERTSHADERSYSTEM_NO_DEPRECATED
#  endif
#endif

#endif /* _OgreRTSSExport_H */
