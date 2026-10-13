
#ifndef _OgreExport_H
#define _OgreExport_H

#ifdef OGREMAIN_STATIC_DEFINE
#  define _OgreExport
#  define _OgrePrivate
#else
#  ifndef _OgreExport
#    ifdef OgreMain_EXPORTS
        /* We are building this library */
#      define _OgreExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgreExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef _OgrePrivate
#    define _OgrePrivate __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef OGRE_DEPRECATED
#  define OGRE_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef OGRE_DEPRECATED_EXPORT
#  define OGRE_DEPRECATED_EXPORT _OgreExport OGRE_DEPRECATED
#endif

#ifndef OGRE_DEPRECATED_NO_EXPORT
#  define OGRE_DEPRECATED_NO_EXPORT _OgrePrivate OGRE_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef OGREMAIN_NO_DEPRECATED
#    define OGREMAIN_NO_DEPRECATED
#  endif
#endif

#endif /* _OgreExport_H */
