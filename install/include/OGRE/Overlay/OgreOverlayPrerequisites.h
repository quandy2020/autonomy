
#ifndef _OgreOverlayExport_H
#define _OgreOverlayExport_H

#ifdef OGREOVERLAY_STATIC_DEFINE
#  define _OgreOverlayExport
#  define OGREOVERLAY_NO_EXPORT
#else
#  ifndef _OgreOverlayExport
#    ifdef OgreOverlay_EXPORTS
        /* We are building this library */
#      define _OgreOverlayExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgreOverlayExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef OGREOVERLAY_NO_EXPORT
#    define OGREOVERLAY_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef OGREOVERLAY_DEPRECATED
#  define OGREOVERLAY_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef OGREOVERLAY_DEPRECATED_EXPORT
#  define OGREOVERLAY_DEPRECATED_EXPORT _OgreOverlayExport OGREOVERLAY_DEPRECATED
#endif

#ifndef OGREOVERLAY_DEPRECATED_NO_EXPORT
#  define OGREOVERLAY_DEPRECATED_NO_EXPORT OGREOVERLAY_NO_EXPORT OGREOVERLAY_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef OGREOVERLAY_NO_DEPRECATED
#    define OGREOVERLAY_NO_DEPRECATED
#  endif
#endif

#endif /* _OgreOverlayExport_H */
