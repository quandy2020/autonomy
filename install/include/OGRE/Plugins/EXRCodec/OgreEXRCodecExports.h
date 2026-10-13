
#ifndef _OgreEXRPluginExport_H
#define _OgreEXRPluginExport_H

#ifdef CODEC_EXR_STATIC_DEFINE
#  define _OgreEXRPluginExport
#  define CODEC_EXR_NO_EXPORT
#else
#  ifndef _OgreEXRPluginExport
#    ifdef Codec_EXR_EXPORTS
        /* We are building this library */
#      define _OgreEXRPluginExport __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define _OgreEXRPluginExport __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CODEC_EXR_NO_EXPORT
#    define CODEC_EXR_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CODEC_EXR_DEPRECATED
#  define CODEC_EXR_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CODEC_EXR_DEPRECATED_EXPORT
#  define CODEC_EXR_DEPRECATED_EXPORT _OgreEXRPluginExport CODEC_EXR_DEPRECATED
#endif

#ifndef CODEC_EXR_DEPRECATED_NO_EXPORT
#  define CODEC_EXR_DEPRECATED_NO_EXPORT CODEC_EXR_NO_EXPORT CODEC_EXR_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CODEC_EXR_NO_DEPRECATED
#    define CODEC_EXR_NO_DEPRECATED
#  endif
#endif

#endif /* _OgreEXRPluginExport_H */
