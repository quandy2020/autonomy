# @ogre — prebuilt Ogre 1.12.x *install* prefix (include/OGRE + lib/).

load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_import(
    name = "OgreMain_so",
    shared_library = "prefix/lib/libOgreMain.so",
)

cc_import(
    name = "OgreOverlay_so",
    shared_library = "prefix/lib/libOgreOverlay.so",
)

cc_library(
    name = "ogre",
    hdrs = glob(
        [
            "prefix/include/**",
        ],
        allow_empty = True,
    ),
    includes = [
        "prefix/include",
        "prefix/include/OGRE",
        "prefix/include/OGRE/Overlay",
        "prefix/include/OGRE/Threading",
    ],
    deps = [
        ":OgreMain_so",
        ":OgreOverlay_so",
    ],
)

# Plugins are dlopened at runtime (RenderSystem_GL, Codec_STBI, …).
filegroup(
    name = "plugins",
    srcs = glob(
        [
            "prefix/lib/OGRE/**",
            "prefix/lib/Codec_*.so*",
            "prefix/lib/RenderSystem_*.so*",
            "prefix/lib/Plugin_*.so*",
        ],
        allow_empty = True,
    ),
)
