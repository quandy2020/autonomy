"""Source inventories for Autoviz — kept separate from BUILD.bazel for clarity.

Layers mirror the on-disk tree. They are concatenated into a single
`cc_library` because `common ↔ ui` and `rendering ↔ display` still share
headers (same constraint as the CMake SHARED target).
"""

# Retired / app-only units excluded from libautoviz.
EXCLUDES = [
    "autoviz/main.cpp",
    "autoviz/rendering/render_window.cpp",
    "autoviz/rendering/render_window.hpp",
    "**/test/**",
    "**/testing/**",
]

# Headers that typically need moc (Q_OBJECT). Broad ui/ + known Qt hubs;
# qt.bzl stubs empty output when a header has no meta-object classes.
MOC_GLOBS = [
    "autoviz/ui/**/*.hpp",
    "autoviz/tools/**/*.hpp",
    "autoviz/common/visualization_manager.hpp",
    "autoviz/common/display_*.hpp",
    "autoviz/common/tool*.hpp",
    "autoviz/common/frame_*.hpp",
    "autoviz/common/view_*.hpp",
    "autoviz/common/selection_*.hpp",
    "autoviz/common/properties/**/*.hpp",
    "autoviz/rendering/ogre_render_window.hpp",
]

QRC_FILES = [
    "resources/autoviz.qrc",
    "styles/styles.qrc",
]

# Logical modules (documentation + future split points).
MODULES = [
    "platform",
    "commsgs",
    "transform",
    "integration",
    "rendering",
    "display",
    "tools",
    "common",
    "ui",
]
