# @qt6 — system Qt6 headers + shared libraries (Ubuntu multiarch layout).

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

# Symlinked tree: include/ → /usr/include/<multiarch>/qt6
#
# Match `pkg-config --cflags Qt6Core Qt6Widgets …`: each module directory must
# be on the include path so `#include <QMap>` resolves via QtCore/QMap.
cc_library(
    name = "qt6",
    hdrs = glob(["include/**"], allow_empty = True),
    includes = [
        "include",
        "include/QtCore",
        "include/QtGui",
        "include/QtWidgets",
        "include/QtOpenGL",
        "include/QtOpenGLWidgets",
        "include/QtXml",
        "include/QtSvg",
        "include/QtNetwork",
    ],
    defines = [
        "QT_CORE_LIB",
        "QT_GUI_LIB",
        "QT_WIDGETS_LIB",
        "QT_OPENGL_LIB",
        "QT_OPENGLWIDGETS_LIB",
        "QT_XML_LIB",
        "QT_SVG_LIB",
        "QT_NETWORK_LIB",
    ],
    linkopts = [
        "-lQt6Core",
        "-lQt6Gui",
        "-lQt6Widgets",
        "-lQt6OpenGL",
        "-lQt6OpenGLWidgets",
        "-lQt6Xml",
        "-lQt6Svg",
        "-lQt6Network",
        "-lGL",
        "-Wl,-rpath,/usr/lib/x86_64-linux-gnu",
    ],
)
