"""Chassis plugin helper: test library + dlopen-able shared object."""

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

def chassis_plugin(name, srcs, hdrs):
    """Declare `:{name}_lib` (for tests) and `//:autodriver_{name}` (plugin .so).

    The plugin links its own objects and dynamically depends on //:autodriver,
    matching the CMake layout (libautodriver_{name}.so → libautodriver.so).
    """
    cc_library(
        name = name + "_lib",
        srcs = srcs,
        hdrs = hdrs,
        deps = [
            "//:autodriver",
            "@autolink//:autolink",
        ],
    )

    cc_binary(
        name = "autodriver_" + name,
        srcs = srcs,
        linkshared = True,
        linkstatic = False,
        deps = [
            "//:autodriver",
            "@autolink//:autolink",
        ],
    )
