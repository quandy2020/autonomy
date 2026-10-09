"""System-installed C++ libraries as thin cc_library targets."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def system_library(name, linkopts, defines = None):
    cc_library(
        name = name,
        linkopts = linkopts,
        defines = defines or [],
        linkstatic = True,
        alwayslink = True,
        visibility = ["//visibility:public"],
    )
