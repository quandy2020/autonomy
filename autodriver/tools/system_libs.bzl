"""Thin wrappers around system-installed C++ libraries."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def system_library(name, linkopts, includes = None, defines = None, visibility = None):
    """Declare a cc_library that only adds include/link flags for a system package."""
    cc_library(
        name = name,
        srcs = [],
        linkopts = linkopts,
        includes = includes or [],
        defines = defines or [],
        linkstatic = True,
        alwayslink = True,
        visibility = visibility or ["//visibility:public"],
    )
