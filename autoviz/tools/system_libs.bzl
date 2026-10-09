"""Declare a cc_library that only forwards system include/link flags."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def system_library(name, linkopts, includes = None, defines = None):
    """Thin wrapper around cc_library for distro / SDK shared libraries."""
    cc_library(
        name = name,
        includes = includes or [],
        linkopts = linkopts,
        defines = defines or [],
        linkstatic = True,
        visibility = ["//visibility:public"],
    )
