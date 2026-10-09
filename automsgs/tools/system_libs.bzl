"""Declare a cc_library that only forwards system include/link flags."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def system_library(name, linkopts, defines = None):
    cc_library(
        name = name,
        linkopts = linkopts,
        defines = defines or [],
        linkstatic = True,
        visibility = ["//visibility:public"],
    )
