# @autonomy_prefix — CMake install/build tree.
#
# Provides installed middleware + any installed autonomy headers
# (including CMake-generated ``*.pb.h`` under include/autonomy/).

load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_import(
    name = "autolink_so",
    shared_library = "prefix/lib/libautolink.so",
)

cc_import(
    name = "automsgs_so",
    shared_library = "prefix/lib/libautomsgs.so",
)

cc_library(
    name = "automsgs_headers",
    hdrs = glob(["prefix/include/automsgs/**"], allow_empty = True),
    includes = ["prefix/include"],
)

cc_library(
    name = "autolink_headers",
    hdrs = glob(["prefix/include/autolink/**"], allow_empty = True),
    includes = ["prefix/include"],
)

# CMake-generated / installed autonomy public headers (proto, config, …).
cc_library(
    name = "autonomy_headers",
    hdrs = glob(["prefix/include/autonomy/**"], allow_empty = True),
    includes = ["prefix/include"],
)

cc_library(
    name = "automsgs",
    deps = [
        ":automsgs_headers",
        ":automsgs_so",
    ],
)

cc_library(
    name = "autolink",
    deps = [
        ":autolink_headers",
        ":autolink_so",
        ":automsgs",
    ],
)
