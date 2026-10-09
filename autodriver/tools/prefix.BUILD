# @prefix — CMake install/build tree (libautolink, libautomsgs, headers).

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
    name = "automsgs",
    hdrs = glob(["prefix/include/automsgs/**"], allow_empty = True),
    includes = ["prefix/include"],
    deps = [":automsgs_so"],
)
