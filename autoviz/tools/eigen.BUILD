# @eigen — system Eigen3 (/usr/include/eigen3).

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "eigen",
    hdrs = glob(["eigen3/**"], allow_empty = True),
    includes = ["eigen3"],
)
