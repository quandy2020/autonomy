# @cli11 — header-only CLI11 (sibling autolink/thirdparty/CLI11/include).

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cli11",
    hdrs = glob(["include/**/*.hpp"], allow_empty = True),
    includes = ["include"],
)
