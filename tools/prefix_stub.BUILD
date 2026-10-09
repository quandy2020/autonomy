# Stub when AUTONOMY_PREFIX is unset (targets exist for analysis only).

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(name = "automsgs")

cc_library(
    name = "autolink",
    deps = [":automsgs"],
)

cc_library(name = "autonomy_headers")
