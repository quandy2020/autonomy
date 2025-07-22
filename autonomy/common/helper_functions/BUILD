load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "helper_functions",
    srcs = glob([
        "*.cc",
    ], allow_empty = True),
    hdrs = glob([
                "*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            "@eigen",
    ],
)
