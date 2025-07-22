load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "transform",
    srcs = glob(
    [
        "*.cc",
    ],
    exclude = [
        "*_test.cc",
    ],
    allow_empty = True),
    hdrs = glob([
                "*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            "@eigen",
            "@abseil-cpp//absl/strings",
            "@glog",
            "//openbot/common:common",
            "//openbot/common/utils:common_lua_parameter_dictionary",
            "//openbot/common/math:common_math",
    ],
    copts = ["-Wno-sign-compare", "-std=c++17"],
)
