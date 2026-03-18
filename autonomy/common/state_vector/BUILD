load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "state_vector",
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
            "//openbot/common/helper_functions:helper_functions",      
    ],
)

cc_test(
    name = "generic_state_ut",
    srcs = [
        "generic_state_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":state_vector",
    ],
)

cc_test(
    name = "variables_ut",
    srcs = [
        "variables_test.cpp",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":state_vector",
    ],
)