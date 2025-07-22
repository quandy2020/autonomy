load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "motion_model",
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
            "//openbot/common/helper_functions:helper_functions",
            "//openbot/common/state_vector:state_vector",
            
    ],
)

cc_test(
    name = "differential_motion_model_ut",
    srcs = [
        "differential_motion_model_test.cc",
    ],
    deps = [
        ":motion_model",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "linear_motion_model_ut",
    srcs = [
        "linear_motion_model_test.cc",
    ],
    deps = [
        ":motion_model",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "stationary_motion_model_ut",
    srcs = [
        "stationary_motion_model_test.cc",
    ],
    deps = [
        ":motion_model",
        "@googletest//:gtest_main",
    ],
)