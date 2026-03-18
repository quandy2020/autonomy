load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "state_estimation",
    srcs = glob(
    [
        "*.cc",
        "**/*.cc",
    ],
    exclude = [
        "*_test.cc",
        "**/*_test.cc",
    ],
    allow_empty = True),
    hdrs = glob([
                "*.h*",
                "**/*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            "@eigen",
            "@glog",
            "//openbot/common/helper_functions:helper_functions",
            "//openbot/common/io:common_io",
            "//openbot/common/motion_model:motion_model",
            "//openbot/common/state_vector:state_vector",
    ],
)

cc_test(
    name = "kalman_filter_ut",
    srcs = [
        "kalman_filter/kalman_filter_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":state_estimation",
    ],
)

cc_test(
    name = "linear_measurement_ut",
    srcs = [
        "measurement/linear_measurement_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":state_estimation",
    ],
)

cc_test(
    name = "uniform_noise_ut",
    srcs = [
        "noise_model/uniform_noise_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":state_estimation",
    ],
)

cc_test(
    name = "wiener_noise_ut",
    srcs = [
        "noise_model/wiener_noise_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":state_estimation",
    ],
)