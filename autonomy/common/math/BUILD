load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "common_math",
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
            "@abseil-cpp//absl/strings:strings",
            "@abseil-cpp//absl/strings:str_format",
            "//openbot/common/utils:common_logging",
            "//openbot/common/utils:common_string_util",
            "//openbot/common:common",        
            "@glog",
    ],
    copts = ["-Wno-sign-compare"],
)

cc_test(
    name = "aabox2d_ut",
    srcs = [
        "aabox2d_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "aaboxkdtree2d_ut",
    srcs = [
        "aaboxkdtree2d_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "angle_ut",
    srcs = [
        "angle_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "box2d_ut",
    srcs = [
        "box2d_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "euler_angles_zxy_ut",
    srcs = [
        "euler_angles_zxy_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "line_segment2d_ut",
    srcs = [
        "line_segment2d_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "math_ut",
    srcs = [
        "math_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "matrix_operations_ut",
    srcs = [
        "matrix_operations_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "matrix_ut",
    srcs = [
        "matrix_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "polygon2d_ut",
    srcs = [
        "polygon2d_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "polynomial_ut",
    srcs = [
        "polynomial_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)

cc_test(
    name = "random_ut",
    srcs = [
        "random_test.cc",
    ],
    deps = [
        "@googletest//:gtest_main",
        ":common_math",
    ],
)
