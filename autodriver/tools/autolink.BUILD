# @autolink — sibling ../autolink headers + @prefix libautolink.so.

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "autolink",
    hdrs = glob(
        [
            "autolink/**/*.hpp",
            "autolink/**/*.h",
        ],
        exclude = [
            "**/test/**",
            "**/testing/**",
            "**/examples/**",
        ],
        allow_empty = True,
    ),
    includes = ["."],
    linkopts = [
        "/usr/local/lib/libglog.so",
        "/usr/local/lib/libprotobuf.so",
        "-Wl,-rpath,/usr/local/lib",
    ],
    deps = ["@prefix//:autolink_so"],
)
