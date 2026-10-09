"""C++ dependency labels for //autonomy/* (SSOT).

Consumed by:
  - root BUILD.bazel → //:cpp_third_party
  - tools/package.bzl → every autonomy_<domain>
"""

# Linked by every autonomy_<domain> library.
AUTONOMY_THIRD_PARTY_DEPS = [
    # BCR (bazel_dep in MODULE.bazel)
    "@eigen//:eigen",
    "@com_github_google_glog//:glog",
    "@com_google_protobuf//:protobuf",
    # CMake prefix (@autonomy_prefix)
    "//:automsgs",
    "//:autonomy_headers",
    # system
    "//:pthread",
]

# Linked only by domains in AUTONOMY_DOMAIN_EXTRA_DEPS (tools/package.bzl).
AUTONOMY_MIDDLEWARE_DEPS = [
    "//:autolink",
]

# Declared in MODULE.bazel; transitive or test-only (not in every domain):
#   @com_github_gflags_gflags//:gflags
#   @com_google_absl//absl/...
#   @googletest//:gtest_main
