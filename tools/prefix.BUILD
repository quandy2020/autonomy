# @autonomy_prefix — optional CMake install tree.
#
# Middleware APIs: @automsgs / @autolink (BCR protobuf).
# Domain protos:   //:autonomy_headers (generated in-repo).
# Kept so use_repo(autonomy_prefix) remains valid for legacy consumers.

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

alias(
    name = "automsgs",
    actual = "@automsgs//:automsgs",
)

alias(
    name = "autolink",
    actual = "@autolink//:autolink",
)

# Prefer //:autonomy_headers. Stub keeps the label resolvable.
cc_library(name = "autonomy_headers")
