"""C++ dependency labels for //autonomy/* (SSOT).

Library set mirrors ``scripts/install_deps`` (thirdparty.json + apt cmake_libs).
Versions live in MODULE.bazel (Bzlmod / BCR). This file only lists labels.

Consumed by:
  - root BUILD.bazel → //:cpp_third_party
  - tools/package.bzl → every autonomy_<domain>
  - domain BUILD files that need grpc / ceres / …
"""

# Linked by every autonomy_<domain> library (install_deps core + apt eigen).
AUTONOMY_THIRD_PARTY_DEPS = [
    # BCR ← install_deps / apt
    "@eigen//:eigen",  # apt libeigen3-dev
    "@com_github_google_glog//:glog",  # install_glog.sh
    "@com_google_protobuf//:protobuf",  # BCR 30.2 (Bazel); CMake still 3.19
    "@yaml-cpp//:yaml-cpp",  # apt libyaml-cpp-dev
    "@nlohmann_json//:json",  # install_nlohmann.sh
    # Middleware headers / libs
    "//:automsgs",  # @automsgs source (BCR protobuf)
    "//:autonomy_headers",  # autonomy/**/*.proto via BCR protoc
    # system
    "//:pthread",
]

# Linked only by domains in AUTONOMY_DOMAIN_EXTRA_DEPS (tools/package.bzl).
AUTONOMY_MIDDLEWARE_DEPS = [
    "//:autolink",
]

# Bridge / async_grpc ← install_grpc.sh
AUTONOMY_GRPC_DEPS = [
    "@com_github_grpc_grpc//:grpc++",
    "@com_github_grpc_grpc//:grpc++_reflection",
]

# Common / optimization ← install_ceres_solver.sh
AUTONOMY_CERES_DEPS = [
    "@ceres-solver//:ceres",
]

# Task / BT ← install_behaviortree_cpp.sh
AUTONOMY_BEHAVIORTREE_DEPS = [
    "@behaviortree_cpp//:behaviortree_cpp",
]

# Perception / map / localization (install_opencv.sh → system prefix).
AUTONOMY_OPENCV_DEPS = [
    "//:opencv",
]

# Point-cloud stack (apt libpcl-dev / BCR). Prefer fine-grained @pcl//:… in BUILD.
AUTONOMY_PCL_DEPS = [
    "@pcl//:common",
]

# Optional / domain-specific (see MODULE.bazel inventory):
#   @osqp//:osqp  @gperftools//:tcmalloc  @fastdds//:fastdds  @assimp//:assimp
#   @onetbb//:tbb  @flann//:flann  @lua//:lua  @tinyxml2//:tinyxml2  @libzmq//:libzmq
#   @sqlite3//:sqlite3
#
# Still not bazel_dep (no BCR): g2o, fbow, taskflow, adolc, ipopt, ogre.
