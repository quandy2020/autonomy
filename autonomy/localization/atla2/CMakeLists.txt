# Copyright 2026 The Openbot Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Atla2 — unified VO / VIO / LIO / LIVO (apps + install assets)
#
# Library sources under this tree are picked up by the parent
# autonomy/localization via autonomy_glob_srcs(). This file only registers
# executables, installs configs, and documents third-party policy.

cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# Apps
# ---------------------------------------------------------------------------

# Offline synthetic / dataset runner
autonomy_binary(autonomy.localization.atla2_offline
  SRCS apps/offline_runner/offline_runner_main.cpp)

# Online process entry (sensor push / poll loop; ROS-free core)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps/air_slam_node/air_slam_node_main.cpp")
  autonomy_binary(autonomy.localization.atla2_node
    SRCS apps/air_slam_node/air_slam_node_main.cpp)
endif()

# Micro-benchmark (timing / step throughput)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps/benchmark/benchmark_main.cpp")
  autonomy_binary(autonomy.localization.atla2_benchmark
    SRCS apps/benchmark/benchmark_main.cpp)
endif()

# Calibration CLIs (stubs until full solvers land)
foreach(_calib IN ITEMS
    camera_imu_calib
    lidar_imu_calib
    camera_lidar_calib
    joint_calib)
  set(_calib_src
    "${CMAKE_CURRENT_SOURCE_DIR}/apps/calibration_tool/${_calib}/${_calib}_main.cpp")
  if(EXISTS "${_calib_src}")
    autonomy_binary(autonomy.localization.atla2_${_calib}
      SRCS apps/calibration_tool/${_calib}/${_calib}_main.cpp)
  endif()
endforeach()

# ---------------------------------------------------------------------------
# Install runtime configs → share/autonomy/localization/conf/atla2/
# ---------------------------------------------------------------------------
install(
  DIRECTORY config/
  DESTINATION share/autonomy/localization/conf/atla2
)

# ---------------------------------------------------------------------------
# Protobuf message definitions (optional codegen)
# ---------------------------------------------------------------------------
add_subdirectory(proto)

# ---------------------------------------------------------------------------
# Third-party policy (prefer system / workspace packages)
# ---------------------------------------------------------------------------
# Do not vendor Eigen / Ceres / OpenCV / Sophus / g2o / GTSAM here unless a
# pinned fork is required. Parent FEATURES (slam) already links Ceres/Eigen/
# OpenCV via autonomy_link_feature. See thirdparty/README.md.

message(STATUS "atla2: apps + config install registered")
