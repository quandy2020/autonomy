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

# Atlas — VO / VIO SLAM (ORB-SLAM3-style frontend + Ceres backend).
#
# Library sources (*.cpp under common/ sensor/ frontend/ map/ backend/ system/)
# are picked up by the parent autonomy/localization via autonomy_glob_srcs().
# Header-only trees (e.g. backend/cost_functions/) need no listing here.
#
# This file registers offline runners, unit tests, and config install.

cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# Offline evaluation binaries (test/app)
# ---------------------------------------------------------------------------

# Unified dataset runner: EuRoC / TUM RGB-D / KITTI · VO / VIO / stereo / RGB-D
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/app/atlas_dataset_main.cpp")
  autonomy_binary(autonomy.localization.atlas_dataset
    SRCS test/app/atlas_dataset_main.cpp)
endif()

# Legacy EuRoC mono-inertial entry (kept for existing scripts / flags)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/app/euroc_mono_inertial_main.cpp")
  autonomy_binary(autonomy.localization.atlas_euroc_mi
    SRCS test/app/euroc_mono_inertial_main.cpp)
endif()

# ---------------------------------------------------------------------------
# Unit tests (test/unit)
# ---------------------------------------------------------------------------

if(BUILD_TEST)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/tracking_test.cpp")
    autonomy_test(localization.atlas.tracking_test
      test/unit/tracking_test.cpp autonomy_localization)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/config_test.cpp")
    autonomy_test(localization.atlas.config_test
      test/unit/config_test.cpp autonomy_localization)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/lidar_pose_test.cpp")
    autonomy_test(localization.atlas.lidar_pose_test
      test/unit/lidar_pose_test.cpp autonomy_localization)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/occupancy_test.cpp")
    autonomy_test(localization.atlas.occupancy_test
      test/unit/occupancy_test.cpp autonomy_localization)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/lidar_reloc_test.cpp")
    autonomy_test(localization.atlas.lidar_reloc_test
      test/unit/lidar_reloc_test.cpp autonomy_localization)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/scan_filter_test.cpp")
    autonomy_test(localization.atlas.scan_filter_test
      test/unit/scan_filter_test.cpp autonomy_localization)
  endif()
endif()

# ---------------------------------------------------------------------------
# Install runtime configs → share/autonomy/localization/conf/atlas/
# ---------------------------------------------------------------------------

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config/
    DESTINATION share/autonomy/localization/conf/atlas
  )
endif()

message(STATUS
  "atlas: offline runners (atlas_dataset, atlas_euroc_mi) + unit tests + "
  "config install")
