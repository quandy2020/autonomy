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

# -----------------------------------------------------------------------------
# autonomy/localization/atlas — VO / VIO SLAM offline runners + tests
#
# Library sources are GLOB'd by the parent localization CMakeLists.
# -----------------------------------------------------------------------------

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.localization.atlas_dataset
  NO_EXPORT
  SOURCES test/app/atlas_dataset_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)

autocmake_binary(autonomy.localization.atlas_euroc_mi
  NO_EXPORT
  SOURCES test/app/euroc_mono_inertial_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)

# --- test --------------------------------------------------------------------
if(BUILD_TEST OR BUILD_TESTING)
  foreach(_t IN ITEMS
      tracking_test
      config_test
      lidar_pose_test
      occupancy_test
      lidar_reloc_test
      scan_filter_test)
    autocmake_test(localization.atlas.${_t}
      SOURCES test/unit/${_t}.cpp
      DEPENDENCIES autonomy_localization)
  endforeach()
endif()

# --- install -----------------------------------------------------------------
install(DIRECTORY config/
  DESTINATION share/autonomy/localization/conf/atlas)

message(STATUS
  "atlas: offline runners (atlas_dataset, atlas_euroc_mi) + unit tests + "
  "config install")
