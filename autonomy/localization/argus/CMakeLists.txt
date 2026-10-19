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
#
#   find_package(Argus REQUIRED)
#   target_link_libraries(foo PRIVATE Argus::argus)

cmake_minimum_required(VERSION 3.16)

# Detect embedding under autonomy/localization (add_subdirectory) vs standalone.
set(ARGUS_IS_EMBEDDED FALSE)
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(ARGUS_IS_EMBEDDED TRUE)
endif()

project(argus VERSION 0.1.0 LANGUAGES CXX)

if(TARGET Argus::argus OR TARGET argus)
  return()
endif()

set(ARGUS_VERSION ${PROJECT_VERSION})
set(ARGUS_SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})

# When embedded, parent localization/CMakeLists may pre-set cache defaults
# (install OFF, apps follow BUILD_TOOLS). Standalone keeps historical defaults.
if(ARGUS_IS_EMBEDDED)
  option(ARGUS_BUILD_INSTALL "Generate install / export rules" OFF)
else()
  option(ARGUS_BUILD_INSTALL "Generate install / export rules" ON)
endif()
option(ARGUS_BUILD_SHARED "Build shared libraries" ON)
option(ARGUS_BUILD_TESTS "Build Argus unit tests" OFF)
option(ARGUS_BUILD_APPS "Build offline_livo / livo_esikf_node / eval_pose_traj" ON)
option(ARGUS_USE_ONNXRUNTIME
  "Require ONNX Runtime for SemanticDynamicFilter" OFF)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
argus_initialize_project()

# Prefer already-created targets from the autonomy umbrella (embedded build).
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED)
endif()
if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
endif()
find_package(Threads REQUIRED)
if(NOT TARGET glog::glog)
  if(TARGET glog)
    add_library(glog::glog ALIAS glog)
  else()
    find_package(glog REQUIRED)
    if(TARGET glog AND NOT TARGET glog::glog)
      add_library(glog::glog ALIAS glog)
    endif()
  endif()
endif()
if(NOT TARGET automsgs::automsgs)
  if(TARGET automsgs)
    add_library(automsgs::automsgs ALIAS automsgs)
  else()
    find_package(automsgs REQUIRED)
  endif()
endif()
find_package(OpenMP)
if(NOT OnnxRuntime_FOUND)
  find_package(OnnxRuntime QUIET)
endif()

if(NOT TARGET automsgs::automsgs)
  message(FATAL_ERROR "automsgs found but target automsgs::automsgs missing")
endif()
if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "Eigen3::Eigen target required")
endif()
if(ARGUS_USE_ONNXRUNTIME AND NOT OnnxRuntime_FOUND)
  message(WARNING
    "ARGUS_USE_ONNXRUNTIME=ON but OnnxRuntime not found; "
    "SemanticDynamicFilter stays pass-through / mask_file")
endif()

message(STATUS
  "Argus deps: Eigen3 OpenCV/${OpenCV_VERSION} glog automsgs "
  "OpenMP=${OpenMP_CXX_FOUND} OnnxRuntime=${OnnxRuntime_FOUND} "
  "embedded=${ARGUS_IS_EMBEDDED}")

file(GLOB_RECURSE ALL_LIBRARY_HDRS
  "${PROJECT_SOURCE_DIR}/argus/*.hpp"
  "${PROJECT_SOURCE_DIR}/argus/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS
  "${PROJECT_SOURCE_DIR}/argus/*.cpp"
  "${PROJECT_SOURCE_DIR}/argus/*.cc")

set(BUILD_SHARED_LIBS ${ARGUS_BUILD_SHARED})
add_library(${PROJECT_NAME} ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
add_library(Argus::argus ALIAS ${PROJECT_NAME})

foreach(_comp IN ITEMS
    common sensor camera frontend perception map fusion backend
    loop_closure relocalization mapping odometry)
  add_library(Argus::${_comp} ALIAS ${PROJECT_NAME})
endforeach()

set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${ARGUS_CXX_FLAGS}"
  VERSION ${ARGUS_VERSION}
  SOVERSION ${ARGUS_SOVERSION}
  EXPORT_NAME argus
  OUTPUT_NAME argus
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  Threads::Threads
  glog::glog
  automsgs::automsgs
  $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
)
if(OnnxRuntime_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ARGUS_HAS_ONNXRUNTIME=1)
  target_link_libraries(${PROJECT_NAME} PUBLIC OnnxRuntime::OnnxRuntime)
endif()

if(ARGUS_BUILD_APPS)
  argus_binary(offline_livo SRCS apps/offline_livo.cpp)
  set(_ARGUS_KITTI_DATASET_SRCS apps/datasets/kitti_dataset.cpp)
  set(_ARGUS_OXFORD_DATASET_SRCS
    apps/datasets/pcd_io.cpp
    apps/datasets/oxford_spires_options.cpp
    apps/datasets/oxford_spires_dataset.cpp
    apps/datasets/newer_college_options.cpp
    apps/datasets/newer_college_dataset.cpp)
  argus_binary(offline_kitti
    SRCS apps/offline_kitti.cpp ${_ARGUS_KITTI_DATASET_SRCS})
  argus_binary(slice_kitti
    SRCS apps/slice_kitti.cpp ${_ARGUS_KITTI_DATASET_SRCS})
  argus_binary(verify_kitti
    SRCS apps/verify_kitti.cpp ${_ARGUS_KITTI_DATASET_SRCS})
  argus_binary(verify_oxford_datasets
    SRCS apps/verify_oxford_datasets.cpp ${_ARGUS_OXFORD_DATASET_SRCS})
  argus_binary(offline_spires_livo
    SRCS apps/offline_spires_livo.cpp ${_ARGUS_OXFORD_DATASET_SRCS})
  argus_binary(livo_esikf_node SRCS apps/livo_esikf_node.cpp)
  argus_binary(eval_pose_traj SRCS apps/eval_pose_traj.cpp)
endif()

if(ARGUS_BUILD_TESTS)
  argus_enable_testing()
  file(GLOB ALL_TESTS "${PROJECT_SOURCE_DIR}/test/test_*.cpp")
  foreach(ABS_FIL ${ALL_TESTS})
    get_filename_component(FIL_WE ${ABS_FIL} NAME_WE)
    string(REGEX REPLACE "^test_" "argus_test_" TEST_TARGET_NAME "${FIL_WE}")
    argus_test(${TEST_TARGET_NAME} SRCS ${ABS_FIL})
  endforeach()
endif()

if(ARGUS_BUILD_INSTALL)
  include(CMakePackageConfigHelpers)
  include(GNUInstallDirs)

  install(TARGETS ${PROJECT_NAME}
    EXPORT ArgusTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

  foreach(HDR ${ALL_LIBRARY_HDRS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    install(FILES ${HDR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${DIR})
  endforeach()

  install(DIRECTORY config cmake DESTINATION share/argus)

  set(ARGUS_CMAKE_DIR share/argus/cmake)
  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/ArgusConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/cmake/argus/ArgusConfig.cmake"
    PATH_VARS ARGUS_CMAKE_DIR
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Argus
  )
  write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/ArgusConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
  )

  install(FILES
    "${PROJECT_BINARY_DIR}/cmake/argus/ArgusConfig.cmake"
    "${PROJECT_BINARY_DIR}/ArgusConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Argus
  )
  install(EXPORT ArgusTargets
    FILE ArgusTargets.cmake
    NAMESPACE Argus::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Argus
  )
  export(EXPORT ArgusTargets
    FILE "${PROJECT_BINARY_DIR}/ArgusTargets.cmake"
    NAMESPACE Argus::
  )
endif()

list(LENGTH ALL_LIBRARY_SRCS _argus_src_count)
message(STATUS
  "Argus ${ARGUS_VERSION}: ${_argus_src_count} sources, "
  "shared=${ARGUS_BUILD_SHARED}, tests=${ARGUS_BUILD_TESTS}, "
  "apps=${ARGUS_BUILD_APPS}")
