# Copyright 2025 The Openbot Authors (duyongquan)
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
# autonomy/localization — SLAM / pose estimation (atlas, Cartographer, …)
#
# Builds
#   autonomy_localization / autonomy::localization
#   autonomy.localization
#   atlas/   subdirectory
#
# Toggle
#   AUTONOMY_BUILD_LOCALIZATION
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
autocmake_find(OpenCV REQUIRED PRIVATE)
autocmake_find(PCL REQUIRED PRIVATE
  COMPONENTS common features filters io kdtree segmentation surface)
autocmake_find(TBB REQUIRED PRIVATE)
autocmake_find(OpenMP PRIVATE QUIET)
autocmake_find(LuaGoogle REQUIRED PRIVATE)
autocmake_find(FBow REQUIRED PRIVATE)
autocmake_find(G2o REQUIRED PRIVATE)
autocmake_find(SQLite3 REQUIRED PRIVATE)
autocmake_find(ZLIB REQUIRED PRIVATE)
autocmake_find(PkgConfig REQUIRED PRIVATE)
pkg_check_modules(CAIRO REQUIRED IMPORTED_TARGET cairo)
autocmake_find(opentelemetry-cpp PRIVATE QUIET CONFIG)

foreach(_pkg IN ITEMS OpenCV PCL TBB LuaGoogle FBow G2o SQLite3 ZLIB PkgConfig)
  if(NOT ${_pkg}_FOUND)
    message(FATAL_ERROR "localization: ${_pkg} is required")
  endif()
endforeach()

if(NOT TARGET autonomy_opencv)
  add_library(autonomy_opencv INTERFACE)
  target_include_directories(autonomy_opencv SYSTEM INTERFACE
    $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>)
  target_link_libraries(autonomy_opencv INTERFACE ${OpenCV_LIBS})
endif()

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_LOCALIZATION_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_LOCALIZATION_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_LOCALIZATION_SRCS EXCLUDE REGEX "_test\\.cpp$")
list(FILTER AUTONOMY_LOCALIZATION_SRCS EXCLUDE REGEX "/(fake_|mock_)")
list(FILTER AUTONOMY_LOCALIZATION_SRCS EXCLUDE REGEX "test_helpers")
list(FILTER AUTONOMY_LOCALIZATION_SRCS EXCLUDE REGEX "/atlas/thirdparty/")

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_localization
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_LOCALIZATION_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    autonomy_opencv
    $<TARGET_NAME_IF_EXISTS:Ipopt::Ipopt>
    $<TARGET_NAME_IF_EXISTS:OpenMP::OpenMP_CXX>
    $<TARGET_NAME_IF_EXISTS:FBow::fbow>
    $<TARGET_NAME_IF_EXISTS:PkgConfig::CAIRO>
    ZLIB::ZLIB
    $<TARGET_NAME_IF_EXISTS:BT::behaviortree_cpp>
    $<TARGET_NAME_IF_EXISTS:behaviortree_cpp::behaviortree_cpp>
    $<TARGET_NAME_IF_EXISTS:opentelemetry-cpp::opentelemetry_trace>
    $<TARGET_NAME_IF_EXISTS:CUDA::cudart>
    $<$<BOOL:${BUILD_GRPC}>:grpc++>
    $<$<BOOL:${BUILD_GRPC}>:grpc>
    $<TARGET_NAME_IF_EXISTS:gRPC::grpc++_reflection>)
add_library(autonomy::localization ALIAS autonomy_localization)

target_include_directories(autonomy_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/atlas/port/thirdparty>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lightning>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lightning/thirdparty>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lightning/core/miao>)
target_include_directories(autonomy_localization SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR})
target_link_libraries(autonomy_localization PUBLIC
  ${PCL_LIBRARIES}
  ${SQLite3_LIBRARIES}
  ${LUA_LIBRARIES}
  g2o::core g2o::stuff
  g2o::types_sba g2o::types_sim3 g2o::solver_dense g2o::solver_eigen)
target_compile_definitions(autonomy_localization PUBLIC
  PCL_NO_PRECOMPILE
  ${PCL_DEFINITIONS}
  $<$<BOOL:${OpenMP_CXX_FOUND}>:GRID_MAP_PCL_OPENMP_FOUND=1>
  $<$<TARGET_EXISTS:gRPC::grpc++_reflection>:AUTONOMY_HAVE_GRPC_REFLECTION=1>
  $<$<TARGET_EXISTS:opentelemetry-cpp::opentelemetry_trace>:AUTONOMY_HAVE_OTEL=1>)

# Optional ML backends (found by common).
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  target_compile_definitions(autonomy_localization PUBLIC AUTONOMY_HAS_ONNXRUNTIME)
  target_include_directories(autonomy_localization SYSTEM PUBLIC ${OnnxRuntime_INCLUDE_DIRS})
  target_link_libraries(autonomy_localization PUBLIC ${OnnxRuntime_LIBRARIES})
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  target_compile_definitions(autonomy_localization PUBLIC AUTONOMY_HAS_TENSORRT)
  target_include_directories(autonomy_localization SYSTEM PUBLIC ${TensorRT_INCLUDE_DIRS})
  target_link_libraries(autonomy_localization PUBLIC ${TensorRT_LIBRARIES})
endif()

if(Ipopt_FOUND AND NOT TARGET Ipopt::Ipopt)
  target_link_libraries(autonomy_localization PUBLIC ${IPOPT_LIBRARIES})
endif()
if(foxglove-sdk_FOUND)
  target_link_libraries(autonomy_localization PUBLIC
    foxglove-sdk::foxglove_cpp_shared ZLIB::ZLIB)
endif()
if(BUILD_PROMETHEUS)
  target_compile_definitions(autonomy_localization PRIVATE USE_PROMETHEUS=1)
  target_link_libraries(autonomy_localization PRIVATE
    prometheus-cpp-core prometheus-cpp-pull)
endif()

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.localization
  NO_EXPORT
  SOURCES localization_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)
add_subdirectory(atlas)

# --- install -----------------------------------------------------------------
autocmake_install_share(localization
  CONF
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
