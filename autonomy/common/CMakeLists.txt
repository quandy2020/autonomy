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
# autonomy/common — shared utilities
#
# Builds
#   autonomy_common / autonomy::common
#
# Toggle
#   AUTONOMY_BUILD_COMMON
#   BUILD_ONNXRUNTIME / BUILD_TENSORRT / BUILD_GRPC
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
option(BUILD_ONNXRUNTIME "ONNX Runtime" ON)
option(BUILD_TENSORRT "TensorRT" OFF)
autocmake_find(OnnxRuntime PRIVATE QUIET)
autocmake_find(TensorRT PRIVATE QUIET)
autocmake_find(CUDAToolkit PRIVATE QUIET)
autocmake_find(Ipopt PRIVATE QUIET)
autocmake_find(gRPC PRIVATE QUIET CONFIG)
if(NOT gRPC_FOUND)
  autocmake_find(gRPC PRIVATE QUIET)
endif()

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_COMMON_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_COMMON_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_COMMON_SRCS EXCLUDE REGEX "_test\\.cpp$")
list(FILTER AUTONOMY_COMMON_SRCS EXCLUDE REGEX "lua_parameter_dictionary\\.cpp$")
list(FILTER AUTONOMY_COMMON_SRCS EXCLUDE REGEX "configuration_file_resolver\\.cpp$")

if(NOT BUILD_GRPC)
  list(FILTER AUTONOMY_COMMON_SRCS EXCLUDE REGEX "/async_grpc/")
endif()
if(NOT Ipopt_FOUND)
  list(FILTER AUTONOMY_COMMON_SRCS EXCLUDE REGEX "/optimization/ipopt/")
endif()

# Network backends are opt-in; keep only tensor.cpp by default.
file(GLOB_RECURSE _net_drop "${CMAKE_CURRENT_SOURCE_DIR}/network/*.cpp")
list(REMOVE_ITEM AUTONOMY_COMMON_SRCS ${_net_drop})
list(APPEND AUTONOMY_COMMON_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/network/common/tensor.cpp")
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  file(GLOB_RECURSE _onnx
    "${CMAKE_CURRENT_SOURCE_DIR}/network/backend/onnx/*.cpp")
  list(APPEND AUTONOMY_COMMON_SRCS ${_onnx})
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  file(GLOB_RECURSE _trt
    "${CMAKE_CURRENT_SOURCE_DIR}/network/backend/tensorrt/*.cpp")
  list(APPEND AUTONOMY_COMMON_SRCS ${_trt})
endif()

if(AUTONOMY_VERSION_CPP)
  list(FILTER AUTONOMY_COMMON_SRCS EXCLUDE REGEX "/version\\.cpp$")
  list(APPEND AUTONOMY_COMMON_SRCS "${AUTONOMY_VERSION_CPP}")
  set_source_files_properties("${AUTONOMY_VERSION_CPP}" PROPERTIES GENERATED TRUE)
endif()

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_common
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_COMMON_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR})
add_library(autonomy::common ALIAS autonomy_common)

target_link_libraries(autonomy_common PUBLIC
  protobuf::libprotobuf
  automsgs
  Eigen3::Eigen
  Ceres::ceres
  yaml-cpp::yaml-cpp
  glog::glog
  autonomy::cli11
  autolink
  Threads::Threads
  nlohmann_json::nlohmann_json
  $<TARGET_NAME_IF_EXISTS:OSQP::OSQP>
  $<TARGET_NAME_IF_EXISTS:Ipopt::Ipopt>
  $<$<BOOL:${BUILD_GRPC}>:grpc++>
  $<$<BOOL:${BUILD_GRPC}>:grpc>
  $<$<AND:$<BOOL:${BUILD_GRPC}>,$<TARGET_EXISTS:gRPC::grpc++_reflection>>:gRPC::grpc++_reflection>
  $<TARGET_NAME_IF_EXISTS:CUDA::cudart>)

target_compile_definitions(autonomy_common PUBLIC
  $<$<AND:$<BOOL:${BUILD_GRPC}>,$<TARGET_EXISTS:gRPC::grpc++_reflection>>:AUTONOMY_HAVE_GRPC_REFLECTION=1>)

if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  target_compile_definitions(autonomy_common PUBLIC AUTONOMY_HAS_ONNXRUNTIME)
  target_include_directories(autonomy_common SYSTEM PUBLIC ${OnnxRuntime_INCLUDE_DIRS})
  target_link_libraries(autonomy_common PUBLIC ${OnnxRuntime_LIBRARIES})
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  target_compile_definitions(autonomy_common PUBLIC AUTONOMY_HAS_TENSORRT)
  target_include_directories(autonomy_common SYSTEM PUBLIC ${TensorRT_INCLUDE_DIRS})
  target_link_libraries(autonomy_common PUBLIC ${TensorRT_LIBRARIES})
endif()

if(Ipopt_FOUND AND NOT TARGET Ipopt::Ipopt)
  target_link_libraries(autonomy_common PUBLIC ${IPOPT_LIBRARIES})
endif()
if(NOT TARGET OSQP::OSQP AND OSQP_LIBRARIES)
  target_link_libraries(autonomy_common PUBLIC ${OSQP_LIBRARIES})
endif()
