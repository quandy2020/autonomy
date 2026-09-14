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

# autonomy/common — shared utilities, math, time, optional inference

include(autonomy_module)
autonomy_module(common)

# Library sources: GLOB *.cpp; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_COMMON_SRCS)

set(_COMMON_OSQP_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/math/mpc_osqp.cpp")
list(REMOVE_ITEM _COMMON_SRCS "${_COMMON_OSQP_SOURCE}")

# LuaParameterDictionary lives with localization (Cartographer); keep common Lua-free.
list(FILTER _COMMON_SRCS EXCLUDE REGEX "lua_parameter_dictionary\\.cpp$")
list(FILTER _COMMON_SRCS EXCLUDE REGEX "configuration_file_resolver\\.cpp$")

# Optional gRPC helpers (async_grpc/) only when BUILD_GRPC is ON.
if(NOT BUILD_GRPC)
  list(FILTER _COMMON_SRCS EXCLUDE REGEX "/async_grpc/")
endif()

# The Ipopt adapter is available only when a consuming domain discovered Ipopt.
if(NOT Ipopt_FOUND)
  list(FILTER _COMMON_SRCS EXCLUDE REGEX "/optimization/ipopt/")
endif()

# network/: keep tensor.cpp always; add ONNX / TensorRT backends when found.
file(GLOB_RECURSE _net_drop "${CMAKE_CURRENT_SOURCE_DIR}/network/*.cpp")
if(_net_drop)
  list(REMOVE_ITEM _COMMON_SRCS ${_net_drop})
endif()
list(APPEND _COMMON_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/network/common/tensor.cpp")
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  file(GLOB_RECURSE _onnx
    "${CMAKE_CURRENT_SOURCE_DIR}/network/backend/onnx/*.cpp")
  list(APPEND _COMMON_SRCS ${_onnx})
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  file(GLOB_RECURSE _trt
    "${CMAKE_CURRENT_SOURCE_DIR}/network/backend/tensorrt/*.cpp")
  list(APPEND _COMMON_SRCS ${_trt})
endif()
unset(_net_drop)
unset(_onnx)
unset(_trt)

autonomy_library(NO_CORE FEATURES inference SRCS ${_COMMON_SRCS})
target_link_libraries(autonomy_common PUBLIC
  protobuf::libprotobuf
  automsgs
  Eigen3::Eigen
  Ceres::ceres
  yaml-cpp::yaml-cpp
  glog::glog
  gflags::gflags
  autolink
  Threads::Threads
  nlohmann_json::nlohmann_json)

if(AUTONOMY_BUILD_COMMON_OSQP)
  add_library(autonomy_common_osqp SHARED "${_COMMON_OSQP_SOURCE}")
  target_include_directories(autonomy_common_osqp PUBLIC
    $<BUILD_INTERFACE:${AUTONOMY_WORKSPACE_ROOT}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(autonomy_common_osqp PUBLIC
    autonomy_common Eigen3::Eigen OSQP::OSQP)
  add_library(autonomy::common_osqp ALIAS autonomy_common_osqp)
  set_property(GLOBAL APPEND
    PROPERTY AUTONOMY_MODULE_TARGETS autonomy_common_osqp)
endif()

unset(_COMMON_SRCS)
unset(_COMMON_OSQP_SOURCE)
