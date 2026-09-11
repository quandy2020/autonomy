# Collect autonomy sources and apply build-option filters.
set(_AUTONOMY_ROOT "${PROJECT_SOURCE_DIR}/autonomy")

file(GLOB_RECURSE ALL_LIBRARY_HDRS "${_AUTONOMY_ROOT}/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "${_AUTONOMY_ROOT}/*.cpp")

set(_AUTONOMY_GRPC_DIRS common/async_grpc bridge/grpc)
foreach(_dir IN LISTS _AUTONOMY_GRPC_DIRS)
  file(GLOB_RECURSE _grpc_hdrs "${_AUTONOMY_ROOT}/${_dir}/*.hpp")
  file(GLOB_RECURSE _grpc_srcs "${_AUTONOMY_ROOT}/${_dir}/*.cpp")
  file(GLOB_RECURSE _grpc_all "${_AUTONOMY_ROOT}/${_dir}/*")
  list(APPEND ALL_GRPC_HDRS ${_grpc_hdrs})
  list(APPEND ALL_GRPC_SRCS ${_grpc_srcs})
  list(APPEND ALL_GRPC_FILES ${_grpc_all})
endforeach()
file(GLOB_RECURSE ALL_GRPC_BRIDGE_FILES "${_AUTONOMY_ROOT}/bridge/bridge_server.*")

file(GLOB_RECURSE TEST_LIBRARY_HDRS
  "${_AUTONOMY_ROOT}/fake_*.hpp"
  "${_AUTONOMY_ROOT}/*test_helpers*.hpp"
  "${_AUTONOMY_ROOT}/*test*.hpp"
  "${_AUTONOMY_ROOT}/mock_*.hpp")
file(GLOB_RECURSE TEST_LIBRARY_SRCS
  "${_AUTONOMY_ROOT}/fake_*.cpp"
  "${_AUTONOMY_ROOT}/*test_helpers*.cpp"
  "${_AUTONOMY_ROOT}/*test*.cpp"
  "${_AUTONOMY_ROOT}/mock_*.cpp")

# Production filter (ROS grid_map MockFilter), not a test double.
list(REMOVE_ITEM TEST_LIBRARY_HDRS
  "${_AUTONOMY_ROOT}/map/grid_map/grid_map_filters/mock_filter.hpp")
list(REMOVE_ITEM TEST_LIBRARY_SRCS
  "${_AUTONOMY_ROOT}/map/grid_map/grid_map_filters/mock_filter.cpp")

file(GLOB_RECURSE ALL_TOOLS "${_AUTONOMY_ROOT}/tools/*")
file(GLOB_RECURSE ALL_EXECUTABLES "${_AUTONOMY_ROOT}/*main.cpp")
file(GLOB_RECURSE ALL_TESTS "${_AUTONOMY_ROOT}/*_test.cpp")
file(GLOB_RECURSE VISUALIZATION_SRCS "${_AUTONOMY_ROOT}/visualization/*.cpp")
file(GLOB_RECURSE ONNX_NETWORK_SRCS "${_AUTONOMY_ROOT}/common/network/*.cpp")
list(REMOVE_ITEM ONNX_NETWORK_SRCS
  "${_AUTONOMY_ROOT}/common/network/common/tensor.cpp")
# Concrete Engine::Create paths need an inference backend. Detector pipeline
# stays in libautonomy when ORT is on.
set(BASE_COMPONENT_SRCS
  "${_AUTONOMY_ROOT}/perception/base/base_component.cpp"
  "${_AUTONOMY_ROOT}/perception/base/engine/model.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/detect/detect.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/segment/segment.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/classify/classify.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/pose/pose.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/obb/obb.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/track/track.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/depth/depth.cpp"
  "${_AUTONOMY_ROOT}/perception/base/tasks/depth/moge.cpp")
set(BASE_COMPONENT_TEST_SRCS
  "${_AUTONOMY_ROOT}/perception/base/base_component_test.cpp")
set(FOLLOW_COMPONENT_SRCS
  "${_AUTONOMY_ROOT}/perception/follow/follow_component.cpp"
  "${_AUTONOMY_ROOT}/perception/follow/options.cpp"
  "${_AUTONOMY_ROOT}/perception/follow/localizer.cpp"
  "${_AUTONOMY_ROOT}/perception/follow/grid.cpp"
  "${_AUTONOMY_ROOT}/perception/follow/planner.cpp")
set(FOLLOW_COMPONENT_TEST_SRCS)
set(AUDIO_COMPONENT_SRCS
  "${_AUTONOMY_ROOT}/audio/audio_component.cpp"
  "${_AUTONOMY_ROOT}/audio/options.cpp"
  "${_AUTONOMY_ROOT}/audio/common/audio_info.cpp"
  "${_AUTONOMY_ROOT}/audio/common/message_process.cpp"
  "${_AUTONOMY_ROOT}/audio/inference/fft.cpp"
  "${_AUTONOMY_ROOT}/audio/inference/direction_detection.cpp"
  "${_AUTONOMY_ROOT}/audio/inference/moving_detection.cpp"
  "${_AUTONOMY_ROOT}/audio/inference/asr/asr_engine.cpp")
set(AUDIO_COMPONENT_TEST_SRCS)
# Offline demos are built as separate binaries (see grid_map_demos/CMakeLists.txt).
file(GLOB_RECURSE _GRID_MAP_DEMOS_SRCS
  "${_AUTONOMY_ROOT}/map/grid_map/grid_map_demos/*")
if(_GRID_MAP_DEMOS_SRCS)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${_GRID_MAP_DEMOS_SRCS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${_GRID_MAP_DEMOS_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${_GRID_MAP_DEMOS_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${_GRID_MAP_DEMOS_SRCS})
  list(REMOVE_ITEM ALL_TESTS ${_GRID_MAP_DEMOS_SRCS})
  list(REMOVE_ITEM ALL_EXECUTABLES ${_GRID_MAP_DEMOS_SRCS})
endif()
unset(_GRID_MAP_DEMOS_SRCS)

unset(_AUTONOMY_ROOT)
unset(_AUTONOMY_GRPC_DIRS)
unset(_dir)
unset(_grpc_hdrs)
unset(_grpc_srcs)
unset(_grpc_all)

# Filter library / test source lists according to build options.
function(autonomy_filter_library_sources)
  file(GLOB_RECURSE ALL_DOTFILES ".*/*")
  if(ALL_DOTFILES)
    list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
    list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
    list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
    list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
    list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_DOTFILES})
  endif()

  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
  # Autolink registration entrypoints belong only to their component DSOs.
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${BASE_COMPONENT_SRCS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${FOLLOW_COMPONENT_SRCS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${AUDIO_COMPONENT_SRCS})
  # Component lifecycle tests link the component DSO directly when ORT exists.
  list(REMOVE_ITEM ALL_TESTS ${BASE_COMPONENT_TEST_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${BASE_COMPONENT_TEST_SRCS})
  if(FOLLOW_COMPONENT_TEST_SRCS)
    list(REMOVE_ITEM ALL_TESTS ${FOLLOW_COMPONENT_TEST_SRCS})
    list(REMOVE_ITEM TEST_LIBRARY_SRCS ${FOLLOW_COMPONENT_TEST_SRCS})
  endif()
  if(AUDIO_COMPONENT_TEST_SRCS)
    list(REMOVE_ITEM ALL_TESTS ${AUDIO_COMPONENT_TEST_SRCS})
    list(REMOVE_ITEM TEST_LIBRARY_SRCS ${AUDIO_COMPONENT_TEST_SRCS})
  endif()
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TESTS})

  # common/network backends: keep shared engine when ORT and/or TensorRT exist.
  set(_network_backend FALSE)
  if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
    set(_network_backend TRUE)
  endif()
  if(BUILD_TENSORRT AND TensorRT_FOUND)
    set(_network_backend TRUE)
  endif()
  if(NOT _network_backend)
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ONNX_NETWORK_SRCS})
  else()
    if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND)
      list(REMOVE_ITEM ALL_LIBRARY_SRCS
        "${PROJECT_SOURCE_DIR}/autonomy/common/network/backend/onnx/onnx.cpp"
        "${PROJECT_SOURCE_DIR}/autonomy/common/network/backend/onnx/io.cpp")
    endif()
    if(NOT BUILD_TENSORRT OR NOT TensorRT_FOUND)
      list(REMOVE_ITEM ALL_LIBRARY_SRCS
        "${PROJECT_SOURCE_DIR}/autonomy/common/network/backend/tensorrt/tensorrt.cpp")
    endif()
  endif()

  unset(_network_backend)

  if(NOT BUILD_GRPC)
    list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_HDRS})
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_SRCS})
    list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_GRPC_HDRS})
    list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_GRPC_SRCS})
    list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
    list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
    list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_BRIDGE_FILES})
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_BRIDGE_FILES})
  endif()

  if(NOT BUILD_TOOLS)
    list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_TOOLS})
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TOOLS})
    list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_TOOLS})
    list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TOOLS})
    list(REMOVE_ITEM ALL_TESTS ${ALL_TOOLS})
    list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_TOOLS})
  endif()

  # Foxglove visualization requires foxglove-sdk (main/test already excluded).
  if(NOT foxglove-sdk_FOUND)
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${VISUALIZATION_SRCS})
  endif()

  # fakedata_test is a foxglove demo binary (disabled), not a unit test.
  list(REMOVE_ITEM ALL_TESTS
    "${PROJECT_SOURCE_DIR}/autonomy/visualization/fakedata_test.cpp")

  # Temporary: incomplete automsgs field-access migration in strata.
  # Re-enable once protobuf accessors are fully converted.
  file(GLOB_RECURSE _AUTOMSGS_WIP_SRCS
    "${PROJECT_SOURCE_DIR}/autonomy/map/strata/*.cpp")
  if(_AUTOMSGS_WIP_SRCS)
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${_AUTOMSGS_WIP_SRCS})
  endif()
  # Matching tests for WIP trees (sources excluded above).
  file(GLOB_RECURSE _AUTOMSGS_WIP_TESTS
    "${PROJECT_SOURCE_DIR}/autonomy/map/strata/*_test.cpp")
  if(_AUTOMSGS_WIP_TESTS)
    list(REMOVE_ITEM ALL_TESTS ${_AUTOMSGS_WIP_TESTS})
  endif()
  unset(_AUTOMSGS_WIP_SRCS)
  unset(_AUTOMSGS_WIP_TESTS)

  set(ALL_LIBRARY_HDRS "${ALL_LIBRARY_HDRS}" PARENT_SCOPE)
  set(ALL_LIBRARY_SRCS "${ALL_LIBRARY_SRCS}" PARENT_SCOPE)
  set(TEST_LIBRARY_HDRS "${TEST_LIBRARY_HDRS}" PARENT_SCOPE)
  set(TEST_LIBRARY_SRCS "${TEST_LIBRARY_SRCS}" PARENT_SCOPE)
  set(ALL_TESTS "${ALL_TESTS}" PARENT_SCOPE)
  set(ALL_EXECUTABLES "${ALL_EXECUTABLES}" PARENT_SCOPE)
  set(ALL_DOTFILES "${ALL_DOTFILES}" PARENT_SCOPE)
endfunction()
