# Collect autonomy sources and apply build-option filters.
set(_AUTONOMY_ROOT "${PROJECT_SOURCE_DIR}/autonomy")

file(GLOB_RECURSE ALL_LIBRARY_HDRS "${_AUTONOMY_ROOT}/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "${_AUTONOMY_ROOT}/*.cpp")

set(_AUTONOMY_GRPC_DIRS common/async_grpc bridge/plugins/grpc)
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
# Tensor buffers are also used by Fathom's injected-runner facade and do not
# depend on an inference backend.
list(REMOVE_ITEM ONNX_NETWORK_SRCS
  "${_AUTONOMY_ROOT}/common/network/common/tensor.cpp")
file(GLOB_RECURSE FATHOM_NETWORK_SRCS
  "${_AUTONOMY_ROOT}/perception/fathom/engine/*.cpp")
list(APPEND FATHOM_NETWORK_SRCS
  "${_AUTONOMY_ROOT}/perception/fathom/fathom_node_runner.cpp")
set(FATHOM_COMPONENT_SRCS
  "${_AUTONOMY_ROOT}/perception/fathom/fathom_component.cpp")
set(FATHOM_COMPONENT_TEST_SRCS
  "${_AUTONOMY_ROOT}/perception/fathom/fathom_component_test.cpp")

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
  # The autolink entrypoint is built only into libfathom_component.so.
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${FATHOM_COMPONENT_SRCS})
  # Component lifecycle tests link the component DSO directly when ORT exists.
  list(REMOVE_ITEM ALL_TESTS ${FATHOM_COMPONENT_TEST_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${FATHOM_COMPONENT_TEST_SRCS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TESTS})

  if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND)
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ONNX_NETWORK_SRCS})
    # Keep Fathom configuration, RGB-D processing, projection, and its
    # injected-runner refiner available when no concrete backend is built.
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${FATHOM_NETWORK_SRCS})
  endif()

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
