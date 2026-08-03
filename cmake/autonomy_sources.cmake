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

file(GLOB_RECURSE ALL_TOOLS "${_AUTONOMY_ROOT}/tools/*")
file(GLOB_RECURSE ALL_EXECUTABLES "${_AUTONOMY_ROOT}/*main.cpp")
file(GLOB_RECURSE ALL_TESTS "${_AUTONOMY_ROOT}/*_test.cpp")
file(GLOB_RECURSE VISUALIZATION_SRCS "${_AUTONOMY_ROOT}/visualization/*.cpp")
file(GLOB_RECURSE ONNX_NETWORK_SRCS "${_AUTONOMY_ROOT}/common/network/*.cpp")

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
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TESTS})

  if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND)
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ONNX_NETWORK_SRCS})
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

  # Temporary: incomplete automsgs field-access migration in these trees.
  # Re-enable once protobuf accessors are fully converted.
  file(GLOB_RECURSE _AUTOMSGS_WIP_SRCS
    "${PROJECT_SOURCE_DIR}/autonomy/localization/cartographer/node/*.cpp"
    "${PROJECT_SOURCE_DIR}/autonomy/map/strata/*.cpp"
    "${PROJECT_SOURCE_DIR}/autonomy/task/apps/exploration/*.cpp")
  # Keep shared path helpers used by localization_main / atlas.
  list(REMOVE_ITEM _AUTOMSGS_WIP_SRCS
    "${PROJECT_SOURCE_DIR}/autonomy/localization/cartographer/node/node_utils.cpp")
  if(_AUTOMSGS_WIP_SRCS)
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${_AUTOMSGS_WIP_SRCS})
  endif()
  # Matching tests for WIP trees (sources excluded above).
  file(GLOB_RECURSE _AUTOMSGS_WIP_TESTS
    "${PROJECT_SOURCE_DIR}/autonomy/map/strata/*_test.cpp"
    "${PROJECT_SOURCE_DIR}/autonomy/localization/cartographer/node/*_test.cpp")
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
