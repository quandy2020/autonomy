# @file autonomy_build.cmake
# @brief Super-project build orchestration: unit-test discovery, Protobuf/gRPC
#        codegen, domain modules, and the umbrella target.
#
# Loaded by autonomy_module.cmake only when the root sets @c AUTONOMY_SUPERPROJECT.
# @see cmake/README.md
# @par gz-cmake mapping
#   gz_configure_build, test discovery

include_guard(GLOBAL)

# @brief Collect unit-test and helper sources for enabled domain modules.
# @details Sets parent-scope @c AUTONOMY_TEST_MODULES, @c AUTONOMY_TESTS_${mod},
#          @c AUTONOMY_TEST_HELPER_SRCS_${mod}, @c AUTONOMY_TEST_HELPER_HDRS_${mod}.
# @pre @c AUTONOMY_ENABLED_MODULES must already be computed.
function(autonomy_configure_tests)
  if(NOT DEFINED AUTONOMY_ENABLED_MODULES)
    message(FATAL_ERROR
      "autonomy_configure_tests: compute AUTONOMY_ENABLED_MODULES first")
  endif()
  set(_root "${PROJECT_SOURCE_DIR}/autonomy")
  set(AUTONOMY_TEST_MODULES "")

  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    if(NOT IS_DIRECTORY "${_root}/${_mod}")
      continue()
    endif()

    # Narrow GLOB: tests + helpers (filename patterns; recursive).
    file(GLOB_RECURSE _tests CONFIGURE_DEPENDS "${_root}/${_mod}/*_test.cpp")
    set(_module_helper_srcs "")
    set(_module_helper_hdrs "")

    file(GLOB_RECURSE _helper_srcs CONFIGURE_DEPENDS
      "${_root}/${_mod}/*fake*.cpp"
      "${_root}/${_mod}/*mock*.cpp"
      "${_root}/${_mod}/*test_helpers*.cpp")
    foreach(_f IN LISTS _helper_srcs)
      if(_f MATCHES "_test\\.cpp$")
        continue()
      endif()
      if(NOT _f MATCHES "/grid_map/grid_map_filters/mock_filter\\.cpp$")
        list(APPEND _module_helper_srcs "${_f}")
      endif()
    endforeach()

    file(GLOB_RECURSE _helper_hdrs CONFIGURE_DEPENDS
      "${_root}/${_mod}/*fake*.hpp"
      "${_root}/${_mod}/*mock*.hpp"
      "${_root}/${_mod}/*test_helpers*.hpp")
    foreach(_f IN LISTS _helper_hdrs)
      if(NOT _f MATCHES "/grid_map/grid_map_filters/mock_filter\\.hpp$")
        list(APPEND _module_helper_hdrs "${_f}")
      endif()
    endforeach()

    if(_mod STREQUAL "common")
      if(NOT AUTONOMY_BUILD_COMMON_OSQP)
        list(REMOVE_ITEM _tests
          "${_root}/common/math/mpc_osqp_test.cpp")
      endif()
      if(NOT Ipopt_FOUND)
        list(FILTER _tests EXCLUDE REGEX "/optimization/(ipopt|test)/")
      endif()
    elseif(_mod STREQUAL "visualization")
      list(REMOVE_ITEM _tests
        "${_root}/visualization/fakedata_test.cpp")
    elseif(_mod STREQUAL "perception")
      list(REMOVE_ITEM _tests
        "${_root}/perception/base/base_component_test.cpp")
    endif()

    list(REMOVE_DUPLICATES _tests)
    list(REMOVE_DUPLICATES _module_helper_srcs)
    list(REMOVE_DUPLICATES _module_helper_hdrs)
    list(APPEND AUTONOMY_TEST_MODULES "${_mod}")

    set(AUTONOMY_TESTS_${_mod} "${_tests}" PARENT_SCOPE)
    set(AUTONOMY_TEST_HELPER_SRCS_${_mod} "${_module_helper_srcs}" PARENT_SCOPE)
    set(AUTONOMY_TEST_HELPER_HDRS_${_mod} "${_module_helper_hdrs}" PARENT_SCOPE)
  endforeach()

  set(AUTONOMY_TEST_MODULES "${AUTONOMY_TEST_MODULES}" PARENT_SCOPE)
endfunction()


# @brief Register protoc / grpc_cpp_plugin custom commands for enabled modules.
# @details Sets parent-scope @c ALL_PROTO_SRCS, @c ALL_PROTO_HDRS,
#          @c ALL_GRPC_SERVICE_SRCS, @c ALL_GRPC_SERVICE_HDRS.
# @pre @c AUTONOMY_ENABLED_MODULES must be set; with @c BUILD_GRPC=ON,
#      grpc_cpp_plugin must be on PATH.
function(autonomy_collect_proto_sources)
  set(_proto_srcs "")
  set(_proto_hdrs "")
  set(_grpc_srcs "")
  set(_grpc_hdrs "")

  set(PROTOBUF_PROTOC_EXECUTABLE "${Protobuf_PROTOC_EXECUTABLE}")

  # Ubuntu jammy protoc 3.12 needs this for proto3 optional; 3.15+ is fine.
  set(_AUTONOMY_PROTOC_EXTRA_ARGS "")
  if(DEFINED _protoc_version)
    set(_pv "${_protoc_version}")
  else()
    execute_process(
      COMMAND "${PROTOBUF_PROTOC_EXECUTABLE}" --version
      OUTPUT_VARIABLE _pv
      ERROR_VARIABLE _pv_err
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE)
    set(_pv "${_pv}${_pv_err}")
  endif()
  if(_pv MATCHES "([0-9]+)\\.([0-9]+)")
    set(_pmaj "${CMAKE_MATCH_1}")
    set(_pmin "${CMAKE_MATCH_2}")
    if(_pmaj EQUAL 3 AND _pmin LESS 15)
      list(APPEND _AUTONOMY_PROTOC_EXTRA_ARGS
        --experimental_allow_proto3_optional)
    elseif(_pmaj LESS 3)
      list(APPEND _AUTONOMY_PROTOC_EXTRA_ARGS
        --experimental_allow_proto3_optional)
    endif()
  endif()

  set(_AUTONOMY_PROTO_INCLUDES -I ${PROJECT_SOURCE_DIR})
  if(DEFINED AUTOMSGS_PROTO_INCLUDE_DIR)
    list(APPEND _AUTONOMY_PROTO_INCLUDES -I ${AUTOMSGS_PROTO_INCLUDE_DIR})
  endif()
  set(_AUTONOMY_PROTO_DEPS "")
  if(TARGET automsgs_proto_copy)
    list(APPEND _AUTONOMY_PROTO_DEPS automsgs_proto_copy)
  endif()

  set(ALL_PROTOS "")
  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    file(GLOB_RECURSE _module_protos
      "${PROJECT_SOURCE_DIR}/autonomy/${_mod}/*.proto")
    list(APPEND ALL_PROTOS ${_module_protos})
  endforeach()
  # orbisview/*.proto are FE/schema docs (JSON + render_schemas.h); not C++ codegen.
  list(FILTER ALL_PROTOS EXCLUDE REGEX ".*/commsgs/proto/.*")
  list(FILTER ALL_PROTOS EXCLUDE REGEX ".*/orbisview/proto/.*")
  set(ALL_GRPC_SERVICES ${ALL_PROTOS})
  list(FILTER ALL_GRPC_SERVICES INCLUDE REGEX "_service\\.proto$")

  if(ALL_GRPC_SERVICES)
    list(REMOVE_ITEM ALL_PROTOS ${ALL_GRPC_SERVICES})
  endif()

  if(BUILD_GRPC)
    find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
    if(NOT GRPC_CPP_PLUGIN)
      message(FATAL_ERROR
        "grpc_cpp_plugin not found. Install gRPC and ensure it is on PATH.")
    endif()

    foreach(ABS_FIL ${ALL_GRPC_SERVICES})
      file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
      get_filename_component(DIR ${REL_FIL} DIRECTORY)
      get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

      set(_grpc_cc "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc")
      set(_grpc_h  "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h")
      set(_pb_cc   "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
      set(_pb_h    "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

      list(APPEND _grpc_srcs ${_grpc_cc} ${_pb_cc})
      list(APPEND _grpc_hdrs ${_grpc_h} ${_pb_h})

      add_custom_command(
        OUTPUT ${_grpc_cc} ${_grpc_h} ${_pb_cc} ${_pb_h}
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS
          ${_AUTONOMY_PROTOC_EXTRA_ARGS}
          --grpc_out=${PROJECT_BINARY_DIR}
          --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
          --cpp_out=${PROJECT_BINARY_DIR}
          ${_AUTONOMY_PROTO_INCLUDES}
          ${ABS_FIL}
        DEPENDS ${ABS_FIL} ${_AUTONOMY_PROTO_DEPS}
        COMMENT "Running gRPC/protoc on ${REL_FIL}"
        VERBATIM
      )
    endforeach()

    set_source_files_properties(
      ${_grpc_srcs} ${_grpc_hdrs} PROPERTIES GENERATED TRUE)
  endif()

  foreach(ABS_FIL ${ALL_PROTOS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

    set(_pb_cc "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
    set(_pb_h  "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

    list(APPEND _proto_srcs ${_pb_cc})
    list(APPEND _proto_hdrs ${_pb_h})

    add_custom_command(
      OUTPUT ${_pb_cc} ${_pb_h}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS ${_AUTONOMY_PROTOC_EXTRA_ARGS}
           --cpp_out=${PROJECT_BINARY_DIR}
           ${_AUTONOMY_PROTO_INCLUDES}
           ${ABS_FIL}
      DEPENDS ${ABS_FIL} ${_AUTONOMY_PROTO_DEPS}
      COMMENT "Running protoc on ${REL_FIL}"
      VERBATIM
    )
  endforeach()

  set_source_files_properties(
    ${_proto_srcs} ${_proto_hdrs} PROPERTIES GENERATED TRUE)

  set(ALL_PROTO_SRCS "${_proto_srcs}" PARENT_SCOPE)
  set(ALL_PROTO_HDRS "${_proto_hdrs}" PARENT_SCOPE)
  set(ALL_GRPC_SERVICE_SRCS "${_grpc_srcs}" PARENT_SCOPE)
  set(ALL_GRPC_SERVICE_HDRS "${_grpc_hdrs}" PARENT_SCOPE)
endfunction()

# @brief Build-phase entry: proto library → add_subdirectory domains → link
#        umbrella @c autonomy.
function(autonomy_configure_build)
  autonomy_add_proto()
  autonomy_add_modules()
  autonomy_link_umbrella()
endfunction()

# @brief Create shared library @c autonomy_proto (alias @c autonomy::proto).
# @pre Call autonomy_collect_proto_sources() first.
function(autonomy_add_proto)
  set(_all ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})
  if(BUILD_GRPC)
    list(APPEND _all ${ALL_GRPC_SERVICE_SRCS} ${ALL_GRPC_SERVICE_HDRS})
  endif()
  add_library(autonomy_proto SHARED ${_all})
  target_include_directories(autonomy_proto PUBLIC
    $<BUILD_INTERFACE:${AUTONOMY_WORKSPACE_ROOT}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(autonomy_proto PUBLIC
    protobuf::libprotobuf automsgs)
  if(BUILD_GRPC)
    target_link_libraries(autonomy_proto PUBLIC grpc++ grpc)
  endif()
  if(TARGET automsgs_proto_copy)
    add_dependencies(autonomy_proto automsgs_proto_copy)
  endif()
  set_property(GLOBAL APPEND PROPERTY AUTONOMY_MODULE_TARGETS autonomy_proto)
  add_library(autonomy::proto ALIAS autonomy_proto)
endfunction()

# @brief Call add_subdirectory(autonomy/${mod}) for each enabled module.
# @details If @c AUTONOMY_MODULE_CONDITION_${mod} is set, its value is a condition
#          variable name (e.g. BUILD_GRPC).
# @pre @c AUTONOMY_ENABLED_MODULES must already be computed.
function(autonomy_add_modules)
  if(NOT DEFINED AUTONOMY_ENABLED_MODULES)
    message(FATAL_ERROR
      "autonomy_add_modules: compute AUTONOMY_ENABLED_MODULES first")
  endif()
  if(NOT TARGET autonomy)
    add_library(autonomy INTERFACE)
    add_library(autonomy::autonomy ALIAS autonomy)
  endif()
  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    if(DEFINED AUTONOMY_MODULE_CONDITION_${_mod})
      # Value is a variable *name* (e.g. BUILD_GRPC or foxglove-sdk_FOUND).
      set(_cond_name "${AUTONOMY_MODULE_CONDITION_${_mod}}")
      if(NOT "${${_cond_name}}")
        message(STATUS "autonomy: skip module '${_mod}' (${_cond_name}=OFF)")
        continue()
      endif()
    endif()
    add_subdirectory(autonomy/${_mod})
  endforeach()
endfunction()

# @brief Attach @c autonomy_proto and each @c autonomy_${mod} to INTERFACE
#        target @c autonomy.
function(autonomy_link_umbrella)
  set(_deps autonomy_proto)
  foreach(_mod IN LISTS AUTONOMY_MODULE_ORDER)
    if(TARGET autonomy_${_mod})
      list(APPEND _deps autonomy_${_mod})
    endif()
  endforeach()
  target_link_libraries(autonomy INTERFACE ${_deps})
  target_include_directories(autonomy INTERFACE
    $<BUILD_INTERFACE:${AUTONOMY_WORKSPACE_ROOT}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>)
endfunction()

# @brief Register per-module gtest executables and @c autonomy_${mod}_test_support.
# @pre @c BUILD_TEST=ON and autonomy_configure_tests() has been called.
function(autonomy_add_tests)
  if(NOT BUILD_TEST)
    return()
  endif()
  foreach(_mod IN LISTS AUTONOMY_TEST_MODULES)
    set(_module_target "autonomy_${_mod}")
    set(_test_support_target "${_module_target}_test_support")
    set(_test_helper_hdrs ${AUTONOMY_TEST_HELPER_HDRS_${_mod}})
    set(_test_helper_srcs ${AUTONOMY_TEST_HELPER_SRCS_${_mod}})

    if(_test_helper_srcs)
      add_library(${_test_support_target}
        ${_test_helper_hdrs} ${_test_helper_srcs})
      target_include_directories(${_test_support_target} SYSTEM PRIVATE
        "${GMOCK_INCLUDE_DIRS}")
      target_link_libraries(${_test_support_target} PRIVATE ${_module_target})
    else()
      add_library(${_test_support_target} INTERFACE)
      if(_test_helper_hdrs)
        target_sources(${_test_support_target} INTERFACE ${_test_helper_hdrs})
      endif()
      target_include_directories(${_test_support_target} SYSTEM INTERFACE
        "${GMOCK_INCLUDE_DIRS}")
    endif()

    foreach(ABS_FIL IN LISTS AUTONOMY_TESTS_${_mod})
      file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
      get_filename_component(DIR ${REL_FIL} DIRECTORY)
      get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
      string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")

      autonomy_test("${TEST_TARGET_NAME}" ${ABS_FIL} ${_module_target})
      if(TARGET automsgs)
        add_dependencies("${TEST_TARGET_NAME}" automsgs)
      endif()
      if(BUILD_GRPC)
        target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++ grpc)
      endif()
      if(BUILD_PROMETHEUS)
        target_link_libraries("${TEST_TARGET_NAME}" PUBLIC
          prometheus-cpp-core prometheus-cpp-pull)
      endif()
      target_link_libraries("${TEST_TARGET_NAME}" PRIVATE
        ${_test_support_target})
    endforeach()
  endforeach()
endfunction()
