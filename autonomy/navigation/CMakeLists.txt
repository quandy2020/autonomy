# Copyright 2026 The Navigation Authors (navigation)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# -----------------------------------------------------------------------------
# autonomy/navigation — end-to-end navigation
#
# Builds
#   navigation / autonomy::navigation
#   navigation_proto
#   optional autolink components
#
# Toggle
#   AUTONOMY_BUILD_NAVIGATION
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
autocmake_find(behaviortree_cpp REQUIRED PRIVATE)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE NAVIGATION_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER NAVIGATION_SRCS EXCLUDE REGEX "/(tools|proto)/")
list(FILTER NAVIGATION_SRCS EXCLUDE REGEX "(_test|_component|benchmark_main|goal_sender_main|configuration_validator_main)\\.cpp$")

# --- protos ------------------------------------------------------------------
set(NAVIGATION_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${NAVIGATION_GENERATED_DIR}/navigation/proto")
file(MAKE_DIRECTORY "${NAVIGATION_GENERATED_DIR}/python")
set(NAVIGATION_AUTOMSGS_PROTO_ROOT "${CMAKE_CURRENT_BINARY_DIR}/automsgs_proto")
file(MAKE_DIRECTORY "${NAVIGATION_AUTOMSGS_PROTO_ROOT}")
if(NOT EXISTS "${NAVIGATION_AUTOMSGS_PROTO_ROOT}/automsgs")
  file(CREATE_LINK "${PROJECT_SOURCE_DIR}/automsgs/proto"
       "${NAVIGATION_AUTOMSGS_PROTO_ROOT}/automsgs" SYMBOLIC)
endif()

file(GLOB NAVIGATION_PROTO_INPUTS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
set(NAVIGATION_PROTO_SRCS)
foreach(proto IN LISTS NAVIGATION_PROTO_INPUTS)
  get_filename_component(name "${proto}" NAME_WE)
  list(APPEND NAVIGATION_PROTO_SRCS
    "${NAVIGATION_GENERATED_DIR}/navigation/proto/${name}.pb.cc")
endforeach()

add_custom_command(
  OUTPUT ${NAVIGATION_PROTO_SRCS}
  COMMAND protobuf::protoc
    "--cpp_out=${NAVIGATION_GENERATED_DIR}"
    "--python_out=${NAVIGATION_GENERATED_DIR}/python"
    "--proto_path=${PROJECT_SOURCE_DIR}/autonomy"
    "--proto_path=${NAVIGATION_AUTOMSGS_PROTO_ROOT}"
    ${NAVIGATION_PROTO_INPUTS}
  DEPENDS ${NAVIGATION_PROTO_INPUTS} protobuf::protoc
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  COMMENT "Generating navigation protos"
  VERBATIM)

autocmake_library(navigation_proto
  SHARED
  NO_EXPORT
  SOURCES ${NAVIGATION_PROTO_SRCS}
  INCLUDES "${NAVIGATION_GENERATED_DIR}"
  DEPENDENCIES protobuf::libprotobuf automsgs)

# --- library -----------------------------------------------------------------
autocmake_library(navigation
  SHARED
  NO_EXPORT
  SOURCES ${NAVIGATION_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}/autonomy
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    navigation_proto
    autonomy_common
    automsgs
    Threads::Threads
    OpenSSL::Crypto
    BT::behaviortree_cpp)

# --- components --------------------------------------------------------------
if(TARGET autolink)
  function(navigation_component name source)
    autocmake_library(${name}
      SHARED
      NO_EXPORT
      SOURCES "${source}"
      INCLUDES ${PROJECT_SOURCE_DIR}/autonomy
      PRIVATE_DEPENDENCIES navigation autolink)
  endfunction()

  navigation_component(navigation_observation_component
    perception/observation_builder_component.cpp)
  navigation_component(navigation_behavior_component
    behavior/behavior_component.cpp)
  navigation_component(navigation_component navigation_component.cpp)
  navigation_component(navigation_safety_component safety/safety_component.cpp)
  navigation_component(navigation_guardian_component guardian/guardian_component.cpp)

  autocmake_binary(navigation_goal_sender
    NO_EXPORT
    SOURCES tools/goal_sender_main.cpp
    DEPENDENCIES navigation_proto autolink)
endif()

# --- tests -------------------------------------------------------------------
option(NAVIGATION_BUILD_TESTS "Build navigation tests" ${BUILD_TESTING})
if(NAVIGATION_BUILD_TESTS)
  enable_testing()
  file(GLOB_RECURSE NAVIGATION_TESTS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp")
  list(FILTER NAVIGATION_TESTS EXCLUDE REGEX "navigation_component_test\\.cpp$")
  foreach(test IN LISTS NAVIGATION_TESTS)
    get_filename_component(name "${test}" NAME_WE)
    add_executable(${name} "${test}")
    target_link_libraries(${name} PRIVATE navigation navigation_proto)
    add_test(NAME ${name} COMMAND ${name})
  endforeach()

  add_executable(navigation_configuration_validator
    tools/configuration_validator_main.cpp)
  target_link_libraries(navigation_configuration_validator PRIVATE navigation)
  add_test(NAME configuration_validator
    COMMAND navigation_configuration_validator "${CMAKE_CURRENT_SOURCE_DIR}/conf")

  if(TARGET navigation_component)
    add_executable(navigation_component_integration tools/component_integration_main.cpp)
    target_link_libraries(navigation_component_integration PRIVATE navigation autolink
      navigation_observation_component navigation_component
      navigation_behavior_component navigation_safety_component navigation_guardian_component)
    add_test(NAME component_integration
      COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tools/run_component_integration.sh"
        $<TARGET_FILE:navigation_component_integration> "${CMAKE_CURRENT_SOURCE_DIR}")
    set_tests_properties(component_integration PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
  endif()
endif()

# --- tools -------------------------------------------------------------------
add_executable(navigation_benchmark tools/benchmark_main.cpp)
target_link_libraries(navigation_benchmark PRIVATE navigation)
add_executable(navigation_defm_validator tools/defm_validator_main.cpp)
target_link_libraries(navigation_defm_validator PRIVATE navigation)

# --- install -----------------------------------------------------------------
autocmake_install_share(navigation
  CONF
  DAG
  LAUNCH)
install(DIRECTORY behavior/trees DESTINATION "share/${PROJECT_NAME}/navigation")
# Match the source resource layout used by BtCppEngine. Keep the existing
# trees directory for consumers of the previous installed layout.
install(DIRECTORY behavior/trees DESTINATION "share/${PROJECT_NAME}/navigation/behavior")
install(FILES README.md DESTINATION "share/${PROJECT_NAME}/doc/navigation")
install(DIRECTORY docs DESTINATION "share/${PROJECT_NAME}/doc/navigation")
