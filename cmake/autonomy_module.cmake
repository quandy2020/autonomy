# @file autonomy_module.cmake
# @brief Domain-module CMake API: library / component / binary and dependency
#        resolution.
#
# @par Usage
#   - Root: @c include(autonomy_module) then @c autonomy_configure_project()
#   - Domain: @c autonomy_module(name) + @c autonomy_library(...)
#
# When @c AUTONOMY_SUPERPROJECT=ON, also loads build/package helpers.
# @par gz-cmake mapping
#   gz_create_core_library, gz_add_component

include_guard(GLOBAL)

include("${CMAKE_CURRENT_LIST_DIR}/autonomy_project.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/autonomy_deps.cmake")

# @brief Resolve a logical dependency name to a linkable CMake target.
# @param _dependency e.g. autonomy_common, common, or an existing TARGET name.
# @param _out_var Output variable (parent scope).
function(autonomy_resolve_dependency _dependency _out_var)
  if(TARGET "${_dependency}")
    set(${_out_var} "${_dependency}" PARENT_SCOPE)
    return()
  endif()
  if(TARGET "autonomy::${_dependency}")
    set(${_out_var} "autonomy::${_dependency}" PARENT_SCOPE)
    return()
  endif()
  string(REGEX REPLACE "^autonomy_" "" _mod "${_dependency}")
  if(TARGET "autonomy::autonomy_${_mod}")
    set(${_out_var} "autonomy::autonomy_${_mod}" PARENT_SCOPE)
    return()
  endif()
  message(FATAL_ERROR
    "autonomy_resolve_dependency: '${_dependency}' not found")
endfunction()

# @brief Set the current domain module name @c _AUTONOMY_MODULE_NAME.
# @param _mod Domain directory name (e.g. planning, common).
macro(autonomy_module _mod)
  set(_AUTONOMY_MODULE_NAME "${_mod}")
  if(NOT DEFINED AUTONOMY_WORKSPACE_ROOT)
    set(AUTONOMY_WORKSPACE_ROOT "${PROJECT_SOURCE_DIR}")
  endif()
endmacro()

# @brief Recursively GLOB *.cpp under the current directory (exclude main and
#        *_test).
# @param _out Output list variable name.
macro(autonomy_glob_srcs _out)
  file(GLOB_RECURSE ${_out} CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
  list(FILTER ${_out} EXCLUDE REGEX "main\\.cpp$")
  list(FILTER ${_out} EXCLUDE REGEX "_test\\.cpp$")
endmacro()

# @brief Create domain shared library @c autonomy_${mod} or an INTERFACE
#        placeholder.
# @param INTERFACE Header-only / placeholder module with no sources.
# @param NO_CORE Skip autonomy_link_core().
# @param DEPENDENCIES Other autonomy modules or targets.
# @param FEATURES Passed to autonomy_link_feature().
# @param SRCS HDRS Source lists (prefer autonomy_glob_srcs).
function(autonomy_library)
  cmake_parse_arguments(_ARG "INTERFACE;NO_CORE" "" "DEPENDENCIES;FEATURES;SRCS;HDRS" ${ARGN})
  set(_mod "${_AUTONOMY_MODULE_NAME}")
  if(NOT _mod)
    message(FATAL_ERROR "autonomy_library: call autonomy_module first")
  endif()

  set(_target "autonomy_${_mod}")
  set(_srcs ${_ARG_SRCS})
  set(_hdrs ${_ARG_HDRS})

  if(_ARG_INTERFACE)
    set(_srcs "")
    set(_hdrs "")
  elseif(NOT _srcs AND NOT _hdrs)
    message(FATAL_ERROR
      "autonomy_library(${_mod}): pass SRCS via autonomy_glob_srcs() or INTERFACE")
  endif()

  # Generated version.cpp replaces the stub when building the super-project.
  if(_mod STREQUAL "common" AND AUTONOMY_VERSION_CPP AND _srcs)
    list(FILTER _srcs EXCLUDE REGEX "/version\\.cpp$")
    list(APPEND _srcs "${AUTONOMY_VERSION_CPP}")
    set_source_files_properties("${AUTONOMY_VERSION_CPP}" PROPERTIES GENERATED TRUE)
  endif()

  if(NOT _srcs)
    add_library(${_target} INTERFACE)
    set(_iface_dependencies "")
    foreach(_dependency IN LISTS _ARG_DEPENDENCIES)
      autonomy_resolve_dependency("${_dependency}" _r)
      list(APPEND _iface_dependencies "${_r}")
    endforeach()
    if(TARGET autonomy_proto)
      list(APPEND _iface_dependencies autonomy_proto)
    elseif(TARGET autonomy::autonomy_proto)
      list(APPEND _iface_dependencies autonomy::autonomy_proto)
    endif()
    if(_iface_dependencies)
      target_link_libraries(${_target} INTERFACE ${_iface_dependencies})
    endif()
    target_include_directories(${_target} INTERFACE
      $<BUILD_INTERFACE:${AUTONOMY_WORKSPACE_ROOT}>
      $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
      $<INSTALL_INTERFACE:include>)
  else()
    add_library(${_target} SHARED ${_srcs} ${_hdrs})
    set_target_properties(${_target} PROPERTIES
      OUTPUT_NAME "autonomy_${_mod}"
      BUILD_RPATH_USE_ORIGIN TRUE
      INSTALL_RPATH "\$ORIGIN:${CMAKE_INSTALL_PREFIX}/lib")
    target_include_directories(${_target} PUBLIC
      $<BUILD_INTERFACE:${AUTONOMY_WORKSPACE_ROOT}>
      $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
      $<INSTALL_INTERFACE:include>)
    if(NOT _ARG_NO_CORE)
      autonomy_link_core(${_target})
      if(TARGET autonomy_proto)
        target_link_libraries(${_target} PUBLIC autonomy_proto)
      elseif(TARGET autonomy::autonomy_proto)
        target_link_libraries(${_target} PUBLIC autonomy::autonomy_proto)
      endif()
    endif()
    if(_ARG_DEPENDENCIES)
      set(_resolved "")
      foreach(_dependency IN LISTS _ARG_DEPENDENCIES)
        autonomy_resolve_dependency("${_dependency}" _r)
        list(APPEND _resolved "${_r}")
      endforeach()
      target_link_libraries(${_target} PUBLIC ${_resolved})
    endif()
    if(_ARG_FEATURES)
      autonomy_link_feature(${_target} ${_ARG_FEATURES})
    endif()
    if(TARGET automsgs_proto_copy)
      add_dependencies(${_target} automsgs_proto_copy)
    endif()
  endif()

  add_library(autonomy::${_mod} ALIAS ${_target})
  set_property(GLOBAL APPEND PROPERTY AUTONOMY_MODULE_TARGETS ${_target})
endfunction()

# @brief Build a loadable component DSO (gz_add_component analogue).
# @param _name Target name.
# @param SOURCES Required.
# @param NO_AUTOLINK Do not link autolink.
# @param GET_TARGET_NAME Variable name to receive the actual target name.
function(autonomy_component _name)
  cmake_parse_arguments(_ARG
    "NO_AUTOLINK"
    "GET_TARGET_NAME"
    "SOURCES;PUBLIC_LINK_LIBS;PRIVATE_LINK_LIBS"
    ${ARGN})
  if(NOT _ARG_SOURCES)
    message(FATAL_ERROR "autonomy_component(${_name}): SOURCES required")
  endif()
  if(NOT _ARG_NO_AUTOLINK AND NOT TARGET autolink)
    message(FATAL_ERROR
      "autonomy_component(${_name}): autolink target missing "
      "(enable autolink embed or pass NO_AUTOLINK)")
  endif()
  if(NOT TARGET autonomy AND NOT TARGET autonomy::autonomy)
    message(FATAL_ERROR
      "autonomy_component(${_name}): autonomy umbrella missing")
  endif()

  add_library(${_name} SHARED ${_ARG_SOURCES})
  set(_inc_root "${AUTONOMY_WORKSPACE_ROOT}")
  if(NOT _inc_root)
    set(_inc_root "${PROJECT_SOURCE_DIR}")
  endif()
  target_include_directories(${_name} PRIVATE
    ${_inc_root}
    ${PROJECT_BINARY_DIR})

  set(_priv "")
  if(TARGET autonomy)
    list(APPEND _priv autonomy)
  else()
    list(APPEND _priv autonomy::autonomy)
  endif()
  if(NOT _ARG_NO_AUTOLINK AND TARGET autolink)
    list(APPEND _priv autolink)
  endif()
  if(TARGET automsgs)
    list(APPEND _priv automsgs)
  endif()
  target_link_libraries(${_name} PRIVATE ${_priv} ${_ARG_PRIVATE_LINK_LIBS})
  if(_ARG_PUBLIC_LINK_LIBS)
    target_link_libraries(${_name} PUBLIC ${_ARG_PUBLIC_LINK_LIBS})
  endif()
  set_target_properties(${_name} PROPERTIES OUTPUT_NAME "${_name}")

  if(_ARG_GET_TARGET_NAME)
    set(${_ARG_GET_TARGET_NAME} "${_name}" PARENT_SCOPE)
  endif()
endfunction()

# @brief Executable installed to bin; links the umbrella @c ${PROJECT_NAME} by
#        default.
# @param _exe Target name.
# @param SRCS Source list.
# @param DEPENDENCIES Extra PRIVATE link targets.
function(autonomy_binary _exe)
  cmake_parse_arguments(_ARG "" "" "SRCS;DEPENDENCIES" ${ARGN})
  if(NOT _ARG_SRCS)
    message(FATAL_ERROR "autonomy_binary(${_exe}): SRCS required")
  endif()

  add_executable(${_exe} ${_ARG_SRCS})
  set_target_properties(${_exe} PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH FALSE
    INSTALL_RPATH "\$ORIGIN/../lib:${CMAKE_INSTALL_PREFIX}/lib"
    BUILD_RPATH "\$ORIGIN:\$ORIGIN/../lib:${CMAKE_BINARY_DIR}/lib"
    SKIP_BUILD_RPATH FALSE)
  if(AUTONOMY_CXX_FLAGS)
    set_target_properties(${_exe} PROPERTIES COMPILE_FLAGS "${AUTONOMY_CXX_FLAGS}")
  endif()

  target_link_libraries(${_exe} PUBLIC ${PROJECT_NAME})
  if(_ARG_DEPENDENCIES)
    set(_bin_dependencies "")
    foreach(_dependency IN LISTS _ARG_DEPENDENCIES)
      autonomy_resolve_dependency("${_dependency}" _r)
      list(APPEND _bin_dependencies "${_r}")
    endforeach()
    target_link_libraries(${_exe} PRIVATE ${_bin_dependencies})
  endif()
  target_link_libraries(${_exe} PRIVATE glog::glog gflags::gflags)
  install(TARGETS ${_exe} RUNTIME DESTINATION bin)
endfunction()

# Super-project only: domain CMakeLists do not need install/test orchestration.
if(AUTONOMY_SUPERPROJECT)
  include("${CMAKE_CURRENT_LIST_DIR}/autonomy_build.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/autonomy_package.cmake")
endif()
