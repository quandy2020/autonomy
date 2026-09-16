# @file autonomy_superproject.cmake
# @brief Root-CMakeLists orchestration: module graph, companion embeds,
#        codegen, and version metadata.
#
# Domain modules only include(autonomy_module); they do not load this file.
# @see autonomy_options.cmake autonomy_build.cmake

include_guard(GLOBAL)

include(autonomy_options)

# @brief Declare domain options, validate the dependency graph, and fill
#        @c AUTONOMY_ENABLED_MODULES.
# @pre Root CMakeLists has already declared product options (BUILD_GRPC, etc.).
macro(autonomy_superproject_bootstrap_modules)
  set(AUTONOMY_MODULE_ORDER
    common transform map vehicle manipulation prediction control planning
    perception localization sensor task system audio bridge visualization)

  autonomy_declare_module_options()
  autonomy_validate_module_graph()
  autonomy_compute_enabled_modules(AUTONOMY_ENABLED_MODULES)

  foreach(_module IN LISTS AUTONOMY_MODULE_ORDER)
    string(TOUPPER "${_module}" _module_upper)
    if(NOT AUTONOMY_BUILD_${_module_upper})
      message(STATUS
        "autonomy: module '${_module}' disabled by "
        "AUTONOMY_BUILD_${_module_upper}=OFF")
    endif()
  endforeach()
  unset(_module)
  unset(_module_upper)
endmacro()

# @brief add_subdirectory(autolink) and force-off its tests/docs options.
macro(autonomy_embed_autolink)
  set(AUTOLINK_BUILD_TEST OFF CACHE BOOL "" FORCE)
  set(AUTOLINK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(AUTOLINK_BUILD_TOOLS ${BUILD_TOOLS} CACHE BOOL "" FORCE)
  option(AUTOLINK_BUILD_PYTHON "Build autolink Python bindings" ON)
  set(AUTOLINK_BUILD_DOCS OFF CACHE BOOL "" FORCE)

  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/autolink/CMakeLists.txt")
    message(FATAL_ERROR
      "autolink submodule is not initialized.\n"
      "From the repository root, run:\n"
      "  git submodule update --init --recursive autolink")
  endif()

  add_subdirectory(autolink)
endmacro()

# @brief Add the autodriver subproject from @c ${PROJECT_SOURCE_DIR}/autodriver.
macro(autonomy_embed_autodriver)
  # Leave AUTODRIVER_BUILD_TEST / EXAMPLES to the caller.
  set(_autodriver_root "${PROJECT_SOURCE_DIR}/autodriver")
  if(NOT EXISTS "${_autodriver_root}/CMakeLists.txt")
    message(FATAL_ERROR
      "autodriver is not present at ${_autodriver_root}")
  endif()

  add_subdirectory("${_autodriver_root}" autodriver)
endmacro()

# @brief Stage autosim into the build tree and pip install (keeps the source
#        tree clean).
macro(autonomy_embed_autosim)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/autosim/pyproject.toml")
    message(FATAL_ERROR
      "autosim is missing at ${CMAKE_CURRENT_SOURCE_DIR}/autosim\n"
      "Expected pyproject.toml for pip install.")
  endif()

  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  set(_autosim_root "${CMAKE_CURRENT_SOURCE_DIR}/autosim")
  set(_autosim_stage "${CMAKE_CURRENT_BINARY_DIR}/autosim_pip")

  add_custom_target(autosim_pip_install ALL
    COMMAND ${CMAKE_COMMAND} -E rm -rf
            "${_autosim_stage}"
            "${_autosim_root}/autosim.egg-info"
            "${_autosim_root}/UNKNOWN.egg-info"
            "${_autosim_root}/build"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_autosim_stage}"
    COMMAND ${CMAKE_COMMAND} -E copy
            "${_autosim_root}/pyproject.toml"
            "${_autosim_stage}/pyproject.toml"
    COMMAND ${CMAKE_COMMAND} -E copy
            "${_autosim_root}/README.md"
            "${_autosim_stage}/README.md"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_autosim_root}/autosim"
            "${_autosim_stage}/autosim"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_autosim_root}/config"
            "${_autosim_stage}/config"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_autosim_root}/urdf"
            "${_autosim_stage}/urdf"
    COMMAND ${Python3_EXECUTABLE} -m pip install "${_autosim_stage}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Installing autosim Python package (pip, staged)"
    VERBATIM)

  if(TARGET automsgs)
    add_dependencies(autosim_pip_install automsgs)
  endif()
  if(TARGET autolink)
    add_dependencies(autosim_pip_install autolink)
  endif()

  message(STATUS "autosim: pip install (staged) ${_autosim_root}")
endmacro()

# @brief automsgs, optional autodriver, proto collection, config.hpp, version.cpp.
macro(autonomy_superproject_prepare_codegen)
  set(AUTONOMY_MODULE_CONDITION_bridge BUILD_GRPC)
  set(AUTONOMY_MODULE_CONDITION_visualization foxglove-sdk_FOUND)

  autonomy_configure_tests()

  add_subdirectory(automsgs)

  if(BUILD_AUTODRIVER)
    autonomy_embed_autodriver()
  endif()

  autonomy_collect_proto_sources()

  configure_file(
    "${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp.cmake"
    "${PROJECT_BINARY_DIR}/autonomy/common/config.hpp"
    @ONLY)
  autonomy_configure_version()
endmacro()
