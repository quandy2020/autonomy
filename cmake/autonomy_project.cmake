# Project bootstrap helpers (gz_configure_project analogue).
# Options and find_package stay in the root CMakeLists.

include_guard(GLOBAL)

macro(autonomy_configure_cxx)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endmacro()

macro(autonomy_configure_project)
  autonomy_configure_cxx()
  set(AUTONOMY_WORKSPACE_ROOT "${PROJECT_SOURCE_DIR}")

  include(autonomy_version)
  read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")

  include("${PROJECT_SOURCE_DIR}/cmake/autonomy_common.cmake")
  autonomy_initialize_project()
  autonomy_enable_testing()
  # CMAKE_MODULE_PATH for cmake/ is usually set by the root CMakeLists before
  # this macro; keep appending for callers that only invoke configure_project.
  list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
  list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

  set(_autonomy_config_dir ${CMAKE_INSTALL_PREFIX}/share/autonomy)
  set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY
    ${_autonomy_config_dir}/localization/conf/cartographer
    CACHE PATH "Cartographer / module conf install root")
  set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY
    ${_autonomy_config_dir}/localization/conf/cartographer
    CACHE PATH ".lua configuration files directory for cartographer")
  unset(_autonomy_config_dir)
endmacro()
