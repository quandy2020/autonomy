# Install / export rules for autonomy.
# autolink is installed with EXPORT AutonomyExport from autolink/CMakeLists.txt
# when embedded; do not list it here again.

install(
  TARGETS ${PROJECT_NAME}
  EXPORT AutonomyExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_AUTODRIVER AND TARGET autodriver)
  install(
    DIRECTORY autodriver/autodriver/
    DESTINATION include/autodriver
    FILES_MATCHING PATTERN "*.hpp"
  )
  install(
    TARGETS autodriver autodriver_autonomy_bridge
    EXPORT AutonomyExport
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )
endif()

install(DIRECTORY config DESTINATION share/autonomy)
install(DIRECTORY cmake DESTINATION share/autonomy/)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/autonomy-config-version.cmake"
  DESTINATION share/autonomy)

set(_autonomy_hdr_excludes PATTERN "internal" EXCLUDE)
if(NOT BUILD_GRPC)
  list(APPEND _autonomy_hdr_excludes
    PATTERN "async_grpc" EXCLUDE
    PATTERN "plugins/grpc" EXCLUDE
    PATTERN "bridge_server.*" EXCLUDE)
endif()
# Exclude only top-level autonomy/tools (path relative to autonomy/).
# Do not use PATTERN "tools" — that also drops mppi_controller/tools headers.
if(NOT BUILD_TOOLS)
  list(APPEND _autonomy_hdr_excludes REGEX "^tools/" EXCLUDE)
endif()
if(NOT foxglove-sdk_FOUND)
  list(APPEND _autonomy_hdr_excludes PATTERN "visualization" EXCLUDE)
endif()

install(
  DIRECTORY autonomy/
  DESTINATION include/autonomy
  FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
    ${_autonomy_hdr_excludes}
)

install(
  DIRECTORY "${PROJECT_BINARY_DIR}/autonomy/"
  DESTINATION include/autonomy
  FILES_MATCHING
    PATTERN "*.pb.h"
    PATTERN "*.grpc.pb.h"
)

set(AUTONOMY_CMAKE_DIR share/autonomy/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  autonomy-config.cmake.in
  ${PROJECT_BINARY_DIR}/autonomy-config.cmake
  PATH_VARS AUTONOMY_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/autonomy
)
install(
  FILES ${PROJECT_BINARY_DIR}/autonomy-config.cmake
  DESTINATION share/autonomy/
)

set(_autonomy_launch_mods localization planning control exploration task system)
if(BUILD_GRPC)
  list(APPEND _autonomy_launch_mods bridge)
endif()
foreach(_mod IN LISTS _autonomy_launch_mods)
  install(
    DIRECTORY autonomy/${_mod}/launch/
    DESTINATION share/autonomy/${_mod}/launch
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING
    PATTERN "*.launch"
  )
endforeach()

install(
  EXPORT AutonomyExport
  NAMESPACE autonomy::
  DESTINATION share/autonomy/cmake
  FILE AutonomyTargets.cmake
  EXPORT_LINK_INTERFACE_LIBRARIES
)
