# @file autonomy_package.cmake
# @brief Install/export @c AutonomyExport and @c autonomy-config.cmake
#        (gz_create_packages install half).
#
# Invoked by autonomy_create_packages() at the end of configure.

include_guard(GLOBAL)

# @brief Register unit tests and run autonomy_install_package().
function(autonomy_create_packages)
  autonomy_add_tests()
  autonomy_install_package()
endfunction()

# @brief Install libraries/headers/launch/conf; generate package config and
#        uninstall target.
function(autonomy_install_package)
  autonomy_collect_required_package_groups(_autonomy_package_groups)
  foreach(_group IN ITEMS
      common_math common_vision osqp map control task localization grpc
      autoviz prometheus sherpa_onnx inference foxglove)
    string(TOUPPER "${_group}" _group_upper)
    if(_group IN_LIST _autonomy_package_groups)
      set(AUTONOMY_PACKAGE_HAS_${_group_upper} ON)
    else()
      set(AUTONOMY_PACKAGE_HAS_${_group_upper} OFF)
    endif()
  endforeach()
  set(AUTONOMY_PACKAGE_HAS_IPOPT ${Ipopt_FOUND})

  get_property(_AUTONOMY_INSTALL_TARGETS GLOBAL PROPERTY AUTONOMY_MODULE_TARGETS)
  if(TARGET autonomy)
    list(INSERT _AUTONOMY_INSTALL_TARGETS 0 autonomy)
  endif()
  list(REMOVE_DUPLICATES _AUTONOMY_INSTALL_TARGETS)
  install(
    TARGETS ${_AUTONOMY_INSTALL_TARGETS}
    EXPORT AutonomyExport
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
  unset(_AUTONOMY_INSTALL_TARGETS)

  if(BUILD_AUTODRIVER AND TARGET autodriver)
    install(
      DIRECTORY autodriver/autodriver/
      DESTINATION include/autodriver
      FILES_MATCHING PATTERN "*.hpp")
    install(
      FILES ${CMAKE_BINARY_DIR}/autodriver/autodriver/conf/conf.hpp
      DESTINATION include/autodriver/conf)
    install(
      DIRECTORY autodriver/config/
      DESTINATION share/autodriver/config
      FILES_MATCHING PATTERN "*.yaml")
    install(
      TARGETS autodriver
      EXPORT AutonomyExport
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib)
    if(TARGET autodriver_main)
      install(
        TARGETS autodriver_main
        EXPORT AutonomyExport
        RUNTIME DESTINATION bin)
    endif()
  endif()

  # Module conf trees are installed by each autonomy_<mod> CMakeLists.
  # (Legacy root config/ removed — use autonomy/<mod>/conf/)
  # Only Find modules required by autonomy-config.cmake (not helpers/docs).
  install(DIRECTORY cmake/modules DESTINATION share/autonomy/cmake)
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/autonomy-config-version.cmake"
    DESTINATION share/autonomy)

  set(_autonomy_hdr_excludes PATTERN "internal" EXCLUDE)
  if(NOT BUILD_GRPC)
    list(APPEND _autonomy_hdr_excludes
      PATTERN "async_grpc" EXCLUDE
      REGEX "bridge/grpc/" EXCLUDE
      PATTERN "bridge_server.*" EXCLUDE)
  endif()
  if(NOT BUILD_TOOLS)
    list(APPEND _autonomy_hdr_excludes REGEX "^tools/" EXCLUDE)
  endif()
  if(NOT foxglove-sdk_FOUND)
    list(APPEND _autonomy_hdr_excludes PATTERN "visualization" EXCLUDE)
  endif()

  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    install(
      DIRECTORY "autonomy/${_mod}/"
      DESTINATION "include/autonomy/${_mod}"
      FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
      ${_autonomy_hdr_excludes})
    if(IS_DIRECTORY "${PROJECT_BINARY_DIR}/autonomy/${_mod}")
      install(
        DIRECTORY "${PROJECT_BINARY_DIR}/autonomy/${_mod}/"
        DESTINATION "include/autonomy/${_mod}"
        FILES_MATCHING
          PATTERN "*.pb.h" PATTERN "*.grpc.pb.h" PATTERN "config.hpp")
    endif()
  endforeach()

  set(AUTONOMY_CMAKE_DIR share/autonomy/cmake)
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/autonomy_config.cmake.in"
    ${PROJECT_BINARY_DIR}/autonomy-config.cmake
    PATH_VARS AUTONOMY_CMAKE_DIR
    INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/autonomy)
  unset(_autonomy_package_groups)
  unset(_group)
  unset(_group_upper)
  install(
    FILES ${PROJECT_BINARY_DIR}/autonomy-config.cmake
    DESTINATION share/autonomy/)

  # Optional uninstall target (cmake/autonomy_uninstall.cmake.in).
  if(NOT TARGET uninstall)
    configure_file(
      "${PROJECT_SOURCE_DIR}/cmake/autonomy_uninstall.cmake.in"
      "${PROJECT_BINARY_DIR}/autonomy_uninstall.cmake"
      @ONLY)
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P
        "${PROJECT_BINARY_DIR}/autonomy_uninstall.cmake"
      COMMENT "Uninstall files listed in install_manifest.txt")
  endif()

  set(_autonomy_launch_mods "")
  foreach(_mod IN ITEMS localization planning control manipulation task system)
    if(_mod IN_LIST AUTONOMY_ENABLED_MODULES)
      list(APPEND _autonomy_launch_mods "${_mod}")
    endif()
  endforeach()
  if(TARGET base_component)
    install(TARGETS base_component LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
    list(APPEND _autonomy_launch_mods perception)
  endif()
  if(TARGET follow_component)
    install(TARGETS follow_component LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
    list(APPEND _autonomy_launch_mods perception)
  endif()
  if(TARGET audio_component)
    install(TARGETS audio_component LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
    list(APPEND _autonomy_launch_mods audio)
    install(
      DIRECTORY autonomy/audio/conf/
      DESTINATION share/autonomy/audio/conf
      FILES_MATCHING PATTERN "*.pb.txt" PATTERN "*.yaml")
    install(
      DIRECTORY autonomy/audio/dag/
      DESTINATION share/autonomy/audio/dag
      FILES_MATCHING PATTERN "*.dag")
  endif()
  if("perception" IN_LIST AUTONOMY_ENABLED_MODULES
      AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/autonomy/perception/launch")
    list(APPEND _autonomy_launch_mods perception)
  endif()
  if("perception" IN_LIST AUTONOMY_ENABLED_MODULES
      AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/autonomy/perception/conf")
    install(
      DIRECTORY autonomy/perception/conf/
      DESTINATION share/autonomy/perception/conf
      FILES_MATCHING PATTERN "*.pb.txt")
  endif()
  if("perception" IN_LIST AUTONOMY_ENABLED_MODULES
      AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/autonomy/perception/base/dag")
    install(
      DIRECTORY autonomy/perception/base/dag/
      DESTINATION share/autonomy/perception/base/dag
      FILES_MATCHING PATTERN "*.dag")
  endif()
  if("perception" IN_LIST AUTONOMY_ENABLED_MODULES
      AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/autonomy/perception/follow/dag")
    install(
      DIRECTORY autonomy/perception/follow/dag/
      DESTINATION share/autonomy/perception/follow/dag
      FILES_MATCHING PATTERN "*.dag")
  endif()
  list(REMOVE_DUPLICATES _autonomy_launch_mods)
  foreach(_mod IN LISTS _autonomy_launch_mods)
    install(
      DIRECTORY autonomy/${_mod}/launch/
      DESTINATION share/autonomy/${_mod}/launch
      USE_SOURCE_PERMISSIONS
      FILES_MATCHING PATTERN "*.launch")
  endforeach()

  install(
    EXPORT AutonomyExport
    NAMESPACE autonomy::
    DESTINATION share/autonomy/cmake
    FILE AutonomyTargets.cmake
    EXPORT_LINK_INTERFACE_LIBRARIES)
endfunction()
