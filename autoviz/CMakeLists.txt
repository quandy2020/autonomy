# Copyright 2026 The Openbot Authors (duyongquan)
#
# Autoviz — Autolink native 3D visualizer (depends on automsgs + autolink only).
#
# Standalone configure from this directory:
#   cmake -B build -DBUILD_AUTOVIZ=ON
#   cmake --build build --target autoviz
# Super-project configure (src/autonomy):
#   cmake -B build -DBUILD_AUTOVIZ=ON && cmake --build build --target autoviz

cmake_minimum_required(VERSION 3.20)

if(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY OR NOT (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR))
  # Included from Autonomy super-project — build autoviz target below.
elseif(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(autonomy LANGUAGES CXX C)
  if(NOT DEFINED BUILD_AUTOVIZ)
    set(BUILD_AUTOVIZ ON CACHE BOOL "Build native 3D visualization tool (autoviz)" FORCE)
  endif()
  message(STATUS "Autoviz: configuring via Autonomy super-project (${CMAKE_CURRENT_LIST_DIR}/..)")
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ${CMAKE_BINARY_DIR}/_autonomy)
  return()
else()
  message(FATAL_ERROR
    "autoviz must be configured with -DBUILD_AUTOVIZ=ON from the Autonomy root "
    "(${CMAKE_CURRENT_LIST_DIR}/..), or run cmake from the autoviz package directory.")
endif()

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets OpenGL Xml Svg Network)

get_filename_component(_AUTONOMY_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

option(AUTOVIZ_USE_QML_VEHICLE "QGC-style Qt Quick 3D vehicle preview panel" ON)
if(DEFINED AUTOVIZ_USE_QML_DRONE AND NOT AUTOVIZ_USE_QML_DRONE)
  set(AUTOVIZ_USE_QML_VEHICLE OFF)
endif()
set(_AUTOVIZ_HAS_QML_VEHICLE OFF)
if(AUTOVIZ_USE_QML_VEHICLE)
  find_package(Qt6 QUIET COMPONENTS Quick QuickWidgets Quick3D Qml)
  if(Qt6Quick_FOUND AND Qt6QuickWidgets_FOUND AND Qt6Quick3D_FOUND AND Qt6Qml_FOUND)
    set(_AUTOVIZ_HAS_QML_VEHICLE ON)
  else()
    message(WARNING
      "Autoviz: Qt6 Quick3D not found — disabling AUTOVIZ_USE_QML_VEHICLE. "
      "Install qt6-declarative-dev qt6-quick3d-dev qt6-quick3d-dev-tools "
      "libqt6shadertools6-dev or set -DAUTOVIZ_USE_QML_VEHICLE=OFF")
    set(AUTOVIZ_USE_QML_VEHICLE OFF)
  endif()
endif()
set(AUTOVIZ_USE_QML_DRONE ${AUTOVIZ_USE_QML_VEHICLE})
set(_AUTOVIZ_HAS_QML_DRONE ${_AUTOVIZ_HAS_QML_VEHICLE})

option(AUTOVIZ_USE_OGRE "Use Ogre render backend for autoviz" OFF)
option(AUTOVIZ_OGRE_VENDOR "Build Ogre 1.12.10 from source for rviz-identical point cloud GLSL (see docs/OGRE_VISUAL_ALIGNMENT.md)" OFF)
option(AUTOVIZ_OGRE_AUTO_VENDOR "Auto-enable AUTOVIZ_OGRE_VENDOR when system Ogre is not 1.12.x" OFF)
option(BUILD_AUTOVIZ_TESTS "Build autoviz verification tools" OFF)
option(AUTOVIZ_USE_ASSIMP "Use Assimp in Ogre mesh_loader" ON)
set(AUTOVIZ_OGRE_ROOT "" CACHE PATH "Prefix of a prebuilt Ogre install (optional)")

set(_AUTOVIZ_SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/autoviz)
file(GLOB_RECURSE AUTOVIZ_SOURCES CONFIGURE_DEPENDS "${_AUTOVIZ_SRC_ROOT}/*.cpp")
file(GLOB_RECURSE AUTOVIZ_HEADERS CONFIGURE_DEPENDS "${_AUTOVIZ_SRC_ROOT}/*.hpp")

# GLOB is evaluated at configure time; keep Ogre-only TUs explicit so re-config
# always picks them up when AUTOVIZ_USE_OGRE is enabled.
set(_AUTOVIZ_OGRE_EXTRA_SOURCES
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_movable_text.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_billboard_line.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_shape.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_line.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_arrow.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_wrench_visual.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_screw_visual.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_effort_visual.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_covariance_visual.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_triangle_polygon.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_mesh_shape.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/geometry.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/orthographic.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_logging.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_mesh_loader.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/mesh_resource.cpp
  ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_indexed_palette.cpp
  ${_AUTOVIZ_SRC_ROOT}/display/ogre_pbr_mesh_draw.cpp
  ${_AUTOVIZ_SRC_ROOT}/display/ogre_entity_draw.cpp
)

if(AUTOVIZ_USE_OGRE AND AUTOVIZ_USE_ASSIMP)
  find_package(assimp QUIET)
  if(NOT assimp_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(ASSIMP assimp)
    endif()
  endif()
  if(assimp_FOUND OR ASSIMP_FOUND)
    list(APPEND _AUTOVIZ_OGRE_EXTRA_SOURCES
      ${_AUTOVIZ_SRC_ROOT}/rendering/mesh_loader_helpers/assimp_loader.cpp)
    set(_AUTOVIZ_HAS_ASSIMP ON)
  else()
    message(WARNING "Assimp not found; mesh_loader supports OBJ/STL/.mesh only")
  endif()
endif()

if(AUTOVIZ_USE_OGRE)
  foreach(_ogre_src ${_AUTOVIZ_OGRE_EXTRA_SOURCES})
    if(EXISTS "${_ogre_src}" AND NOT "${_ogre_src}" IN_LIST AUTOVIZ_SOURCES)
      list(APPEND AUTOVIZ_SOURCES "${_ogre_src}")
    endif()
  endforeach()
else()
  foreach(_ogre_src ${_AUTOVIZ_OGRE_EXTRA_SOURCES})
    list(REMOVE_ITEM AUTOVIZ_SOURCES "${_ogre_src}")
  endforeach()
  list(REMOVE_ITEM AUTOVIZ_SOURCES
    "${_AUTOVIZ_SRC_ROOT}/rendering/viewport_projection_finder.cpp")
  list(REMOVE_ITEM AUTOVIZ_HEADERS
    "${_AUTOVIZ_SRC_ROOT}/rendering/viewport_projection_finder.hpp")
endif()

set(_AUTOVIZ_RECORDER_PLAYER_SOURCES
  ${_AUTONOMY_ROOT}/autolink/autolink/tools/autolink_recorder/player/player.cpp
  ${_AUTONOMY_ROOT}/autolink/autolink/tools/autolink_recorder/player/play_task.cpp
  ${_AUTONOMY_ROOT}/autolink/autolink/tools/autolink_recorder/player/play_task_buffer.cpp
  ${_AUTONOMY_ROOT}/autolink/autolink/tools/autolink_recorder/player/play_task_consumer.cpp
  ${_AUTONOMY_ROOT}/autolink/autolink/tools/autolink_recorder/player/play_task_producer.cpp
)

add_executable(autoviz ${AUTOVIZ_SOURCES} ${AUTOVIZ_HEADERS} ${_AUTOVIZ_RECORDER_PLAYER_SOURCES}
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/autoviz.qrc)

target_include_directories(autoviz PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${_AUTONOMY_ROOT}
)

add_dependencies(autoviz automsgs)

target_link_libraries(autoviz PRIVATE
  automsgs
  autolink
  yaml-cpp
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  Qt6::Xml
  Qt6::Svg
  Qt6::Network
  glog
  protobuf::libprotobuf
)

if(UNIX AND NOT APPLE)
  target_link_libraries(autoviz PRIVATE ${CMAKE_DL_LIBS})
elseif(APPLE)
  target_link_libraries(autoviz PRIVATE ${CMAKE_DL_LIBS})
endif()

if(WIN32)
  target_link_libraries(autoviz PRIVATE kernel32)
endif()

if(AUTOVIZ_USE_QML_DRONE AND _AUTOVIZ_HAS_QML_DRONE)
  add_subdirectory(qml)
  target_compile_definitions(autoviz PRIVATE AUTOVIZ_USE_QML_DRONE)
  target_link_libraries(autoviz PRIVATE
    autoviz_vehicle_qml
    autoviz_vehicle_qmlplugin
    autoviz_vehicle_qmlplugin_init
    Qt6::QuickWidgets)
endif()

if(AUTOVIZ_USE_OGRE)
  set(_AUTOVIZ_USE_OGRE_VENDOR ${AUTOVIZ_OGRE_VENDOR})
  if(NOT _AUTOVIZ_USE_OGRE_VENDOR AND NOT AUTOVIZ_OGRE_ROOT)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(_AUTOVIZ_OGRE_PROBE OGRE)
    endif()
    if(AUTOVIZ_OGRE_AUTO_VENDOR AND
       (NOT _AUTOVIZ_OGRE_PROBE_FOUND OR NOT _AUTOVIZ_OGRE_PROBE_VERSION MATCHES "^1\\.12"))
      message(STATUS
        "System Ogre is not 1.12.x (found '${_AUTOVIZ_OGRE_PROBE_VERSION}'); "
        "enabling AUTOVIZ_OGRE_VENDOR for rviz compatibility")
      set(_AUTOVIZ_USE_OGRE_VENDOR ON)
    endif()
  endif()

  if(_AUTOVIZ_USE_OGRE_VENDOR)
    message(STATUS "Autoviz: building Ogre 1.12.10 from source (FetchContent)")
    include(FetchContent)
    set(OGRE_BUILD_COMPONENT_BITES OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_COMPONENT_PYTHON OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_COMPONENT_JAVA OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_COMPONENT_CSHARP OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_SAMPLES FALSE CACHE BOOL "" FORCE)
    set(OGRE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(OGRE_CONFIG_THREADS 0 CACHE STRING "" FORCE)
    set(OGRE_RESOURCEMANAGER_STRICT 2 CACHE STRING "" FORCE)
    set(OGRE_BUILD_RENDERSYSTEM_GL TRUE CACHE BOOL "" FORCE)
    set(OGRE_BUILD_RENDERSYSTEM_D3D11 OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_RENDERSYSTEM_D3D9 OFF CACHE BOOL "" FORCE)
    set(_AUTOVIZ_OGRE_PATCH_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_ogre_patches.sh")
    FetchContent_Declare(
      autoviz_ogre
      GIT_REPOSITORY https://github.com/OGRECave/ogre.git
      GIT_TAG v1.12.10
      GIT_SHALLOW TRUE
      PATCH_COMMAND bash "${_AUTOVIZ_OGRE_PATCH_SCRIPT}" <SOURCE_DIR>)
    FetchContent_MakeAvailable(autoviz_ogre)
    set(_AUTOVIZ_OGRE_TARGET OgreMain)
    set(_AUTOVIZ_OGRE_OVERLAY_TARGET OgreOverlay)
    set(_AUTOVIZ_OGRE_INCLUDE_DIRS "${autoviz_ogre_SOURCE_DIR}/OgreMain/include")
    if(EXISTS "${autoviz_ogre_BINARY_DIR}/include")
      list(APPEND _AUTOVIZ_OGRE_INCLUDE_DIRS "${autoviz_ogre_BINARY_DIR}/include")
    endif()
    if(OGRE_PLUGINDIR)
      set(_AUTOVIZ_OGRE_PLUGIN_DIR "${OGRE_PLUGINDIR}")
    endif()
  elseif(AUTOVIZ_OGRE_ROOT)
    find_path(_AUTOVIZ_OGRE_INCLUDE_DIR OGRE/Ogre.h
              HINTS "${AUTOVIZ_OGRE_ROOT}/include" "${AUTOVIZ_OGRE_ROOT}/include/OGRE")
    find_library(_AUTOVIZ_OGRE_LIBRARY NAMES OgreMain
                 HINTS "${AUTOVIZ_OGRE_ROOT}/lib" "${AUTOVIZ_OGRE_ROOT}/lib64")
    find_library(_AUTOVIZ_OGRE_OVERLAY_LIBRARY NAMES OgreOverlay
                 HINTS "${AUTOVIZ_OGRE_ROOT}/lib" "${AUTOVIZ_OGRE_ROOT}/lib64")
    if(NOT _AUTOVIZ_OGRE_INCLUDE_DIR OR NOT _AUTOVIZ_OGRE_LIBRARY)
      message(FATAL_ERROR "AUTOVIZ_OGRE_ROOT does not contain OgreMain")
    endif()
    set(_AUTOVIZ_OGRE_INCLUDE_DIRS "${_AUTOVIZ_OGRE_INCLUDE_DIR}")
    set(_AUTOVIZ_OGRE_TARGET "${_AUTOVIZ_OGRE_LIBRARY}")
    set(_AUTOVIZ_OGRE_OVERLAY_TARGET "${_AUTOVIZ_OGRE_OVERLAY_LIBRARY}")
    set(_AUTOVIZ_OGRE_PLUGIN_DIR "${AUTOVIZ_OGRE_ROOT}/lib/OGRE")
  else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OGRE REQUIRED OGRE)
    pkg_check_modules(OGRE_OVERLAY REQUIRED OGRE-Overlay)
    if(OGRE_VERSION VERSION_LESS "14.0")
      message(WARNING
        "System Ogre ${OGRE_VERSION} is older than 14.x; Autoviz targets Ogre 14.4+ "
        "(stub materials). Use -DAUTOVIZ_OGRE_VENDOR=ON for legacy rviz 1.12 media.")
    else()
      message(STATUS "Autoviz Ogre backend: system Ogre ${OGRE_VERSION}")
    endif()
    set(_AUTOVIZ_OGRE_TARGET ${OGRE_LIBRARIES})
    set(_AUTOVIZ_OGRE_OVERLAY_TARGET ${OGRE_OVERLAY_LIBRARIES})
    set(_AUTOVIZ_OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${OGRE_OVERLAY_INCLUDE_DIRS})
    if(OGRE_PLUGINDIR)
      set(_AUTOVIZ_OGRE_PLUGIN_DIR "${OGRE_PLUGINDIR}")
    endif()
  endif()

  set(_AUTOVIZ_OGRE_RVIZ_MEDIA OFF)
  if(_AUTOVIZ_USE_OGRE_VENDOR)
    set(_AUTOVIZ_OGRE_RVIZ_MEDIA ON)
  elseif(AUTOVIZ_OGRE_ROOT)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(_AUTOVIZ_OGRE_ROOT_PROBE OGRE)
    endif()
    if(_AUTOVIZ_OGRE_ROOT_PROBE_FOUND AND _AUTOVIZ_OGRE_ROOT_PROBE_VERSION MATCHES "^1\\.12")
      set(_AUTOVIZ_OGRE_RVIZ_MEDIA ON)
    endif()
  elseif(OGRE_FOUND AND OGRE_VERSION MATCHES "^1\\.12")
    set(_AUTOVIZ_OGRE_RVIZ_MEDIA ON)
  endif()

  target_compile_definitions(autoviz PRIVATE AUTOVIZ_USE_OGRE)
  target_compile_definitions(autoviz PRIVATE
    AUTOVIZ_OGRE_MEDIA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources/ogre_media")
  if(_AUTOVIZ_OGRE_RVIZ_MEDIA)
    target_compile_definitions(autoviz PRIVATE AUTOVIZ_OGRE_RVIZ_MEDIA)
    message(STATUS "Autoviz point cloud visual: rviz ogre_media GLSL (Ogre 1.12.x)")
  else()
    message(STATUS "Autoviz point cloud visual: stub materials (see autoviz/docs/OGRE_VISUAL_ALIGNMENT.md)")
    message(STATUS "  For rviz-identical shaders: -DAUTOVIZ_OGRE_VENDOR=ON")
  endif()
  if(_AUTOVIZ_OGRE_PLUGIN_DIR)
    target_compile_definitions(autoviz PRIVATE AUTOVIZ_OGRE_PLUGIN_DIR="${_AUTOVIZ_OGRE_PLUGIN_DIR}")
  endif()
  target_include_directories(autoviz SYSTEM PRIVATE ${_AUTOVIZ_OGRE_INCLUDE_DIRS})
  find_package(Eigen3 REQUIRED)
  target_link_libraries(autoviz PRIVATE ${_AUTOVIZ_OGRE_TARGET} ${_AUTOVIZ_OGRE_OVERLAY_TARGET}
    Eigen3::Eigen)
  if(_AUTOVIZ_HAS_ASSIMP)
    if(assimp_FOUND)
      target_link_libraries(autoviz PRIVATE assimp::assimp)
    else()
      target_include_directories(autoviz SYSTEM PRIVATE ${ASSIMP_INCLUDE_DIRS})
      target_link_libraries(autoviz PRIVATE ${ASSIMP_LIBRARIES})
    endif()
    target_compile_definitions(autoviz PRIVATE AUTOVIZ_USE_ASSIMP)
  endif()
endif()

target_compile_features(autoviz PRIVATE cxx_std_17)

if(BUILD_AUTOVIZ)
  add_executable(autoviz_bicmap_publisher
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/bicmap_example_publisher.cpp)
  add_dependencies(autoviz_bicmap_publisher automsgs)
  target_include_directories(autoviz_bicmap_publisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} ${_AUTONOMY_ROOT})
  target_link_libraries(autoviz_bicmap_publisher PRIVATE
    automsgs autolink gflags glog protobuf::libprotobuf)
  target_compile_features(autoviz_bicmap_publisher PRIVATE cxx_std_17)
endif()

set_target_properties(autoviz PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH "\$ORIGIN/../lib:${CMAKE_INSTALL_PREFIX}/lib"
  BUILD_RPATH "\$ORIGIN:\$ORIGIN/../lib:${CMAKE_BINARY_DIR}/lib"
)

if(APPLE)
  set_target_properties(autoviz PROPERTIES
    INSTALL_RPATH "@loader_path/../lib;@loader_path/../lib/autonomy"
    BUILD_RPATH "@loader_path;@loader_path/../lib"
  )
endif()

install(TARGETS autoviz RUNTIME DESTINATION bin)
if(BUILD_AUTOVIZ)
  install(TARGETS autoviz_bicmap_publisher RUNTIME DESTINATION bin)
endif()
install(FILES config/default.autoviz config/turtlebot3_burger.urdf
        DESTINATION share/autonomy/autoviz)
install(DIRECTORY resources/ogre_media
        DESTINATION share/autonomy/autoviz)
install(PROGRAMS scripts/mcap_to_record.py scripts/publish_test_sensors.py
        scripts/publish_bicmap_example.py scripts/run_bicmap_example.sh
        scripts/autoviz_bicmap_publisher scripts/install_linux_desktop.sh
        DESTINATION share/autonomy/autoviz/scripts)

# Linux desktop integration (Ubuntu dock / GNOME app menu).
set(_AUTOVIZ_ICON_SVG ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/aviz.svg)
set(AUTOVIZ_APP_DESCRIPTION "Autolink native 3D visualizer")
string(TIMESTAMP AUTOVIZ_BUILD_DATE "%Y-%m-%d" UTC)
if(NOT AUTOVIZ_VERSION)
  set(AUTOVIZ_VERSION "0.1.0")
endif()
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/deploy/linux/org.autonomy.autoviz.desktop.in
  ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.desktop
  @ONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/deploy/linux/org.autonomy.autoviz.appdata.xml.in
  ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.appdata.xml
  @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.desktop
        DESTINATION share/applications)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.appdata.xml
        DESTINATION share/metainfo)
install(FILES ${_AUTOVIZ_ICON_SVG}
        DESTINATION share/icons/hicolor/scalable/apps
        RENAME aviz.svg)

find_program(RSVG_CONVERT rsvg-convert)
set(_AUTOVIZ_ICON_PNGS "")
if(RSVG_CONVERT)
  foreach(_size 48 64 128 256)
    set(_png_dir ${CMAKE_CURRENT_BINARY_DIR}/icons/hicolor/${_size}x${_size}/apps)
    set(_png ${_png_dir}/aviz.png)
    add_custom_command(
      OUTPUT ${_png}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${_png_dir}
      COMMAND ${RSVG_CONVERT} -w ${_size} -h ${_size}
              ${_AUTOVIZ_ICON_SVG} -o ${_png}
      DEPENDS ${_AUTOVIZ_ICON_SVG}
      COMMENT "Generating aviz ${_size}x${_size} dock icon")
    list(APPEND _AUTOVIZ_ICON_PNGS ${_png})
  endforeach()
  add_custom_target(autoviz_icons ALL DEPENDS ${_AUTOVIZ_ICON_PNGS})
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/icons/hicolor/
          DESTINATION share/icons)
endif()

if(BUILD_AUTOVIZ_TESTS AND AUTOVIZ_USE_OGRE)
  find_package(GTest QUIET)
  set(_AUTOVIZ_TEST_MESHES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/ogre_media_resources")

  function(_autoviz_add_ogre_gtest _target_name)
    add_executable(${_target_name} ${ARGN})
    add_dependencies(${_target_name} automsgs)
    target_include_directories(${_target_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR} ${_AUTONOMY_ROOT})
    target_compile_definitions(${_target_name} PRIVATE
      AUTOVIZ_USE_OGRE AUTOVIZ_OGRE_MEDIA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources/ogre_media"
      AUTOVIZ_TEST_MESHES_DIR="${_AUTOVIZ_TEST_MESHES_DIR}")
    if(_AUTOVIZ_OGRE_RVIZ_MEDIA)
      target_compile_definitions(${_target_name} PRIVATE AUTOVIZ_OGRE_RVIZ_MEDIA)
    endif()
    if(_AUTOVIZ_OGRE_PLUGIN_DIR)
      target_compile_definitions(${_target_name} PRIVATE
        AUTOVIZ_OGRE_PLUGIN_DIR="${_AUTOVIZ_OGRE_PLUGIN_DIR}")
    endif()
    if(_AUTOVIZ_HAS_ASSIMP)
      target_compile_definitions(${_target_name} PRIVATE AUTOVIZ_USE_ASSIMP)
    endif()
    target_include_directories(${_target_name} SYSTEM PRIVATE ${_AUTOVIZ_OGRE_INCLUDE_DIRS})
    target_link_libraries(${_target_name} PRIVATE
      automsgs GTest::gtest Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
      Qt6::Network Eigen3::Eigen
      glog protobuf::libprotobuf ${_AUTOVIZ_OGRE_TARGET} ${_AUTOVIZ_OGRE_OVERLAY_TARGET}
      ${CMAKE_DL_LIBS})
    if(_AUTOVIZ_HAS_ASSIMP)
      if(assimp_FOUND)
        target_link_libraries(${_target_name} PRIVATE assimp::assimp)
      else()
        target_include_directories(${_target_name} SYSTEM PRIVATE ${ASSIMP_INCLUDE_DIRS})
        target_link_libraries(${_target_name} PRIVATE ${ASSIMP_LIBRARIES})
      endif()
    endif()
    target_compile_features(${_target_name} PRIVATE cxx_std_17)
    if(UNIX AND NOT APPLE)
      find_package(X11 QUIET)
      if(X11_FOUND)
        target_link_libraries(${_target_name} PRIVATE X11::X11 GL)
      else()
        target_link_libraries(${_target_name} PRIVATE X11 GL)
      endif()
    endif()
    add_test(NAME ${_target_name} COMMAND ${_target_name})
    if(DEFINED ENV{DISPLAY})
      set(_autoviz_test_display "$ENV{DISPLAY}")
    else()
      set(_autoviz_test_display ":1")
    endif()
    set_tests_properties(${_target_name} PROPERTIES
      ENVIRONMENT "DISPLAY=${_autoviz_test_display}")
  endfunction()

  set(_AUTOVIZ_OGRE_TEST_RENDERING_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/ogre_testing_environment.cpp
    ${_AUTOVIZ_SRC_ROOT}/rendering/render_system.cpp
    ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_resource_config.cpp
    ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_material_manager.cpp
    ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_procedural_shape.cpp
    ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_logging.cpp
    ${_AUTOVIZ_SRC_ROOT}/rendering/gpu_capabilities.cpp
    ${_AUTOVIZ_SRC_ROOT}/display/primitive_mesh.cpp
    ${_AUTOVIZ_SRC_ROOT}/display/obj_mesh.cpp
  )

  add_executable(autoviz_ogre_pick_verify
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/ogre_pick_verify.cpp
    ${_AUTOVIZ_SRC_ROOT}/common/pick_registry.cpp
    ${_AUTOVIZ_SRC_ROOT}/common/pick_handle.cpp)
  target_include_directories(autoviz_ogre_pick_verify PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(autoviz_ogre_pick_verify PRIVATE Qt6::Gui)
  target_compile_features(autoviz_ogre_pick_verify PRIVATE cxx_std_17)
  install(TARGETS autoviz_ogre_pick_verify RUNTIME DESTINATION bin)

  if(GTest_FOUND)
    set(_AUTOVIZ_GTEST_MAIN ${CMAKE_CURRENT_SOURCE_DIR}/tests/gtest_main.cpp)

    set(_AUTOVIZ_MESH_LOADER_TEST_SOURCES
      ${_AUTOVIZ_GTEST_MAIN}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/mesh_loader_test.cpp
      ${_AUTOVIZ_OGRE_TEST_RENDERING_SOURCES}
      ${_AUTOVIZ_SRC_ROOT}/rendering/mesh_resource.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/ogre_mesh_loader.cpp
    )
    if(_AUTOVIZ_HAS_ASSIMP)
      list(APPEND _AUTOVIZ_MESH_LOADER_TEST_SOURCES
        ${_AUTOVIZ_SRC_ROOT}/rendering/mesh_loader_helpers/assimp_loader.cpp)
    endif()
    _autoviz_add_ogre_gtest(autoviz_mesh_loader_test ${_AUTOVIZ_MESH_LOADER_TEST_SOURCES})
    _autoviz_add_ogre_gtest(autoviz_rendering_objects_test
      ${_AUTOVIZ_GTEST_MAIN}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/rendering_objects_test.cpp
      ${_AUTOVIZ_OGRE_TEST_RENDERING_SOURCES}
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_line.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_shape.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_arrow.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_billboard_line.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_wrench_visual.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_screw_visual.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_mesh_shape.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_covariance_visual.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_point_cloud.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_point_cloud_renderable.cpp
    )
    install(TARGETS autoviz_mesh_loader_test autoviz_rendering_objects_test
            RUNTIME DESTINATION bin)
  else()
    message(STATUS "GTest not found; skipping autoviz Ogre gtests")
  endif()
endif()

if(BUILD_AUTOVIZ_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    set(_AUTOVIZ_BICMAP_TEST_SOURCES
      ${_AUTOVIZ_SRC_ROOT}/display/display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/map_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/path_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/point_cloud2_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/point_cloud_utils.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/marker_array_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/marker_draw_utils.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_poi_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_robot_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_semantic_zone_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_fov_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_road_graph_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_canvas_label_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_label_bubble_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_iot_bubble_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_robot3d_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/strata_building_display.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/screen_projection.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/transform_utils.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/ogre_overlay_draw.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/ogre_label_draw.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/ogre_colored_points_draw.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/ogre_mesh_draw.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/primitive_mesh.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/obj_mesh.cpp
      ${_AUTOVIZ_SRC_ROOT}/display/arrow_mesh_utils.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/scene_overlay.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/grid_renderer.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/text_raster_utils.cpp
      ${_AUTOVIZ_SRC_ROOT}/rendering/point_cloud_style_utils.cpp
      ${_AUTOVIZ_SRC_ROOT}/common/config.cpp
      ${_AUTOVIZ_SRC_ROOT}/common/display_property.cpp
      ${_AUTOVIZ_SRC_ROOT}/common/pick_registry.cpp
      ${_AUTOVIZ_SRC_ROOT}/common/pick_handle.cpp
      ${_AUTOVIZ_SRC_ROOT}/common/selection_handler.cpp
      ${_AUTOVIZ_SRC_ROOT}/transform/buffer.cpp
      ${_AUTOVIZ_SRC_ROOT}/transform/buffer_utils.cpp
      ${_AUTOVIZ_SRC_ROOT}/transform/tf2/buffer_core.cpp
      ${_AUTOVIZ_SRC_ROOT}/transform/tf2/cache.cpp
      ${_AUTOVIZ_SRC_ROOT}/transform/tf2/static_cache.cpp
      ${_AUTOVIZ_SRC_ROOT}/transform/tf2/time.cpp
    )

    function(_autoviz_add_display_gtest _target_name)
      add_executable(${_target_name} ${ARGN})
      add_dependencies(${_target_name} automsgs)
      target_include_directories(${_target_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} ${_AUTONOMY_ROOT})
      target_link_libraries(${_target_name} PRIVATE
        automsgs autolink yaml-cpp
        GTest::gtest Qt6::Gui Qt6::Widgets Qt6::OpenGL
        Qt6::Network glog protobuf::libprotobuf ${CMAKE_DL_LIBS})
      target_compile_features(${_target_name} PRIVATE cxx_std_17)
      add_test(NAME ${_target_name} COMMAND ${_target_name})
      if(DEFINED ENV{DISPLAY})
        set(_autoviz_test_display "$ENV{DISPLAY}")
      else()
        set(_autoviz_test_display ":1")
      endif()
      set_tests_properties(${_target_name} PROPERTIES
        ENVIRONMENT "DISPLAY=${_autoviz_test_display}")
    endfunction()

    _autoviz_add_display_gtest(autoviz_bicmap_examples_test
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/bicmap_gtest_main.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/bicmap_examples_test.cpp
      ${_AUTOVIZ_BICMAP_TEST_SOURCES}
    )
    install(TARGETS autoviz_bicmap_examples_test RUNTIME DESTINATION bin)
  endif()
endif()
