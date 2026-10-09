# Autoviz CMake helpers (included from the package CMakeLists.txt).

include_guard(GLOBAL)

# ---------------------------------------------------------------------------
# Paths, options, Qt AUTOMOC, output dirs.
# ---------------------------------------------------------------------------
macro(autoviz_setup)
  set(AUTOVIZ_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
  get_filename_component(AUTOVIZ_DEPS_ROOT "${AUTOVIZ_ROOT}/.." ABSOLUTE)
  set(AUTOVIZ_SRC_ROOT "${AUTOVIZ_ROOT}/autoviz")

  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(AUTOVIZ_STANDALONE ON)
  else()
    set(AUTOVIZ_STANDALONE OFF)
  endif()

  if(NOT AUTOVIZ_APP_NAME)
    set(AUTOVIZ_APP_NAME "Aviz" CACHE STRING "Application display name")
  endif()
  if(NOT AUTOVIZ_APP_DESCRIPTION)
    set(AUTOVIZ_APP_DESCRIPTION "Autolink native 3D visualizer" CACHE STRING
        "Application description")
  endif()
  set(AUTOVIZ_VERSION "${PROJECT_VERSION}" CACHE STRING "Application version" FORCE)

  # Viewport backend is Ogre 1.x only (no pure OpenGL RenderWindow path).
  option(AUTOVIZ_OGRE_VENDOR "Build Ogre 1.12.10 from source (RViz point-cloud GLSL)" OFF)
  option(AUTOVIZ_OGRE_AUTO_VENDOR
    "Auto FetchContent Ogre 1.12.10 when system Ogre is missing or not 1.12.x" ON)
  option(AUTOVIZ_USE_ASSIMP "Use Assimp in Ogre mesh_loader" ON)
  set(AUTOVIZ_OGRE_ROOT "" CACHE PATH "Prebuilt Ogre install prefix (optional)")

  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  if(NOT DEFINED CMAKE_AUTOGEN_PARALLEL)
    cmake_host_system_information(RESULT _nproc QUERY NUMBER_OF_LOGICAL_CORES)
    set(CMAKE_AUTOGEN_PARALLEL ${_nproc})
  endif()

  if(AUTOVIZ_STANDALONE)
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()
    set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
  endif()

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

  if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  endif()
endmacro()

# ---------------------------------------------------------------------------
# Embed sibling autolink/automsgs when configuring standalone.
# ---------------------------------------------------------------------------
macro(autoviz_embed_siblings)
  if(AUTOVIZ_STANDALONE)
    list(PREPEND CMAKE_MODULE_PATH "${AUTOVIZ_DEPS_ROOT}/cmake/modules")
    include(EnsureProtobuf319)
    autonomy_require_protobuf()
    foreach(_dep autolink automsgs)
      if(NOT EXISTS "${AUTOVIZ_DEPS_ROOT}/${_dep}/CMakeLists.txt")
        message(FATAL_ERROR
          "Missing ${_dep} at ${AUTOVIZ_DEPS_ROOT}/${_dep}\n"
          "Run: git submodule update --init --recursive")
      endif()
    endforeach()
    if(NOT TARGET autolink)
      set(AUTOLINK_BUILD_TEST OFF CACHE BOOL "" FORCE)
      set(AUTOLINK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
      set(AUTOLINK_BUILD_TOOLS ON CACHE BOOL "" FORCE)
      set(AUTOLINK_BUILD_PYTHON ON CACHE BOOL "Build autolink Python bindings" FORCE)
      set(AUTOLINK_BUILD_DOCS OFF CACHE BOOL "" FORCE)
      add_subdirectory("${AUTOVIZ_DEPS_ROOT}/autolink" "${CMAKE_BINARY_DIR}/_deps/autolink")
    endif()
    if(NOT TARGET automsgs)
      add_subdirectory("${AUTOVIZ_DEPS_ROOT}/automsgs" "${CMAKE_BINARY_DIR}/_deps/automsgs")
    endif()
  endif()

  _autoviz_stub_find_config(autolink)
  _autoviz_stub_find_config(automsgs)

  if(TARGET glog::glog AND NOT TARGET glog)
    add_library(glog ALIAS glog::glog)
  endif()
endmacro()

# ---------------------------------------------------------------------------
# Embedded sibling packages: satisfy find_package() after add_subdirectory().
# ---------------------------------------------------------------------------
function(_autoviz_stub_find_config name)
  if(NOT TARGET ${name})
    return()
  endif()
  set(_dir "${CMAKE_CURRENT_BINARY_DIR}/_embedded_cmake/${name}")
  file(MAKE_DIRECTORY "${_dir}")
  file(WRITE "${_dir}/${name}Config.cmake" "set(${name}_FOUND TRUE)\n")
  list(PREPEND CMAKE_PREFIX_PATH "${_dir}")
  set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# Collect library sources (GLOB + optional Ogre / Assimp units).
#
# Sets in parent scope:
#   AUTOVIZ_SOURCES, AUTOVIZ_HEADERS, AUTOVIZ_RECORDER_SOURCES
#   _AUTOVIZ_HAS_ASSIMP (optional)
# ---------------------------------------------------------------------------
function(autoviz_collect_sources)
  set(_ogre_sources
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_movable_text.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_billboard_line.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_shape.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_line.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_arrow.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_wrench_visual.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_screw_visual.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_effort_visual.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_covariance_visual.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_triangle_polygon.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_mesh_shape.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/geometry.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/orthographic.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/ogre_logging.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/ogre_mesh_loader.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/mesh_resource.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/ogre_indexed_palette.cpp
    ${AUTOVIZ_SRC_ROOT}/display/ogre_pbr_mesh_draw.cpp
    ${AUTOVIZ_SRC_ROOT}/display/ogre_entity_draw.cpp
  )

  file(GLOB_RECURSE _sources CONFIGURE_DEPENDS "${AUTOVIZ_SRC_ROOT}/*.cpp")
  file(GLOB_RECURSE _headers CONFIGURE_DEPENDS "${AUTOVIZ_SRC_ROOT}/*.hpp")

  set(_has_assimp OFF)
  if(AUTOVIZ_USE_ASSIMP)
    find_package(assimp QUIET)
    if(NOT assimp_FOUND)
      find_package(PkgConfig QUIET)
      if(PkgConfig_FOUND)
        pkg_check_modules(ASSIMP assimp)
      endif()
    endif()
    if(assimp_FOUND OR ASSIMP_FOUND)
      list(APPEND _ogre_sources
        ${AUTOVIZ_SRC_ROOT}/rendering/mesh_loader_helpers/assimp_loader.cpp)
      set(_has_assimp ON)
    else()
      message(WARNING "Assimp not found; mesh_loader supports OBJ/STL/.mesh only")
    endif()
  endif()

  foreach(_src ${_ogre_sources})
    if(EXISTS "${_src}" AND NOT "${_src}" IN_LIST _sources)
      list(APPEND _sources "${_src}")
    endif()
  endforeach()

  list(REMOVE_ITEM _sources "${AUTOVIZ_SRC_ROOT}/main.cpp")
  # Viewport is Ogre 1.x only — do not compile the retired QOpenGLWidget path.
  list(REMOVE_ITEM _sources
    "${AUTOVIZ_SRC_ROOT}/rendering/render_window.cpp")
  list(REMOVE_ITEM _headers
    "${AUTOVIZ_SRC_ROOT}/rendering/render_window.hpp")

  set(_recorder
    ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/player.cpp
    ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task.cpp
    ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task_buffer.cpp
    ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task_consumer.cpp
    ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task_producer.cpp
  )

  set(AUTOVIZ_SOURCES "${_sources}" PARENT_SCOPE)
  set(AUTOVIZ_HEADERS "${_headers}" PARENT_SCOPE)
  set(AUTOVIZ_RECORDER_SOURCES "${_recorder}" PARENT_SCOPE)
  if(_has_assimp)
    set(_AUTOVIZ_HAS_ASSIMP ON PARENT_SCOPE)
  endif()
  # assimp find results needed by autoviz_apply_ogre_backend
  if(assimp_FOUND)
    set(assimp_FOUND "${assimp_FOUND}" PARENT_SCOPE)
  endif()
  if(ASSIMP_FOUND)
    set(ASSIMP_FOUND "${ASSIMP_FOUND}" PARENT_SCOPE)
    set(ASSIMP_INCLUDE_DIRS "${ASSIMP_INCLUDE_DIRS}" PARENT_SCOPE)
    set(ASSIMP_LIBRARIES "${ASSIMP_LIBRARIES}" PARENT_SCOPE)
  endif()
endfunction()

# ---------------------------------------------------------------------------
# Ogre 1.x render backend (required).
# Resolution order: VENDOR (source) > AUTOVIZ_OGRE_ROOT > system pkg-config.
# ---------------------------------------------------------------------------
function(autoviz_apply_ogre_backend _target)
  set(_vendor ${AUTOVIZ_OGRE_VENDOR})
  if(NOT _vendor AND NOT AUTOVIZ_OGRE_ROOT)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(_ogre_probe OGRE)
    endif()
    if(AUTOVIZ_OGRE_AUTO_VENDOR AND
       (NOT _ogre_probe_FOUND OR NOT _ogre_probe_VERSION MATCHES "^1\\.12"))
      message(STATUS "System Ogre is not 1.12.x; enabling AUTOVIZ_OGRE_VENDOR")
      set(_vendor ON)
    endif()
  endif()

  if(_vendor)
    message(STATUS "Autoviz: building Ogre 1.12.10 via FetchContent")
    include(FetchContent)
    # Ogre 1.12 + its Dependencies still declare cmake_minimum_required < 3.5.
    # CMake 4+ rejects that unless this (env + cache) is set for nested configures.
    set(ENV{CMAKE_POLICY_VERSION_MINIMUM} "3.5")
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
    set(OGRE_BUILD_COMPONENT_BITES OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_COMPONENT_PYTHON OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_COMPONENT_JAVA OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_COMPONENT_CSHARP OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_COMPONENT_OVERLAY_IMGUI OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_SAMPLES FALSE CACHE BOOL "" FORCE)
    set(OGRE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(OGRE_CONFIG_THREADS 0 CACHE STRING "" FORCE)
    set(OGRE_RESOURCEMANAGER_STRICT 2 CACHE STRING "" FORCE)
    set(OGRE_BUILD_RENDERSYSTEM_GL TRUE CACHE BOOL "" FORCE)
    set(OGRE_BUILD_RENDERSYSTEM_D3D11 OFF CACHE BOOL "" FORCE)
    set(OGRE_BUILD_RENDERSYSTEM_D3D9 OFF CACHE BOOL "" FORCE)
    # Frameworks use $(PLATFORM_NAME)/$(CONFIGURATION) post-build scripts that
    # only work with the Xcode generator; Ninja needs plain dylibs.
    if(APPLE)
      set(OGRE_BUILD_LIBS_AS_FRAMEWORKS OFF CACHE BOOL "" FORCE)
      # Ogre may otherwise leave CMAKE_OSX_SYSROOT as the bare "macosx" token,
      # which breaks Command Line Tools builds (-isysroot macosx).
      if(NOT CMAKE_OSX_SYSROOT OR CMAKE_OSX_SYSROOT STREQUAL "macosx")
        execute_process(
          COMMAND xcrun --sdk macosx --show-sdk-path
          OUTPUT_VARIABLE _ogre_sdk
          OUTPUT_STRIP_TRAILING_WHITESPACE
          ERROR_QUIET)
        if(_ogre_sdk)
          set(CMAKE_OSX_SYSROOT "${_ogre_sdk}" CACHE PATH "" FORCE)
          message(STATUS "Autoviz: CMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT}")
        endif()
      endif()
    endif()
    # Prefer a vendored tarball (avoids flaky git clone to github.com).
    set(_ogre_tarball "${AUTOVIZ_ROOT}/thirdparty/ogre-1.12.10.tar.gz")
    if(EXISTS "${_ogre_tarball}")
      message(STATUS "Autoviz: using local Ogre tarball ${_ogre_tarball}")
      FetchContent_Declare(autoviz_ogre
        URL "file://${_ogre_tarball}"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        PATCH_COMMAND bash "${AUTOVIZ_ROOT}/cmake/apply_ogre_patches.sh" <SOURCE_DIR>)
    else()
      message(STATUS "Autoviz: downloading Ogre 1.12.10 release tarball")
      FetchContent_Declare(autoviz_ogre
        URL https://github.com/OGRECave/ogre/archive/refs/tags/v1.12.10.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        PATCH_COMMAND bash "${AUTOVIZ_ROOT}/cmake/apply_ogre_patches.sh" <SOURCE_DIR>)
    endif()
    FetchContent_MakeAvailable(autoviz_ogre)
    # OgreMain mixes .cpp and .mm; CMake PCH built as C++ breaks ObjC++ compiles.
    if(APPLE AND TARGET OgreMain)
      set_target_properties(OgreMain PROPERTIES DISABLE_PRECOMPILE_HEADERS ON)
    endif()
    set(_ogre_lib OgreMain)
    set(_ogre_overlay OgreOverlay)
    set(_ogre_includes "${autoviz_ogre_SOURCE_DIR}/OgreMain/include")
    if(EXISTS "${autoviz_ogre_BINARY_DIR}/include")
      list(APPEND _ogre_includes "${autoviz_ogre_BINARY_DIR}/include")
    endif()
    # Stage plugins next to the autonomy binary tree so runtime always finds
    # them (FetchContent's lib/ may be empty until Ogre finishes linking).
    set(_ogre_plugin_stage "${CMAKE_BINARY_DIR}/lib")
    if(APPLE)
      set(_ogre_plugin_build "${autoviz_ogre_BINARY_DIR}/lib/macosx")
    else()
      set(_ogre_plugin_build "${autoviz_ogre_BINARY_DIR}/lib")
    endif()
    set(_ogre_plugins "${_ogre_plugin_stage}")
    # OgreMain alone is not enough — RenderSystem_* must be built and loadable.
    foreach(_plug IN ITEMS RenderSystem_GL RenderSystem_GL3Plus Codec_STBI)
      if(TARGET ${_plug})
        add_dependencies(${_target} ${_plug})
        add_custom_command(TARGET ${_target} POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory "${_ogre_plugin_stage}"
          COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${_plug}>" "${_ogre_plugin_stage}/"
          COMMENT "Stage Ogre plugin ${_plug} -> ${_ogre_plugin_stage}")
      endif()
    endforeach()
    # Also keep the build-tree plugin dir as a runtime fallback via env docs.
    if(NOT _ogre_plugin_build STREQUAL _ogre_plugin_stage)
      set(_ogre_plugins "${_ogre_plugin_stage}")
    endif()
    set(_rviz_media ON)
  elseif(AUTOVIZ_OGRE_ROOT)
    find_path(_inc OGRE/Ogre.h HINTS "${AUTOVIZ_OGRE_ROOT}/include" "${AUTOVIZ_OGRE_ROOT}/include/OGRE")
    find_library(_lib OgreMain HINTS "${AUTOVIZ_OGRE_ROOT}/lib" "${AUTOVIZ_OGRE_ROOT}/lib64")
    find_library(_overlay OgreOverlay HINTS "${AUTOVIZ_OGRE_ROOT}/lib" "${AUTOVIZ_OGRE_ROOT}/lib64")
    if(NOT _inc OR NOT _lib)
      message(FATAL_ERROR "AUTOVIZ_OGRE_ROOT does not contain OgreMain")
    endif()
    set(_ogre_includes "${_inc}")
    set(_ogre_lib "${_lib}")
    set(_ogre_overlay "${_overlay}")
    set(_ogre_plugins "${AUTOVIZ_OGRE_ROOT}/lib/OGRE")
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(_probe OGRE)
    endif()
    set(_rviz_media OFF)
    if(_probe_FOUND AND _probe_VERSION MATCHES "^1\\.12")
      set(_rviz_media ON)
    endif()
  else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OGRE REQUIRED OGRE)
    pkg_check_modules(OGRE_OVERLAY REQUIRED OGRE-Overlay)
    set(_ogre_lib ${OGRE_LIBRARIES})
    set(_ogre_overlay ${OGRE_OVERLAY_LIBRARIES})
    set(_ogre_includes ${OGRE_INCLUDE_DIRS} ${OGRE_OVERLAY_INCLUDE_DIRS})
    if(OGRE_PLUGINDIR)
      set(_ogre_plugins "${OGRE_PLUGINDIR}")
    endif()
    set(_rviz_media OFF)
    if(OGRE_VERSION MATCHES "^1\\.12")
      set(_rviz_media ON)
    endif()
  endif()

  target_compile_definitions(${_target} PRIVATE
    AUTOVIZ_OGRE_MEDIA_DIR="${AUTOVIZ_ROOT}/resources/ogre_media")
  if(_rviz_media)
    target_compile_definitions(${_target} PRIVATE AUTOVIZ_OGRE_AVIZ_MEDIA)
    message(STATUS "Point cloud shaders: rviz ogre_media (Ogre 1.12.x)")
  else()
    message(STATUS "Point cloud shaders: stub materials (use -DAUTOVIZ_OGRE_VENDOR=ON for RViz parity)")
  endif()
  if(_ogre_plugins)
    target_compile_definitions(${_target} PRIVATE AUTOVIZ_OGRE_PLUGIN_DIR="${_ogre_plugins}")
  endif()

  target_include_directories(${_target} SYSTEM PRIVATE ${_ogre_includes})
  find_package(Eigen3 REQUIRED)
  target_link_libraries(${_target} PRIVATE ${_ogre_lib} ${_ogre_overlay} Eigen3::Eigen)

  if(_AUTOVIZ_HAS_ASSIMP)
    if(assimp_FOUND)
      target_link_libraries(${_target} PRIVATE assimp::assimp)
    else()
      target_include_directories(${_target} SYSTEM PRIVATE ${ASSIMP_INCLUDE_DIRS})
      target_link_libraries(${_target} PRIVATE ${ASSIMP_LIBRARIES})
    endif()
    target_compile_definitions(${_target} PRIVATE AUTOVIZ_USE_ASSIMP)
  endif()
endfunction()

# ---------------------------------------------------------------------------
# Desktop entry, AppStream metadata, and multi-size icons.
# ---------------------------------------------------------------------------
function(autoviz_install_desktop_assets)
  install(FILES ${AUTOVIZ_ROOT}/config/default.autoviz
    DESTINATION share/autonomy/autoviz)
  install(DIRECTORY ${AUTOVIZ_ROOT}/resources/ogre_media
    DESTINATION share/autonomy/autoviz)

  string(TIMESTAMP AUTOVIZ_BUILD_DATE "%Y-%m-%d" UTC)
  configure_file(${AUTOVIZ_ROOT}/deploy/linux/org.autonomy.autoviz.desktop.in
    ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.desktop @ONLY)
  configure_file(${AUTOVIZ_ROOT}/deploy/linux/org.autonomy.autoviz.appdata.xml.in
    ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.appdata.xml @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.desktop
    DESTINATION share/applications)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.appdata.xml
    DESTINATION share/metainfo)

  # Same squirrel artwork as macOS (aviz.png / aviz_*.png / aviz.icns).
  set(_icon_png ${AUTOVIZ_ROOT}/resources/icons/aviz.png)
  set(_icon_svg ${AUTOVIZ_ROOT}/resources/icons/aviz.svg)
  if(EXISTS ${_icon_svg})
    install(FILES ${_icon_svg}
      DESTINATION share/icons/hicolor/scalable/apps RENAME aviz.svg)
  endif()

  set(_icon_sizes 32 48 64 128 256 512)
  set(_png_outputs "")
  foreach(_size IN LISTS _icon_sizes)
    set(_dir ${CMAKE_CURRENT_BINARY_DIR}/icons/hicolor/${_size}x${_size}/apps)
    set(_png ${_dir}/aviz.png)
    set(_src "")
    if(EXISTS ${AUTOVIZ_ROOT}/resources/icons/aviz_${_size}.png)
      set(_src ${AUTOVIZ_ROOT}/resources/icons/aviz_${_size}.png)
    elseif(EXISTS ${_icon_png})
      set(_src ${_icon_png})
    endif()
    if(NOT _src)
      continue()
    endif()
    add_custom_command(OUTPUT ${_png}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${_dir}
      COMMAND ${CMAKE_COMMAND} -E copy ${_src} ${_png}
      DEPENDS ${_src} COMMENT "Install aviz ${_size}x${_size} icon")
    list(APPEND _png_outputs ${_png})
  endforeach()
  if(EXISTS ${AUTOVIZ_ROOT}/resources/icons/aviz_1024.png)
    set(_dir ${CMAKE_CURRENT_BINARY_DIR}/icons/hicolor/1024x1024/apps)
    set(_png ${_dir}/aviz.png)
    add_custom_command(OUTPUT ${_png}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${_dir}
      COMMAND ${CMAKE_COMMAND} -E copy
        ${AUTOVIZ_ROOT}/resources/icons/aviz_1024.png ${_png}
      DEPENDS ${AUTOVIZ_ROOT}/resources/icons/aviz_1024.png
      COMMENT "Install aviz 1024x1024 icon")
    list(APPEND _png_outputs ${_png})
  endif()
  if(NOT _png_outputs)
    return()
  endif()
  add_custom_target(autoviz_icons ALL DEPENDS ${_png_outputs})
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/icons/hicolor/ DESTINATION share/icons)
endfunction()

# ---------------------------------------------------------------------------
# Post-create wiring for libautoviz: Qt / FFmpeg / Ogre / i18n.
# ---------------------------------------------------------------------------
function(autoviz_finalize_library _target)
  target_include_directories(${_target} PRIVATE ${AUTOVIZ_DEPS_ROOT})
  add_dependencies(${_target} automsgs)

  target_link_libraries(${_target} PRIVATE
    yaml-cpp
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
    Qt6::Xml Qt6::Svg Qt6::Network
    glog::glog protobuf::libprotobuf)

  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(AUTOVIZ_FFMPEG QUIET IMPORTED_TARGET
      libavcodec libavutil libswscale)
  endif()
  if(AUTOVIZ_FFMPEG_FOUND)
    target_compile_definitions(${_target} PRIVATE AUTOVIZ_USE_FFMPEG)
    target_link_libraries(${_target} PRIVATE PkgConfig::AUTOVIZ_FFMPEG)
    message(STATUS "Autoviz: FFmpeg video decoding enabled")
  else()
    message(STATUS "Autoviz: FFmpeg not found; H264/H265/VP9 decoding disabled")
  endif()

  if(UNIX OR APPLE)
    target_link_libraries(${_target} PRIVATE ${CMAKE_DL_LIBS})
  endif()
  if(WIN32)
    target_link_libraries(${_target} PRIVATE kernel32)
  endif()

  autoviz_apply_ogre_backend(${_target})

  # Ubuntu splits LinguistTools: qt6-tools-dev ships CMake targets that
  # require qt6-l10n-tools + qt6-tools-dev-tools binaries. Probe the
  # binaries first — find_package aborts hard if any IMPORTED path is gone.
  set(_autoviz_qt_bin "")
  foreach(_prefix
      "${Qt6_DIR}/../../../lib/qt6/bin"
      "/usr/lib/qt6/bin")
    if(EXISTS "${_prefix}/lconvert" AND EXISTS "${_prefix}/lrelease")
      set(_autoviz_qt_bin "${_prefix}")
      break()
    endif()
  endforeach()
  if(_autoviz_qt_bin AND EXISTS "/usr/lib/qt6/libexec/lprodump")
    find_package(Qt6 QUIET COMPONENTS LinguistTools)
  endif()
  if(Qt6LinguistTools_FOUND)
    qt_add_translations(${_target}
      TS_FILES ${AUTOVIZ_ROOT}/translations/autoviz_zh_CN.ts
      RESOURCE_PREFIX "/i18n")
  else()
    message(WARNING
      "Autoviz: Qt6 lrelease/lconvert incomplete; translations skipped "
      "(install qt6-l10n-tools qt6-tools-dev-tools)")
  endif()
endfunction()

function(autoviz_print_summary)
  if(AUTOVIZ_STANDALONE)
    set(_mode "standalone")
  else()
    set(_mode "super-project")
  endif()
  message(STATUS "")
  message(STATUS
    "Autoviz ${PROJECT_VERSION} (${_mode}): viewport=Ogre1.x "
    "vendor=${AUTOVIZ_OGRE_VENDOR} auto_vendor=${AUTOVIZ_OGRE_AUTO_VENDOR}")
  message(STATUS "")
endfunction()
