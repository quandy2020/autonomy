# =============================================================================
# Third-party and in-tree dependencies (autolink + automsgs)
#
# Standalone: add_subdirectory embeds siblings here.
# Super-project: autolink is already added by the parent; skip embed, find Qt only.
# =============================================================================

function(_autoviz_embed_sibling_deps)
  if(TARGET autolink AND TARGET automsgs)
    return()
  endif()
  foreach(_dep autolink automsgs)
    if(NOT EXISTS "${AUTOVIZ_DEPS_ROOT}/${_dep}/CMakeLists.txt")
      message(FATAL_ERROR
        "Missing ${_dep} at ${AUTOVIZ_DEPS_ROOT}/${_dep}\n"
        "Run: git submodule update --init --recursive src/autonomy/${_dep}")
    endif()
  endforeach()

  # Trim autolink sub-build to what autoviz needs (recorder sources, etc.).
  set(AUTOLINK_BUILD_TEST OFF CACHE BOOL "" FORCE)
  set(AUTOLINK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(AUTOLINK_BUILD_TOOLS ON CACHE BOOL "" FORCE)
  set(AUTOLINK_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(AUTOLINK_BUILD_DOCS OFF CACHE BOOL "" FORCE)

  add_subdirectory("${AUTOVIZ_DEPS_ROOT}/autolink" "${CMAKE_BINARY_DIR}/_deps/autolink")
  add_subdirectory("${AUTOVIZ_DEPS_ROOT}/automsgs" "${CMAKE_BINARY_DIR}/_deps/automsgs")
endfunction()

if(AUTOVIZ_STANDALONE)
  list(APPEND CMAKE_MODULE_PATH "${AUTOVIZ_DEPS_ROOT}/cmake/modules")
  _autoviz_embed_sibling_deps()
  include(EnsureProtobuf319)
  find_package(yaml-cpp REQUIRED)
  find_package(Glog REQUIRED)
  find_package(gflags QUIET)
endif()

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets OpenGL Xml Svg Network)

# QML vehicle panel is optional; missing Quick3D disables the feature without failing the build.
set(_AUTOVIZ_HAS_QML OFF)
if(AUTOVIZ_USE_QML_VEHICLE)
  find_package(Qt6 QUIET COMPONENTS Quick QuickWidgets Quick3D Qml)
  if(Qt6Quick_FOUND AND Qt6QuickWidgets_FOUND AND Qt6Quick3D_FOUND AND Qt6Qml_FOUND)
    set(_AUTOVIZ_HAS_QML ON)
  else()
    message(WARNING "Qt6 Quick3D not found; AUTOVIZ_USE_QML_VEHICLE disabled")
    set(AUTOVIZ_USE_QML_VEHICLE OFF)
  endif()
endif()
