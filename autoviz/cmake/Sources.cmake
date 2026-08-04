# =============================================================================
# Collect sources for the autoviz executable
#
# Default: GLOB autoviz/*.cpp. Ogre TUs are listed explicitly so they can be
# excluded when AUTOVIZ_USE_OGRE=OFF (GLOB is evaluated at configure time).
# =============================================================================

set(_AUTOVIZ_OGRE_SOURCES
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

file(GLOB_RECURSE AUTOVIZ_SOURCES CONFIGURE_DEPENDS "${AUTOVIZ_SRC_ROOT}/*.cpp")
file(GLOB_RECURSE AUTOVIZ_HEADERS CONFIGURE_DEPENDS "${AUTOVIZ_SRC_ROOT}/*.hpp")

if(AUTOVIZ_USE_OGRE AND AUTOVIZ_USE_ASSIMP)
  find_package(assimp QUIET)
  if(NOT assimp_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(ASSIMP assimp)
    endif()
  endif()
  if(assimp_FOUND OR ASSIMP_FOUND)
    list(APPEND _AUTOVIZ_OGRE_SOURCES
      ${AUTOVIZ_SRC_ROOT}/rendering/mesh_loader_helpers/assimp_loader.cpp)
    set(_AUTOVIZ_HAS_ASSIMP ON)
  else()
    message(WARNING "Assimp not found; mesh_loader supports OBJ/STL/.mesh only")
  endif()
endif()

if(AUTOVIZ_USE_OGRE)
  foreach(_src ${_AUTOVIZ_OGRE_SOURCES})
    if(EXISTS "${_src}" AND NOT "${_src}" IN_LIST AUTOVIZ_SOURCES)
      list(APPEND AUTOVIZ_SOURCES "${_src}")
    endif()
  endforeach()
else()
  foreach(_src ${_AUTOVIZ_OGRE_SOURCES})
    list(REMOVE_ITEM AUTOVIZ_SOURCES "${_src}")
  endforeach()
  list(REMOVE_ITEM AUTOVIZ_SOURCES
    "${AUTOVIZ_SRC_ROOT}/rendering/viewport_projection_finder.cpp")
  list(REMOVE_ITEM AUTOVIZ_HEADERS
    "${AUTOVIZ_SRC_ROOT}/rendering/viewport_projection_finder.hpp")
endif()

# Playback panel compiles autolink recorder sources directly (no internal lib link).
set(AUTOVIZ_RECORDER_SOURCES
  ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/player.cpp
  ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task.cpp
  ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task_buffer.cpp
  ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task_consumer.cpp
  ${AUTOVIZ_DEPS_ROOT}/autolink/autolink/tools/recorder/player/play_task_producer.cpp
)
