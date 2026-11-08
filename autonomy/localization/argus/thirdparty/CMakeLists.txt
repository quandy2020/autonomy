# Argus thirdparty — git submodule (gtsam)
#
# Default: do not build (use find_package / system installs).
# Parent option: ARGUS_BUILD_VENDORED_GTSAM.

set(_ARGUS_TP "${CMAKE_CURRENT_SOURCE_DIR}")

# --- GTSAM -----------------------------------------------------------------
if(ARGUS_USE_GTSAM AND ARGUS_BUILD_VENDORED_GTSAM
   AND NOT TARGET gtsam
   AND EXISTS "${_ARGUS_TP}/gtsam/CMakeLists.txt")
  message(STATUS "Argus: building vendored GTSAM from thirdparty/gtsam")
  set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_TIMING OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
  set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
  add_subdirectory("${_ARGUS_TP}/gtsam" "${CMAKE_BINARY_DIR}/thirdparty/gtsam"
    EXCLUDE_FROM_ALL)
  if(TARGET gtsam)
    set(GTSAM_FOUND TRUE PARENT_SCOPE)
    set(GTSAM_FOUND TRUE)
  endif()
endif()
