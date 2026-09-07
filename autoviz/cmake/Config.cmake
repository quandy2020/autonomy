# =============================================================================
# Build options and toolchain defaults
# =============================================================================

# Application metadata (substituted into .desktop / AppStream templates via @ONLY).
if(NOT AUTOVIZ_APP_NAME)
  set(AUTOVIZ_APP_NAME "Aviz" CACHE STRING "Application display name")
endif()
if(NOT AUTOVIZ_VERSION)
  set(AUTOVIZ_VERSION "0.1.0" CACHE STRING "Application version")
endif()
if(NOT AUTOVIZ_APP_DESCRIPTION)
  set(AUTOVIZ_APP_DESCRIPTION "Autolink native 3D visualizer" CACHE STRING
      "Application description")
endif()

# Feature toggles
option(AUTOVIZ_USE_QML_VEHICLE "Enable Qt Quick 3D vehicle preview panel" ON)
# Legacy CMake name; source still uses AUTOVIZ_USE_QML_DRONE macro.
if(DEFINED AUTOVIZ_USE_QML_DRONE AND NOT AUTOVIZ_USE_QML_DRONE)
  set(AUTOVIZ_USE_QML_VEHICLE OFF)
endif()

option(AUTOVIZ_USE_OGRE "Use Ogre render backend (instead of default OpenGL path)" OFF)
option(AUTOVIZ_OGRE_VENDOR "Build Ogre 1.12.10 from source (RViz point-cloud GLSL)" OFF)
option(AUTOVIZ_OGRE_AUTO_VENDOR "Auto-enable VENDOR when system Ogre is not 1.12.x" OFF)
option(AUTOVIZ_USE_ASSIMP "Use Assimp in Ogre mesh_loader" ON)
set(AUTOVIZ_OGRE_ROOT "" CACHE PATH "Prebuilt Ogre install prefix (optional)")

option(BUILD_AUTOVIZ_TESTS "Build gtest verification targets (dev only, off by default)" OFF)
# C++ tutorial publishers under examples/cpp (also built with target `autoviz`).
option(AUTOVIZ_BUILD_CPP_EXAMPLES "Build C++ tutorial publishers (01–24)" ON)

# Standalone-only: compiler settings and output directories
if(AUTOVIZ_STANDALONE)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  if(NOT BUILD_AUTOVIZ_TESTS)
    set(BUILD_TESTING OFF CACHE INTERNAL "" FORCE)
  endif()
endif()

# Qt code generation (set once globally)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED CMAKE_AUTOGEN_PARALLEL)
  cmake_host_system_information(RESULT _nproc QUERY NUMBER_OF_LOGICAL_CORES)
  set(CMAKE_AUTOGEN_PARALLEL ${_nproc})
endif()
