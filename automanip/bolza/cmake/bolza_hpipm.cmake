# Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
#
# BLASFEO + HPIPM for SQP/IPM QP.
# BLASFEO: always FetchContent (no in-tree submodule).
# HPIPM: prefer thirdparty/hpipm submodule; else FetchContent.
# Pins match OCS2 hpipm_catkin / blasfeo_catkin.

include(FetchContent)

set(_BOLZA_TP "${BOLZA_ROOT_DIR}/thirdparty")
set(_BOLZA_HPIPM_SRC "")
if(EXISTS "${_BOLZA_TP}/hpipm/CMakeLists.txt")
  set(_BOLZA_HPIPM_SRC "${_BOLZA_TP}/hpipm")
endif()

# --- BLASFEO (FetchContent only) ---------------------------------------------
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "BLASFEO target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "BLASFEO examples" FORCE)

if(NOT TARGET blasfeo)
  FetchContent_Declare(
    blasfeo
    GIT_REPOSITORY https://github.com/giaf/blasfeo.git
    GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
  )
  FetchContent_MakeAvailable(blasfeo)
endif()
if(NOT TARGET blasfeo)
  message(FATAL_ERROR "bolza: FetchContent blasfeo did not create target 'blasfeo'")
endif()

set(BLASFEO_PATH "${blasfeo_SOURCE_DIR}" CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR "${blasfeo_SOURCE_DIR}/include" CACHE STRING "Path to BLASFEO header files." FORCE)

# --- HPIPM -------------------------------------------------------------------
set(TARGET AVX CACHE STRING "HPIPM target architecture" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "HPIPM tests" FORCE)

if(_BOLZA_HPIPM_SRC)
  message(STATUS "bolza: HPIPM via thirdparty/hpipm submodule")
  if(NOT TARGET hpipm)
    add_subdirectory("${_BOLZA_HPIPM_SRC}" "${CMAKE_CURRENT_BINARY_DIR}/_deps/hpipm")
  endif()
  set(hpipm_SOURCE_DIR "${_BOLZA_HPIPM_SRC}")
else()
  message(STATUS "bolza: HPIPM via FetchContent (no thirdparty/hpipm)")
  FetchContent_Declare(
    hpipm
    GIT_REPOSITORY https://github.com/giaf/hpipm.git
    GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
  )
  FetchContent_MakeAvailable(hpipm)
endif()

if(NOT TARGET hpipm)
  message(FATAL_ERROR "bolza: hpipm target missing after submodule/FetchContent")
endif()

add_library(bolza_hpipm_deps INTERFACE)
target_link_libraries(bolza_hpipm_deps INTERFACE hpipm blasfeo)
target_include_directories(bolza_hpipm_deps INTERFACE
  "${blasfeo_SOURCE_DIR}/include"
  "${hpipm_SOURCE_DIR}/include")

set(BOLZA_HAS_HPIPM ON)
message(STATUS "bolza: HPIPM ON (blasfeo=${blasfeo_SOURCE_DIR}, hpipm=${hpipm_SOURCE_DIR})")
