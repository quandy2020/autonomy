# Rockship aarch64 cross toolchain for Autonomy
# Pattern: jd_robot/docker/toolchain-docker.cmake + product/joy_mini_debian/aarch64.cmake
#
# Sysroot via SYSROOT_DIR (mount host rootfs at /sysroot).
# Used with: nvidia image + --profile cross

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

if(DEFINED ENV{SYSROOT_DIR} AND NOT "$ENV{SYSROOT_DIR}" STREQUAL "")
  get_filename_component(TARGET_SYSROOT "$ENV{SYSROOT_DIR}" REALPATH)
elseif(EXISTS "/sysroot/usr")
  set(TARGET_SYSROOT "/sysroot")
else()
  message(FATAL_ERROR
    "SYSROOT_DIR is not set and /sysroot is empty. "
    "Mount the board rootfs and export SYSROOT_DIR, e.g.\n"
    "  -e SYSROOT_DIR=/sysroot -v <rootfs>:/sysroot:ro")
endif()

set(CMAKE_SYSROOT "${TARGET_SYSROOT}")

set(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++")
set(CMAKE_AR "aarch64-linux-gnu-ar" CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB "aarch64-linux-gnu-ranlib" CACHE FILEPATH "" FORCE)
set(CMAKE_STRIP "aarch64-linux-gnu-strip" CACHE FILEPATH "" FORCE)
set(CMAKE_LINKER "/usr/bin/aarch64-linux-gnu-ld")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${TARGET_SYSROOT} -fPIC -Wno-error" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${TARGET_SYSROOT} -fPIC -Wno-error -Wno-unused-parameter -Wno-deprecated-declarations" CACHE STRING "" FORCE)

set(RPATH_LINK_FLAGS "-Wl,-rpath-link,${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu:${TARGET_SYSROOT}/usr/local/lib:${TARGET_SYSROOT}/opt/ros/jazzy/lib")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${RPATH_LINK_FLAGS} --sysroot=${TARGET_SYSROOT}" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${RPATH_LINK_FLAGS} --sysroot=${TARGET_SYSROOT}" CACHE STRING "" FORCE)
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}" CACHE STRING "" FORCE)

set(CMAKE_FIND_ROOT_PATH
  ${TARGET_SYSROOT}
  ${TARGET_SYSROOT}/usr
  ${TARGET_SYSROOT}/usr/local
  ${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu
  ${TARGET_SYSROOT}/opt/ros/jazzy
)

set(CMAKE_PREFIX_PATH
  "${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake"
  "${TARGET_SYSROOT}/usr/local/lib/cmake"
  "${TARGET_SYSROOT}/opt/ros/jazzy"
)

set(CMAKE_IGNORE_PATH
  "/usr/lib/x86_64-linux-gnu"
  "/usr/lib"
  "/usr/local/lib"
)

set(ENV{PKG_CONFIG_DIR} "")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${TARGET_SYSROOT}")
set(ENV{PKG_CONFIG_LIBDIR} "${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${TARGET_SYSROOT}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_PATH} "${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Host python for codegen; target libs/headers from sysroot when present.
set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "" FORCE)
if(EXISTS "${TARGET_SYSROOT}/usr/include/python3.11")
  set(Python3_INCLUDE_DIR "${TARGET_SYSROOT}/usr/include/python3.11" CACHE PATH "" FORCE)
endif()
if(EXISTS "${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.11.so.1.0")
  set(Python3_LIBRARY "${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.11.so.1.0" CACHE FILEPATH "" FORCE)
endif()
