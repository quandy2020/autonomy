# Embed autolink as a subdirectory with fixed build options.

set(AUTOLINK_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(AUTOLINK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(AUTOLINK_BUILD_TOOLS ON CACHE BOOL "" FORCE)
option(AUTOLINK_BUILD_PYTHON "Build autolink Python bindings" ON)
set(AUTOLINK_BUILD_DOCS OFF CACHE BOOL "" FORCE)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/autolink/CMakeLists.txt")
  message(FATAL_ERROR
    "autolink submodule is not initialized.\n"
    "From the repository root, run:\n"
    "  git submodule update --init --recursive autolink")
endif()

add_subdirectory(autolink)
