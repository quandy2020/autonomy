# Embed autodriver as a subdirectory when building the autonomy super-project.

set(AUTODRIVER_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(AUTODRIVER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

#set(AUTODRIVER_BUILD_AUTONOMY_BRIDGE ON CACHE BOOL "" FORCE)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/autodriver/CMakeLists.txt")
  message(FATAL_ERROR
    "autodriver is not present at ${CMAKE_CURRENT_SOURCE_DIR}/autodriver")
endif()

add_subdirectory(autodriver)
