# Embed autodriver as a subdirectory when building the autonomy super-project.
# Leave AUTODRIVER_BUILD_TEST / EXAMPLES to the caller.

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/autodriver/CMakeLists.txt")
  message(FATAL_ERROR
    "autodriver is not present at ${CMAKE_CURRENT_SOURCE_DIR}/autodriver")
endif()

add_subdirectory(autodriver)
