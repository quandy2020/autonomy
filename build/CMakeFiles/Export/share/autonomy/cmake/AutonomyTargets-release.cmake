#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "autonomy" for configuration "Release"
set_property(TARGET autonomy APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(autonomy PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libautonomy.so"
  IMPORTED_SONAME_RELEASE "libautonomy.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS autonomy )
list(APPEND _IMPORT_CHECK_FILES_FOR_autonomy "${_IMPORT_PREFIX}/lib/libautonomy.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
