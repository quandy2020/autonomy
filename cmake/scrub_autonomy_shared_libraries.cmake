# Remove truncated shared libraries under lib_dir (interrupted links).
if(NOT DEFINED lib_dir)
  message(FATAL_ERROR "scrub_autonomy_shared_libraries.cmake: lib_dir not set")
endif()

file(GLOB _scrub_libs
  "${lib_dir}/libautonomy.so"
  "${lib_dir}/libautolink.so"
  "${lib_dir}/libautomsgs.so"
  "${lib_dir}/libautomsgs.so.*")
foreach(_scrub_lib IN LISTS _scrub_libs)
  if(EXISTS "${_scrub_lib}" AND NOT IS_DIRECTORY "${_scrub_lib}")
    file(SIZE "${_scrub_lib}" _scrub_so_size)
    if(_scrub_so_size LESS 1024)
      message(WARNING
        "Removing corrupt shared library '${_scrub_lib}' (${_scrub_so_size} bytes)")
      file(REMOVE "${_scrub_lib}")
    endif()
  endif()
endforeach()
