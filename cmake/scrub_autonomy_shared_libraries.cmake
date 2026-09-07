# Remove truncated shared libraries and empty object files left by interrupted builds.
# Make treats existing empty .o as up-to-date, which then links as "undefined reference to main"
# or produces tiny stub .so files.

if(NOT DEFINED lib_dir)
  message(FATAL_ERROR "scrub_autonomy_shared_libraries.cmake: lib_dir not set")
endif()

# Known package shared libs: a healthy build is multi-MB; stubs are ~15–50KB.
set(_scrub_min_so_bytes 100000)
file(GLOB _scrub_libs
  "${lib_dir}/libautonomy.so"
  "${lib_dir}/libautonomy.so.*"
  "${lib_dir}/libautolink.so"
  "${lib_dir}/libautolink.so.*"
  "${lib_dir}/libautomsgs.so"
  "${lib_dir}/libautomsgs.so.*")
foreach(_scrub_lib IN LISTS _scrub_libs)
  if(EXISTS "${_scrub_lib}" AND NOT IS_DIRECTORY "${_scrub_lib}" AND NOT IS_SYMLINK "${_scrub_lib}")
    file(SIZE "${_scrub_lib}" _scrub_so_size)
    if(_scrub_so_size LESS _scrub_min_so_bytes)
      message(WARNING
        "Removing corrupt shared library '${_scrub_lib}' (${_scrub_so_size} bytes)")
      file(REMOVE "${_scrub_lib}")
    endif()
  endif()
endforeach()

# Empty object files poison incremental builds.
if(DEFINED binary_dir AND EXISTS "${binary_dir}")
  file(GLOB_RECURSE _scrub_objs "${binary_dir}/*.o")
  foreach(_scrub_obj IN LISTS _scrub_objs)
    if(EXISTS "${_scrub_obj}" AND NOT IS_DIRECTORY "${_scrub_obj}")
      file(SIZE "${_scrub_obj}" _scrub_obj_size)
      if(_scrub_obj_size EQUAL 0)
        message(WARNING "Removing empty object file '${_scrub_obj}'")
        file(REMOVE "${_scrub_obj}")
      endif()
    endif()
  endforeach()
endif()
