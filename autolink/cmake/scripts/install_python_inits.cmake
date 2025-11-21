# Ensure __init__.py files exist for Python packages during installation

set(_root "${CMAKE_INSTALL_PREFIX}/python")
file(MAKE_DIRECTORY "${_root}")
if(NOT EXISTS "${_root}/__init__.py")
  file(WRITE "${_root}/__init__.py" "# namespace package\n")
endif()

set(_autolink "${_root}/autolink")
file(MAKE_DIRECTORY "${_autolink}")
if(NOT EXISTS "${_autolink}/__init__.py")
  file(WRITE "${_autolink}/__init__.py" "# autolink package\n")
endif()

set(_subdirs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  map_msgs
  nav_msgs
  pcl_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  stereo_msgs
  trajectory_msgs
  vehicle_msgs
  vision_msgs
  proto
  examples
)
foreach(_d ${_subdirs})
  set(_p "${_autolink}/${_d}")
  if(EXISTS "${_p}")
    if(NOT EXISTS "${_p}/__init__.py")
      file(WRITE "${_p}/__init__.py" "# ${_d} package\n")
    endif()
  endif()
endforeach()

