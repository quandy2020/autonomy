# Install autosim Python package when BUILD_AUTOSIM=ON.
# Runtime still requires autolink + automsgs Python from the same workspace build.
#
# setuptools always writes *.egg-info next to pyproject.toml. Stage a copy
# under the CMake build directory so the source tree stays clean.

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/autosim/pyproject.toml")
  message(FATAL_ERROR
    "autosim is missing at ${CMAKE_CURRENT_SOURCE_DIR}/autosim\n"
    "Expected pyproject.toml for pip install.")
endif()

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(_autosim_root "${CMAKE_CURRENT_SOURCE_DIR}/autosim")
set(_autosim_stage "${CMAKE_CURRENT_BINARY_DIR}/autosim_pip")

add_custom_target(autosim_pip_install ALL
  COMMAND ${CMAKE_COMMAND} -E rm -rf
          "${_autosim_stage}"
          "${_autosim_root}/autosim.egg-info"
          "${_autosim_root}/UNKNOWN.egg-info"
          "${_autosim_root}/build"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${_autosim_stage}"
  COMMAND ${CMAKE_COMMAND} -E copy
          "${_autosim_root}/pyproject.toml"
          "${_autosim_stage}/pyproject.toml"
  COMMAND ${CMAKE_COMMAND} -E copy
          "${_autosim_root}/README.md"
          "${_autosim_stage}/README.md"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${_autosim_root}/autosim"
          "${_autosim_stage}/autosim"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${_autosim_root}/config"
          "${_autosim_stage}/config"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${_autosim_root}/urdf"
          "${_autosim_stage}/urdf"
  COMMAND ${Python3_EXECUTABLE} -m pip install "${_autosim_stage}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Installing autosim Python package (pip, staged)"
  VERBATIM)

if(TARGET automsgs)
  add_dependencies(autosim_pip_install automsgs)
endif()
if(TARGET autolink)
  add_dependencies(autosim_pip_install autolink)
endif()

message(STATUS "autosim: pip install (staged) ${_autosim_root}")
