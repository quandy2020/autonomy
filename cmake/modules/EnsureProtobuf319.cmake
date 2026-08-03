# Pin protobuf to 3.19.x installed under /usr/local.
# gRPC, system packages, or PyTorch (Docker sets CMAKE_PREFIX_PATH to torch)
# may expose incompatible protobuf headers that must not be mixed with protoc 3.19.

if(AUTONOMY_PROTOBUF319_CONFIGURED)
  return()
endif()
set(AUTONOMY_PROTOBUF319_CONFIGURED TRUE)

set(_AUTONOMY_PROTOBUF_PREFIX "/usr/local")

# Docker NVIDIA images export CMAKE_PREFIX_PATH=<venv>/site-packages/torch, which
# makes FindProtobuf pick torch/include while protoc stays at /usr/local/bin.
function(_autonomy_sanitize_cmake_prefix_path)
  foreach(_var IN ITEMS CMAKE_PREFIX_PATH CMAKE_APPLE_FRAMEWORK_PREFIX)
    if(NOT ${_var})
      continue()
    endif()
    set(_filtered "")
    foreach(_entry IN LISTS ${_var})
      if(_entry MATCHES "(^|/)(torch|site-packages)(/|$)")
        continue()
      endif()
      list(APPEND _filtered "${_entry}")
    endforeach()
    set(${_var} "${_filtered}" PARENT_SCOPE)
  endforeach()
endfunction()

_autonomy_sanitize_cmake_prefix_path()
list(PREPEND CMAKE_PREFIX_PATH "${_AUTONOMY_PROTOBUF_PREFIX}")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE STRING
  "Semicolon-separated list of prefixes" FORCE)

set(Protobuf_ROOT "${_AUTONOMY_PROTOBUF_PREFIX}" CACHE PATH "Protobuf 3.19 install prefix" FORCE)
set(Protobuf_DIR "${_AUTONOMY_PROTOBUF_PREFIX}/lib/cmake/protobuf" CACHE PATH
  "Protobuf CMake package directory" FORCE)
unset(_AUTONOMY_PROTOC CACHE)
unset(Protobuf_PROTOC_EXECUTABLE CACHE)
unset(Protobuf_INCLUDE_DIR CACHE)
unset(Protobuf_INCLUDE_DIRS CACHE)

find_program(_AUTONOMY_PROTOC NAMES protoc protoc-3.19.4.0
  PATHS "${_AUTONOMY_PROTOBUF_PREFIX}/bin"
  NO_DEFAULT_PATH
  NO_CMAKE_PATH
  NO_CMAKE_ENVIRONMENT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
)

find_package(Protobuf REQUIRED PATHS "${_AUTONOMY_PROTOBUF_PREFIX}" NO_DEFAULT_PATH)

if(Protobuf_VERSION VERSION_LESS "3.19.0"
    OR Protobuf_VERSION VERSION_GREATER_EQUAL "3.20.0")
  message(FATAL_ERROR
    "Protobuf 3.19.x is required (found ${Protobuf_VERSION}). "
    "Run: bash src/autonomy/docker/install/install_protobuf.sh")
endif()

if(NOT _AUTONOMY_PROTOC)
  message(FATAL_ERROR
    "protoc not found under ${_AUTONOMY_PROTOBUF_PREFIX}/bin. "
    "Run: bash src/autonomy/docker/install/install_protobuf.sh")
endif()

set(Protobuf_PROTOC_EXECUTABLE "${_AUTONOMY_PROTOC}" CACHE FILEPATH "protoc executable" FORCE)

execute_process(
  COMMAND "${Protobuf_PROTOC_EXECUTABLE}" --version
  OUTPUT_VARIABLE _protoc_version_out
  ERROR_VARIABLE _protoc_version_err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
set(_protoc_version "${_protoc_version_out}${_protoc_version_err}")
if(NOT _protoc_version MATCHES "3\\.19\\.")
  message(FATAL_ERROR
    "protoc 3.19.x is required (got '${_protoc_version}' from "
    "${Protobuf_PROTOC_EXECUTABLE}). "
    "Run: bash src/autonomy/docker/install/install_protobuf.sh")
endif()

# Never allow torch/pip include trees to leak in after FindProtobuf runs again.
set(_autonomy_protobuf_include "${_AUTONOMY_PROTOBUF_PREFIX}/include")
set(Protobuf_INCLUDE_DIR "${_autonomy_protobuf_include}" CACHE PATH "" FORCE)
set(Protobuf_INCLUDE_DIRS "${_autonomy_protobuf_include}" CACHE PATH "" FORCE)
set(PROTOBUF_INCLUDE_DIR "${_autonomy_protobuf_include}" CACHE PATH "" FORCE)

if(TARGET protobuf::libprotobuf)
  set_property(TARGET protobuf::libprotobuf PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${_autonomy_protobuf_include}")
endif()
if(TARGET protobuf::libprotoc)
  set_property(TARGET protobuf::libprotoc PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${_autonomy_protobuf_include}")
endif()

message(STATUS "Using Protobuf ${Protobuf_VERSION} includes: ${Protobuf_INCLUDE_DIRS}")
message(STATUS "Using protoc: ${Protobuf_PROTOC_EXECUTABLE} (${_protoc_version})")

# Config-mode Protobuf does not always populate legacy variables used by
# autonomy/autolink CMakeLists (Protobuf_LIBRARIES / PROTOBUF_LIBRARY).
if(TARGET protobuf::libprotobuf)
  set(Protobuf_LIBRARIES protobuf::libprotobuf)
  set(PROTOBUF_LIBRARY protobuf::libprotobuf)
elseif(NOT Protobuf_LIBRARIES AND PROTOBUF_LIBRARY)
  set(Protobuf_LIBRARIES "${PROTOBUF_LIBRARY}")
elseif(Protobuf_LIBRARIES AND NOT PROTOBUF_LIBRARY)
  set(PROTOBUF_LIBRARY "${Protobuf_LIBRARIES}")
endif()

if(NOT Protobuf_LIBRARIES AND NOT TARGET protobuf::libprotobuf)
  message(FATAL_ERROR
    "Protobuf library target not found after find_package. "
    "Run: bash src/autonomy/docker/install/install_protobuf.sh")
endif()

# Call from subprojects instead of find_package(Protobuf) so torch cannot
# overwrite include dirs while /usr/local/bin/protoc remains pinned.
function(autonomy_require_protobuf)
  if(NOT AUTONOMY_PROTOBUF319_CONFIGURED)
    include(EnsureProtobuf319)
  endif()
endfunction()
