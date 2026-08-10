# Resolve Protobuf for Autonomy / automsgs / autoviz.
#
# Linux Docker (default): pin to Protobuf 3.19.x under /usr/local so gRPC /
# torch CMAKE_PREFIX_PATH cannot mix headers with protoc.
#
# macOS / Homebrew (and other hosts without the pin): use the system or brew
# ProtobufConfig.cmake (e.g. /opt/homebrew) without requiring 3.19.x.

if(AUTONOMY_PROTOBUF319_CONFIGURED)
  return()
endif()
set(AUTONOMY_PROTOBUF319_CONFIGURED TRUE)

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

function(_autonomy_protobuf_has_config prefix)
  if(EXISTS "${prefix}/lib/cmake/protobuf/protobuf-config.cmake"
      OR EXISTS "${prefix}/lib/cmake/protobuf/ProtobufConfig.cmake")
    set(_autonomy_protobuf_has_config_result TRUE PARENT_SCOPE)
  else()
    set(_autonomy_protobuf_has_config_result FALSE PARENT_SCOPE)
  endif()
endfunction()

_autonomy_sanitize_cmake_prefix_path()

# --- Choose install prefix ---------------------------------------------------
set(_AUTONOMY_PROTOBUF_PREFIX "")
set(_AUTONOMY_PROTOBUF_PIN_319 FALSE)

if(DEFINED AUTONOMY_PROTOBUF_PREFIX AND AUTONOMY_PROTOBUF_PREFIX)
  set(_AUTONOMY_PROTOBUF_PREFIX "${AUTONOMY_PROTOBUF_PREFIX}")
elseif(DEFINED ENV{AUTONOMY_PROTOBUF_PREFIX} AND NOT "$ENV{AUTONOMY_PROTOBUF_PREFIX}" STREQUAL "")
  set(_AUTONOMY_PROTOBUF_PREFIX "$ENV{AUTONOMY_PROTOBUF_PREFIX}")
endif()

# Prefer pinned 3.19 under /usr/local when present (Linux Docker / install_protobuf.sh).
if(NOT _AUTONOMY_PROTOBUF_PREFIX)
  _autonomy_protobuf_has_config("/usr/local")
  if(_autonomy_protobuf_has_config_result AND EXISTS "/usr/local/bin/protoc")
    execute_process(
      COMMAND "/usr/local/bin/protoc" --version
      OUTPUT_VARIABLE _pin_protoc_out
      ERROR_VARIABLE _pin_protoc_err
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
    )
    set(_pin_protoc_ver "${_pin_protoc_out}${_pin_protoc_err}")
    if(_pin_protoc_ver MATCHES "3\\.19\\.")
      set(_AUTONOMY_PROTOBUF_PREFIX "/usr/local")
      set(_AUTONOMY_PROTOBUF_PIN_319 TRUE)
    endif()
  endif()
endif()

# macOS Homebrew (Apple Silicon then Intel keg)
if(NOT _AUTONOMY_PROTOBUF_PREFIX)
  foreach(_brew_prefix IN ITEMS "/opt/homebrew" "/usr/local")
    _autonomy_protobuf_has_config("${_brew_prefix}")
    if(_autonomy_protobuf_has_config_result)
      set(_AUTONOMY_PROTOBUF_PREFIX "${_brew_prefix}")
      break()
    endif()
    _autonomy_protobuf_has_config("${_brew_prefix}/opt/protobuf")
    if(_autonomy_protobuf_has_config_result)
      set(_AUTONOMY_PROTOBUF_PREFIX "${_brew_prefix}/opt/protobuf")
      break()
    endif()
  endforeach()
endif()

if(NOT _AUTONOMY_PROTOBUF_PREFIX)
  # Last resort: let CMake search the default paths (no pin).
  set(_AUTONOMY_PROTOBUF_PIN_319 FALSE)
endif()

if(_AUTONOMY_PROTOBUF_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${_AUTONOMY_PROTOBUF_PREFIX}")
  set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE STRING
    "Semicolon-separated list of prefixes" FORCE)
  set(Protobuf_ROOT "${_AUTONOMY_PROTOBUF_PREFIX}" CACHE PATH
    "Protobuf install prefix" FORCE)
  set(Protobuf_DIR "${_AUTONOMY_PROTOBUF_PREFIX}/lib/cmake/protobuf" CACHE PATH
    "Protobuf CMake package directory" FORCE)
else()
  # Clear a stale FORCE from a previous configure that pointed at missing /usr/local.
  if(Protobuf_DIR AND NOT EXISTS "${Protobuf_DIR}/protobuf-config.cmake"
      AND NOT EXISTS "${Protobuf_DIR}/ProtobufConfig.cmake")
    unset(Protobuf_DIR CACHE)
  endif()
endif()

unset(_AUTONOMY_PROTOC CACHE)
unset(Protobuf_PROTOC_EXECUTABLE CACHE)
unset(Protobuf_INCLUDE_DIR CACHE)
unset(Protobuf_INCLUDE_DIRS CACHE)

if(_AUTONOMY_PROTOBUF_PIN_319)
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

  set(Protobuf_PROTOC_EXECUTABLE "${_AUTONOMY_PROTOC}" CACHE FILEPATH
    "protoc executable" FORCE)

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
else()
  # Flexible: Homebrew / system Protobuf (macOS, or hosts without 3.19 pin).
  if(_AUTONOMY_PROTOBUF_PREFIX)
    find_program(_AUTONOMY_PROTOC NAMES protoc
      PATHS "${_AUTONOMY_PROTOBUF_PREFIX}/bin"
      NO_DEFAULT_PATH)
    find_package(Protobuf REQUIRED CONFIG
      PATHS "${_AUTONOMY_PROTOBUF_PREFIX}"
      NO_DEFAULT_PATH)
  else()
    find_package(Protobuf REQUIRED)
    find_program(_AUTONOMY_PROTOC NAMES protoc)
  endif()

  if(_AUTONOMY_PROTOC)
    set(Protobuf_PROTOC_EXECUTABLE "${_AUTONOMY_PROTOC}" CACHE FILEPATH
      "protoc executable" FORCE)
  elseif(TARGET protobuf::protoc)
    get_target_property(_protoc_loc protobuf::protoc IMPORTED_LOCATION)
    if(NOT _protoc_loc)
      get_target_property(_protoc_loc protobuf::protoc
        IMPORTED_LOCATION_RELEASE)
    endif()
    if(_protoc_loc)
      set(Protobuf_PROTOC_EXECUTABLE "${_protoc_loc}" CACHE FILEPATH
        "protoc executable" FORCE)
    endif()
  endif()

  if(NOT Protobuf_PROTOC_EXECUTABLE)
    message(FATAL_ERROR
      "protoc not found. On macOS: brew install protobuf\n"
      "Or set AUTONOMY_PROTOBUF_PREFIX to the install prefix.")
  endif()

  execute_process(
    COMMAND "${Protobuf_PROTOC_EXECUTABLE}" --version
    OUTPUT_VARIABLE _protoc_version_out
    ERROR_VARIABLE _protoc_version_err
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )
  set(_protoc_version "${_protoc_version_out}${_protoc_version_err}")

  if(_AUTONOMY_PROTOBUF_PREFIX AND EXISTS "${_AUTONOMY_PROTOBUF_PREFIX}/include")
    set(_autonomy_protobuf_include "${_AUTONOMY_PROTOBUF_PREFIX}/include")
    set(Protobuf_INCLUDE_DIR "${_autonomy_protobuf_include}" CACHE PATH "" FORCE)
    set(Protobuf_INCLUDE_DIRS "${_autonomy_protobuf_include}" CACHE PATH "" FORCE)
    set(PROTOBUF_INCLUDE_DIR "${_autonomy_protobuf_include}" CACHE PATH "" FORCE)
    # Homebrew/Config packages sometimes omit usable INTERFACE includes for
    # consumers that only compile OBJECT libraries.
    if(TARGET protobuf::libprotobuf)
      set_property(TARGET protobuf::libprotobuf PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES "${_autonomy_protobuf_include}")
    endif()
    if(TARGET protobuf::libprotoc)
      set_property(TARGET protobuf::libprotoc PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES "${_autonomy_protobuf_include}")
    endif()
  endif()

  message(STATUS
    "Protobuf pin 3.19.x not required on this host "
    "(using ${_AUTONOMY_PROTOBUF_PREFIX})")
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
    "On macOS: brew install protobuf\n"
    "On Linux Docker: bash src/autonomy/docker/install/install_protobuf.sh")
endif()

# Call from subprojects instead of find_package(Protobuf) so torch cannot
# overwrite include dirs while /usr/local/bin/protoc remains pinned.
function(autonomy_require_protobuf)
  if(NOT AUTONOMY_PROTOBUF319_CONFIGURED)
    include(EnsureProtobuf319)
  endif()
endfunction()
