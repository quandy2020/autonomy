# FindAutolink.cmake - Find the Autolink library (autonomy project).
#
# Usage:
#   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#   find_package(Autolink REQUIRED)
#   target_link_libraries(your_target PRIVATE Autolink::autolink)
#
# Optional:
#   - AUTOLINK_ROOT: root of autolink source tree (e.g. autonomy/src/autolink)
#   - AUTOLINK_BUILD_DIR: autolink build dir where libautolink is (default: ${AUTOLINK_ROOT}/build)

include(FindPackageHandleStandardArgs)

# 仅使用 /usr/local 下的 autolink（或通过 AUTOLINK_ROOT 指定），不使用同级源码目录
set(_AUTOLINK_SEARCH_ROOTS "")
if(DEFINED ENV{AUTOLINK_ROOT})
  list(APPEND _AUTOLINK_SEARCH_ROOTS "$ENV{AUTOLINK_ROOT}")
endif()
if(AUTOLINK_ROOT)
  list(APPEND _AUTOLINK_SEARCH_ROOTS "${AUTOLINK_ROOT}")
endif()
if(NOT _AUTOLINK_SEARCH_ROOTS)
  # 默认仅使用安装前缀 /usr/local
  if(EXISTS "/usr/local/include/autolink/autolink.hpp")
    list(APPEND _AUTOLINK_SEARCH_ROOTS "/usr/local/include")
  endif()
endif()

set(AUTOLINK_FOUND FALSE)
set(AUTOLINK_INCLUDE_DIRS "")
set(AUTOLINK_LIBRARIES "")

foreach(_ROOT IN LISTS _AUTOLINK_SEARCH_ROOTS)
  if(NOT _ROOT OR NOT EXISTS "${_ROOT}")
    continue()
  endif()
  set(_INC "${_ROOT}")
  if(EXISTS "${_INC}/autolink/autolink.hpp")
    set(AUTOLINK_INCLUDE_DIRS "${_INC}")
    set(_AUTOLINK_ROOT "${_ROOT}")
    break()
  endif()
endforeach()

if(AUTOLINK_INCLUDE_DIRS)
  set(_BUILD_DIR "${AUTOLINK_BUILD_DIR}")
  if(NOT _BUILD_DIR)
    # 安装到 /usr/local 时头文件在 include，库在 lib
    if(_AUTOLINK_ROOT MATCHES "/include$")
      get_filename_component(_PARENT "${_AUTOLINK_ROOT}" DIRECTORY)
      set(_BUILD_DIR "${_PARENT}/lib")
    else()
      set(_BUILD_DIR "${_AUTOLINK_ROOT}/build/lib")
    endif()
  endif()
  find_library(AUTOLINK_LIBRARY
    NAMES autolink
    PATHS "${_BUILD_DIR}" "${_BUILD_DIR}/.."
    NO_DEFAULT_PATH
  )
  if(AUTOLINK_LIBRARY)
    set(AUTOLINK_LIBRARIES "${AUTOLINK_LIBRARY}")
    set(AUTOLINK_FOUND TRUE)
  endif()
endif()

find_package_handle_standard_args(Autolink
  REQUIRED_VARS AUTOLINK_INCLUDE_DIRS AUTOLINK_LIBRARIES
  FOUND_VAR AUTOLINK_FOUND
)

if(Autolink_FOUND)
  set(_AUTOLINK_INTERFACE_INCLUDES "${AUTOLINK_INCLUDE_DIRS}")
  # /usr/local 的 autolink 头文件会拉取 fastrtps/fastcdr，需把 ROS 下 FastDDS 相关 include 传给消费者
  unset(_FastRTPS_HEADER_DIR CACHE)
  unset(_FastCDR_HEADER_DIR CACHE)
  find_path(_FastRTPS_HEADER_DIR "fastrtps/attributes/PublisherAttributes.h"
    PATHS "/opt/ros/humble/include/fastrtps" "/opt/ros/jazzy/include/fastrtps" "/opt/ros/iron/include/fastrtps"
          "/opt/ros/humble/include" "/opt/ros/jazzy/include" "/opt/ros/iron/include"
          "/usr/local/include" "/usr/include"
  )
  find_path(_FastCDR_HEADER_DIR "fastcdr/Cdr.h"
    PATHS "/opt/ros/humble/include/fastcdr" "/opt/ros/jazzy/include/fastcdr" "/opt/ros/iron/include/fastcdr"
          "/opt/ros/humble/include" "/opt/ros/jazzy/include" "/opt/ros/iron/include"
          "/usr/local/include" "/usr/include"
  )
  if(_FastRTPS_HEADER_DIR)
    list(APPEND _AUTOLINK_INTERFACE_INCLUDES "${_FastRTPS_HEADER_DIR}")
  endif()
  if(_FastCDR_HEADER_DIR)
    list(APPEND _AUTOLINK_INTERFACE_INCLUDES "${_FastCDR_HEADER_DIR}")
  endif()
  if(NOT TARGET Autolink::autolink)
    add_library(Autolink::autolink SHARED IMPORTED)
  endif()
  set_target_properties(Autolink::autolink PROPERTIES
    IMPORTED_LOCATION "${AUTOLINK_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${_AUTOLINK_INTERFACE_INCLUDES}"
  )
  unset(_AUTOLINK_INTERFACE_INCLUDES)
endif()

unset(_AUTOLINK_SEARCH_ROOTS)
unset(_INC)
unset(_ROOT)
unset(_BUILD_DIR)
unset(_PARENT)
