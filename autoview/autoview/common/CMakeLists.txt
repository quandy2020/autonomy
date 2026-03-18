# Copyright 2025 The Openbot Authors (duyongquan)
# Common library: config, properties, visualizer_app, visualization_manager, interaction, display, etc.

cmake_minimum_required(VERSION 3.16)

set(COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Autolink QUIET)

# 包含 common 下所有 .cpp / .hpp，与目录结构保持同步
file(GLOB_RECURSE COMMON_SOURCES "*.cpp")
file(GLOB_RECURSE COMMON_HEADERS "*.hpp")

# 未找到 Autolink 时不编译依赖 Autolink 的 interaction 实现
if(NOT Autolink_FOUND)
  list(REMOVE_ITEM COMMON_SOURCES
    "interaction/node_abstraction.cpp"
    "interaction/client_abstraction.cpp"
  )
endif()

add_library(autoview_common STATIC ${COMMON_SOURCES} ${COMMON_HEADERS})

# Include root (parent of autoview/) so that #include "autoview/common/..." resolves.
target_include_directories(autoview_common PUBLIC
  "${COMMON_DIR}/../.."
  ${COMMON_DIR}
)
if(Autolink_FOUND)
  target_include_directories(autoview_common PRIVATE ${AUTOLINK_INCLUDE_DIRS})
  target_link_libraries(autoview_common PRIVATE Autolink::autolink)
endif()
target_link_libraries(autoview_common PUBLIC Qt::Core Qt::Widgets)
set_target_properties(autoview_common PROPERTIES
  CXX_STANDARD 17
  POSITION_INDEPENDENT_CODE ON
)

if(TARGET autoview)
  target_link_libraries(autoview PRIVATE autoview_common)
endif()
