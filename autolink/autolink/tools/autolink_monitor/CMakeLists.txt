# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# C++ 工具：autolink_monitor（对应 Autolink tools monitor，依赖 ncurses）
file(GLOB AUTOLINK_MONITOR_SOURCES "*.cpp")
add_executable(autolink_monitor ${AUTOLINK_MONITOR_SOURCES})
target_include_directories(autolink_monitor PRIVATE BEFORE ${CMAKE_SOURCE_DIR})
target_link_libraries(autolink_monitor PRIVATE autolink)
find_package(Curses REQUIRED)
if(TARGET Curses::Curses)
  target_link_libraries(autolink_monitor PRIVATE Curses::Curses)
else()
  target_include_directories(autolink_monitor PRIVATE ${CURSES_INCLUDE_DIR})
  target_link_libraries(autolink_monitor PRIVATE ${CURSES_LIBRARIES})
endif()

install(TARGETS autolink_monitor
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
