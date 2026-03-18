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

# C++ 工具：autolink_launch（对应 Autolink tools launch 命令）
# 需显式链接 TinyXML2（main.cpp 直接使用 tinyxml2）
add_executable(autolink_launch main.cpp)
target_include_directories(autolink_launch PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(autolink_launch PRIVATE autolink)
  # 显式链接 TinyXML2（与 autolink 一致，避免 ld 未解析符号）
if(TARGET TinyXML2::TinyXML2)
  target_link_libraries(autolink_launch PRIVATE TinyXML2::TinyXML2)
else()
  if(NOT TinyXML2_LIB)
    find_library(TinyXML2_LIB NAMES tinyxml2)
  endif()
  if(TinyXML2_LIB)
    target_link_libraries(autolink_launch PRIVATE ${TinyXML2_LIB})
  endif()
endif()

install(TARGETS autolink_launch
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
