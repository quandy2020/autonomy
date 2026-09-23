# Copyright 2026 The Openbot Authors (duyongquan)
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

include_guard(GLOBAL)

# @brief Default build options: library type, warnings, hidden symbols, sanitizers.
macro(_autocmake_options)
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
  option(AUTOCMAKE_HIDE_SYMBOLS "Hide symbols unless explicitly exported" OFF)
  option(AUTOCMAKE_SANITIZER "Build with address and undefined-behavior sanitizers" OFF)

  if(AUTOCMAKE_HIDE_SYMBOLS)
    set(CMAKE_C_VISIBILITY_PRESET hidden)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
  endif()

  get_property(_autocmake_languages GLOBAL PROPERTY ENABLED_LANGUAGES)
  if(_autocmake_languages)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang" OR CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    elseif(MSVC)
      add_compile_options(/W4)
    endif()
    if(AUTOCMAKE_SANITIZER)
      autocmake_sanitize(ADDRESS UNDEFINED)
    endif()
  endif()
endmacro()
