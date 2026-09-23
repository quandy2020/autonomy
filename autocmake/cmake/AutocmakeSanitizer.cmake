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

# @brief Add sanitizer flags for the current directory and below.
#
# @param ADDRESS Enable AddressSanitizer.
# @param UNDEFINED Enable UndefinedBehaviorSanitizer.
# @param THREAD Enable ThreadSanitizer. Not combined with ADDRESS.
function(autocmake_sanitize)
  cmake_parse_arguments(ARG "ADDRESS;UNDEFINED;THREAD" "" "" ${ARGN})
  if(NOT ARG_ADDRESS AND NOT ARG_UNDEFINED AND NOT ARG_THREAD)
    set(ARG_ADDRESS TRUE)
    set(ARG_UNDEFINED TRUE)
  endif()
  if(ARG_THREAD AND (ARG_ADDRESS OR ARG_UNDEFINED))
    message(FATAL_ERROR "autocmake_sanitize: THREAD cannot be combined with ADDRESS or UNDEFINED")
  endif()
  if(MSVC)
    message(STATUS "autocmake_sanitize: skipped on MSVC")
    return()
  endif()

  set(_san "")
  if(ARG_ADDRESS)
    list(APPEND _san address)
  endif()
  if(ARG_UNDEFINED)
    list(APPEND _san undefined)
  endif()
  if(ARG_THREAD)
    list(APPEND _san thread)
  endif()
  list(JOIN _san "," _san)
  add_compile_options(-fsanitize=${_san} -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${_san})
endfunction()
