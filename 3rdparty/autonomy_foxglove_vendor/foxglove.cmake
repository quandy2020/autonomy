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

macro(build_foxglove)

  if (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(FOXGLOVE_SDK_RELEASES "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.11.0/foxglove-v0.11.0-cpp-aarch64-unknown-linux-gnu.zip" "28807d1a02d4afe613afe1229b298ca1c48ee1d881c496686d93da08897b6a25")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(FOXGLOVE_SDK_RELEASES "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.11.0/foxglove-v0.11.0-cpp-x86_64-unknown-linux-gnu.zip" "da778da2de6688ba3a7ebcafb1813bfafa7cbc7a931d49f9fa90c77c538ca3a0")
  else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}")
  endif()

  list(GET FOXGLOVE_SDK_RELEASES 0 url)
  list(GET FOXGLOVE_SDK_RELEASES 1 hash)

  # Fetch Foxglove SDK
  include(FetchContent)
  FetchContent_Declare(
      foxglove
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
      # See available releases and builds here: https://github.com/foxglove/foxglove-sdk/releases?q=sdk%2F&expanded=true
      URL ${url}
      URL_HASH SHA256=${hash}
  )
  FetchContent_MakeAvailable(foxglove)
endmacro()

build_foxglove()
