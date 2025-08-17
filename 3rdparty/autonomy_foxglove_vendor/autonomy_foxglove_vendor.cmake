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
  # Fetch Foxglove SDK
  include(FetchContent)
  FetchContent_Declare(
      foxglove
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
      # See available releases and builds here: https://github.com/foxglove/foxglove-sdk/releases?q=sdk%2F&expanded=true
      URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.11.0/foxglove-v0.11.0-cpp-aarch64-unknown-linux-gnu.zip
      URL_HASH SHA256=28807d1a02d4afe613afe1229b298ca1c48ee1d881c496686d93da08897b6a25
  )
  FetchContent_MakeAvailable(foxglove)
endmacro()

build_foxglove()
