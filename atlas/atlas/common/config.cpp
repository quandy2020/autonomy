/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common/config.hpp"

namespace atlas {
namespace common {
namespace {

std::string& DefaultDirectoryStorage() {
  static std::string dir;
  return dir;
}

}  // namespace

const std::string& DefaultConfigurationDirectory() {
  return DefaultDirectoryStorage();
}

void SetDefaultConfigurationDirectory(std::string path) {
  DefaultDirectoryStorage() = std::move(path);
}

}  // namespace common
}  // namespace atlas
