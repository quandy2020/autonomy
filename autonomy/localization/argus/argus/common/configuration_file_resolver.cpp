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

#include "common/configuration_file_resolver.hpp"

#include <fstream>
#include <utility>

#include "common/config.hpp"
#include "common/logging.hpp"

namespace argus {
namespace common {

ConfigurationFileResolver::ConfigurationFileResolver(
    std::vector<std::string> configuration_files_directories)
    : configuration_files_directories_(
          std::move(configuration_files_directories)) {
  if (!DefaultConfigurationDirectory().empty()) {
    configuration_files_directories_.push_back(DefaultConfigurationDirectory());
  }
}

std::string ConfigurationFileResolver::GetFullPath(
    const std::string& basename) const {
  if (basename.empty()) {
    return {};
  }
  // Absolute / existing path as-is.
  {
    std::ifstream stream(basename.c_str());
    if (stream.good()) {
      return basename;
    }
  }
  for (const auto& dir : configuration_files_directories_) {
    if (dir.empty()) {
      continue;
    }
    const std::string filename = dir + "/" + basename;
    std::ifstream stream(filename.c_str());
    if (stream.good()) {
      return filename;
    }
  }
  return {};
}

std::string ConfigurationFileResolver::GetFullPathOrDie(
    const std::string& basename) const {
  const std::string path = GetFullPath(basename);
  if (path.empty()) {
    ARGUS_ERROR << "Config file '" << basename << "' was not found.";
  } else {
    ARGUS_INFO << "Found '" << path << "' for '" << basename << "'.";
  }
  return path;
}

std::string ConfigurationFileResolver::GetFileContent(
    const std::string& basename) const {
  const std::string filename = GetFullPath(basename);
  if (filename.empty()) {
    return {};
  }
  std::ifstream stream(filename.c_str());
  if (!stream) {
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
}

std::string ConfigurationFileResolver::GetFileContentOrDie(
    const std::string& basename) const {
  if (basename.empty()) {
    ARGUS_ERROR << "File basename cannot be empty.";
    return {};
  }
  const std::string filename = GetFullPathOrDie(basename);
  if (filename.empty()) {
    return {};
  }
  std::ifstream stream(filename.c_str());
  if (!stream) {
    ARGUS_ERROR << "Cannot read '" << filename << "'.";
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
}

}  // namespace common
}  // namespace argus
