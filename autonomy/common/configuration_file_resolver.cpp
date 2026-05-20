/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/configuration_file_resolver.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "autonomy/common/config.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace common {

ConfigurationFileResolver::ConfigurationFileResolver(
    const std::vector<std::string>& configuration_files_directories)
    : configuration_files_directories_(configuration_files_directories) {
    configuration_files_directories_.push_back(kConfigurationFilesDirectory);
}

std::string ConfigurationFileResolver::GetFullPathOrDie(
    const std::string& basename) {
    for (const auto& path : configuration_files_directories_) {
        const std::string filename = path + "/" + basename;
        std::ifstream stream(filename.c_str());
        if (stream.good()) {
            LOG(INFO) << "Found '" << filename << "' for '" << basename << "'.";
            return filename;
        }
    }
    std::ostringstream searched;
    searched << "File '" << basename << "' was not found. Searched:";
    for (const auto& path : configuration_files_directories_) {
        searched << "\n  " << path << "/" << basename;
    }
    LOG(FATAL) << searched.str();
}

std::vector<std::string> ConfigurationSearchDirectories(
    const std::string& user_configuration_directory) {
    std::vector<std::string> directories;
    if (!user_configuration_directory.empty()) {
        directories.push_back(user_configuration_directory);
    }
    directories.emplace_back(std::string(kSourceDirectory) + "/config");
    return directories;
}

std::string ResolveConfigurationRootDirectory(
    const std::string& user_configuration_directory,
    const std::string& probe_relative_path) {
    ConfigurationFileResolver resolver(
        ConfigurationSearchDirectories(user_configuration_directory));
    const std::string full_path =
        resolver.GetFullPathOrDie(probe_relative_path);
    const std::string suffix = "/" + probe_relative_path;
    CHECK_GE(full_path.size(), suffix.size());
    CHECK_EQ(full_path.compare(full_path.size() - suffix.size(),
                               suffix.size(), suffix),
             0)
        << full_path << " does not end with " << suffix;
    return full_path.substr(0, full_path.size() - suffix.size());
}

std::string ConfigurationFileResolver::GetFileContentOrDie(
    const std::string& basename) {
    CHECK(!basename.empty()) << "File basename cannot be empty." << basename;
    const std::string filename = GetFullPathOrDie(basename);
    std::ifstream stream(filename.c_str());
    return std::string((std::istreambuf_iterator<char>(stream)),
                       std::istreambuf_iterator<char>());
}

}  // namespace common
}  // namespace autonomy
