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

/**
 * @file configuration_file_resolver.hpp
 * @brief Search ordered directories for configuration files
 * (Cartographer-style).
 */

#ifndef ARGUS_COMMON_CONFIGURATION_FILE_RESOLVER_HPP_
#define ARGUS_COMMON_CONFIGURATION_FILE_RESOLVER_HPP_

#include <string>
#include <vector>

namespace argus {
namespace common {

/**
 * @brief Resolve basenames against a list of directories.
 *
 * Search order: constructor directories, then DefaultConfigurationDirectory()
 * if non-empty.
 */
class ConfigurationFileResolver
{
public:
    /**
     * @brief Build resolver with ordered search paths.
     * @param configuration_files_directories Directories tried first-to-last.
     */
    explicit ConfigurationFileResolver(
        std::vector<std::string> configuration_files_directories);

    /**
     * @brief Absolute path, or empty if not found.
     * @param basename Configuration file name (not a full path).
     * @return Resolved absolute path, or empty string.
     */
    std::string GetFullPath(const std::string& basename) const;

    /**
     * @brief Like GetFullPath; logs and returns empty on failure.
     * @param basename Configuration file name.
     * @return Resolved path, or empty after logging on failure.
     */
    std::string GetFullPathOrDie(const std::string& basename) const;

    /**
     * @brief File contents; empty string if missing / unreadable.
     * @param basename Configuration file name.
     * @return Entire file contents, or empty on error.
     */
    std::string GetFileContent(const std::string& basename) const;

    /**
     * @brief Load file or abort via logging when missing.
     * @param basename Configuration file name.
     * @return File contents; empty only after fatal log path.
     */
    std::string GetFileContentOrDie(const std::string& basename) const;

    /** @return Search directories including defaults. */
    const std::vector<std::string>& directories() const {
        return configuration_files_directories_;
    }

private:
    std::vector<std::string> configuration_files_directories_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_CONFIGURATION_FILE_RESOLVER_HPP_
