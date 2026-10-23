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
 * @brief Resolve configuration basenames against an ordered directory list
 *        (Cartographer-style).
 */

#ifndef ATLAS_COMMON_CONFIGURATION_FILE_RESOLVER_HPP_
#define ATLAS_COMMON_CONFIGURATION_FILE_RESOLVER_HPP_

#include <string>
#include <vector>

namespace atlas {
namespace common {

/**
 * @brief Locate configuration files by basename across ordered search paths.
 * Search order:
 *   1. Directories supplied to the constructor (first to last);
 *   2. @ref DefaultConfigurationDirectory if non-empty.
 * Absolute paths that already exist are returned as-is by @ref GetFullPath.
 */
class ConfigurationFileResolver {
 public:
  /**
   * @brief Construct with an ordered list of search directories.
   * @param[in] configuration_files_directories Directories tried in order.
   *            The default configuration directory (if set) is appended.
   */
  explicit ConfigurationFileResolver(
      std::vector<std::string> configuration_files_directories);

  /**
   * @brief Resolve @p basename to an absolute filesystem path.
   * @param[in] basename File name or absolute path (not required to exist
   *                     when absolute — existence is checked).
   * @return Absolute path of the first match, or an empty string if not
   *         found.
   */
  std::string GetFullPath(const std::string& basename) const;

  /**
   * @brief Like @ref GetFullPath, with diagnostic logging.
   * @param[in] basename Configuration file name to resolve.
   * @return Resolved path on success; empty string after logging on failure.
   */
  std::string GetFullPathOrDie(const std::string& basename) const;

  /**
   * @brief Read the entire contents of the file identified by @p basename.
   * @param[in] basename Configuration file name to resolve and read.
   * @return File bytes as a string, or empty if the file is missing or
   *         unreadable.
   */
  std::string GetFileContent(const std::string& basename) const;

  /**
   * @brief Like @ref GetFileContent, with diagnostic logging on failure.
   * @param[in] basename Configuration file name to resolve and read.
   * @return File contents on success; empty string after logging on failure
   *         (including empty @p basename).
   */
  std::string GetFileContentOrDie(const std::string& basename) const;

  /**
   * @brief Directories currently used for resolution (including defaults).
   * @return Const reference to the internal ordered directory list.
   */
  const std::vector<std::string>& directories() const {
    return configuration_files_directories_;
  }

 private:
  std::vector<std::string> configuration_files_directories_;
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_CONFIGURATION_FILE_RESOLVER_HPP_
