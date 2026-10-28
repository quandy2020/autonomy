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
 * @file yaml_parameter.hpp
 * @brief OpenCV FileStorage wrapper for nested YAML / JSON parameters.
 */

#ifndef ARGUS_COMMON_YAML_PARAMETER_HPP_
#define ARGUS_COMMON_YAML_PARAMETER_HPP_

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace argus {
namespace common {

/**
 * @brief Read nested configuration keys using `'/'` as a path separator.
 * Example key: `"vio/grid_size"`. Backed by `cv::FileStorage` (YAML or JSON).
 */
class YamlParameter {
 public:
  /**
   * @brief Construct an unopened parameter store.
   * Call @ref Open before reading keys.
   */
  YamlParameter() = default;

  /**
   * @brief Construct and immediately open @p path.
   * @param[in] path Filesystem path passed to @ref Open.
   */
  explicit YamlParameter(const std::string& path);

  /**
   * @brief Open or reopen a configuration file.
   * Closes any previously opened storage first.
   * @param[in] path Path to a YAML/JSON file readable by OpenCV FileStorage.
   * @return `true` if the storage opened successfully; `false` otherwise
   *         (an error is logged).
   */
  bool Open(const std::string& path);

  /**
   * @brief Whether the underlying FileStorage is currently open.
   * @return `true` if open; `false` otherwise.
   */
  bool IsOpen() const { return storage_.isOpened(); }

  /**
   * @brief Close the FileStorage and clear @ref path.
   */
  void Close();

  /**
   * @brief Path last successfully passed to @ref Open (may be empty).
   * @return Const reference to the stored path string.
   */
  const std::string& path() const { return path_; }

  /**
   * @brief Read a nested key, or return @p default_value if missing.
   * @tparam T Type supported by OpenCV `FileNode` extraction (`operator>>`).
   * @param[in] key            Slash-separated key, e.g. `"vio/grid_size"`.
   * @param[in] default_value  Value returned when the key is absent or empty.
   * @return Parsed value on success; otherwise @p default_value.
   */
  template <typename T>
  T Get(const std::string& key, const T& default_value) const {
    cv::FileNode node = Resolve(key);
    if (node.empty()) {
      return default_value;
    }
    T value = default_value;
    node >> value;
    return value;
  }

  /**
   * @brief Read a required nested key into an output argument.
   * @tparam T Type supported by OpenCV `FileNode` extraction.
   * @param[in]  key Slash-separated key.
   * @param[out] out Destination; must be non-null.
   * @return `true` if @p out was written; `false` if @p out is null or the
   *         key is missing / empty.
   */
  template <typename T>
  bool GetRequired(const std::string& key, T* out) const {
    if (out == nullptr) {
      return false;
    }
    cv::FileNode node = Resolve(key);
    if (node.empty()) {
      return false;
    }
    node >> *out;
    return true;
  }

  /**
   * @brief Read a sequence (or single scalar) of doubles.
   * @param[in] key Slash-separated key.
   * @return Vector of doubles. Empty if the key is missing. A scalar node
   *         yields a one-element vector; a sequence yields one entry per
   *         element (non-numeric elements are skipped).
   */
  std::vector<double> GetDoubleVector(const std::string& key) const;

  /**
   * @brief Test whether a nested key resolves to a non-empty node.
   * @param[in] key Slash-separated key.
   * @return `true` if the node exists and is non-empty; `false` otherwise.
   */
  bool Has(const std::string& key) const;

 private:
  /**
   * @brief Walk slash-separated segments from the FileStorage root.
   * @param[in] key Nested key path.
   * @return Resolved `cv::FileNode` (empty on failure).
   */
  cv::FileNode Resolve(const std::string& key) const;

  std::string path_;
  cv::FileStorage storage_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_YAML_PARAMETER_HPP_
