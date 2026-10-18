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
 * @brief Thin OpenCV FileStorage wrapper for FAST-LIVO2-style YAML configs.
 */

#ifndef ARGUS_COMMON_YAML_PARAMETER_HPP_
#define ARGUS_COMMON_YAML_PARAMETER_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace argus {
namespace common {

/**
 * @brief Read nested keys with '/' separators, e.g. `"vio/grid_size"`.
 */
class YamlParameter
{
public:
    YamlParameter() = default;

    /**
     * @brief Open YAML/JSON via OpenCV FileStorage.
     * @param path File path passed to @ref Open.
     */
    explicit YamlParameter(const std::string& path);

    /**
     * @brief Open or reopen a configuration file.
     * @param path Path to YAML/JSON readable by OpenCV FileStorage.
     * @return True if storage opened successfully.
     */
    bool Open(const std::string& path);

    /** @return True if FileStorage is open. */
    bool IsOpen() const {
        return storage_.isOpened();
    }

    /** @brief Close the underlying FileStorage. */
    void Close();

    /** @return Path last passed to @ref Open. */
    const std::string& path() const {
        return path_;
    }

    /**
     * @brief Read a nested key or return default if missing.
     * @tparam T Type supported by OpenCV FileNode streaming.
     * @param key Slash-separated key, e.g. `"vio/grid_size"`.
     * @param default_value Value returned when the key is absent.
     * @return Parsed value or @p default_value.
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
     * @brief Read a required nested key into @p out.
     * @tparam T Type supported by OpenCV FileNode streaming.
     * @param key Slash-separated key.
     * @param out Destination (must be non-null).
     * @return False if @p out is null or the key is missing.
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
     * @brief Read a sequence of doubles (OpenCV FileNode sequence).
     * @param key Slash-separated key.
     * @return Vector of doubles (empty if key missing or not a sequence).
     */
    std::vector<double> GetDoubleVector(const std::string& key) const;

    /**
     * @brief Check whether a nested key exists.
     * @param key Slash-separated key.
     * @return True if the resolved node is non-empty.
     */
    bool Has(const std::string& key) const;

private:
    cv::FileNode Resolve(const std::string& key) const;

    std::string path_;
    cv::FileStorage storage_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_YAML_PARAMETER_HPP_
