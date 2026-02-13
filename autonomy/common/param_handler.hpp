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

#pragma once

#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "yaml-cpp/yaml.h"

namespace autonomy {
namespace common {

class ParamHandler
{
public:
    /**
     * Define ParamHandler::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ParamHandler)

    /**
     * @brief Constructor from file path
     *
     * @param file_name The parameter file name
     */
    ParamHandler(const std::string& file_name);

    /**
     * @brief Constructor from YAML node
     *
     * @param node The YAML node to wrap
     */
    explicit ParamHandler(const YAML::Node& node);

    /**
     * @brief Destructor
     */
    virtual ~ParamHandler();

    /**
     * @brief Get string value
     *
     * @param key The parameter key
     * @param str_value The string value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    bool GetString(const std::string& key, std::string& str_value);

    /**
     * @brief Get string value with category
     *
     * @param category The parameter category
     * @param key The parameter key
     * @param str_value The string value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    bool GetString(const std::string& category, const std::string& key, std::string& str_value);

    /**
     * @brief Get vector value
     *
     * @param key The parameter key
     * @param vec_value The vector value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    template <typename T>
    bool GetVector(const std::string& key, std::vector<T>& vec_value) {
        try {
            vec_value = config_[key].as<std::vector<T> >();
        } catch (std::exception& e) {
            return false;
        }
        return true;
    }

    /**
     * @brief Get vector value with category
     *
     * @param category The parameter category
     * @param key The parameter key
     * @param vec_value The vector value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    template <typename T>
    bool GetVector(const std::string& category, const std::string& key, std::vector<T>& vec_value) {
        try {
            vec_value = config_[category][key].as<std::vector<T> >();
        } catch (std::exception& e) {
            return false;
        }
        return true;
    }

    template <typename T>
    bool Get2DArray(const std::string& category, const std::string& key, std::vector<std::vector<T> >& vec_value) {
        try {
            vec_value = config_[category][key].as<std::vector<std::vector<T> > >();
        } catch (std::exception& e) {
            return false;
        }
        return true;
    }

    /**
     * @brief Get value
     *
     * @param key The parameter key
     * @param value The value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    template <typename T>
    bool GetValue(const std::string& key, T& value) {
        try {
            value = config_[key].as<T>();
            return true;
        } catch (std::exception& e) {
            return false;
        }
    }

    /**
     * @brief Get value with category
     *
     * @param category The parameter category
     * @param key The parameter key
     * @param value The value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    template <typename T>
    bool GetValue(const std::string& category, const std::string& key, T& value) {
        try {
            value = config_[category][key].as<T>();
            return true;
        } catch (std::exception& e) {
            return false;
        }
    }

    /**
     * @brief Get boolean value with category
     *
     * @param category The parameter category
     * @param key The parameter key
     * @param bool_value The boolean value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    bool GetBoolean(const std::string& category, const std::string& key, bool& bool_value) {
        return GetValue(category, key, bool_value);
    }

    /**
     * @brief Get keys
     *
     * @return std::vector<std::string> The keys
     */
    std::vector<std::string> GetKeys() const {
        std::vector<std::string> v;
        if (!config_.IsMap()) {
            return v;
        }
        v.reserve(config_.size());
        for (auto it = config_.begin(); it != config_.end(); it++) {
            v.push_back(it->first.as<std::string>());
        }
        return v;
    }

    /**
     * @brief Get boolean value
     *
     * @param key The parameter key
     * @param bool_value The boolean value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    bool GetBoolean(const std::string& key, bool& bool_value);

    /**
     * @brief Get integer value
     *
     * @param key The parameter key
     * @param int_value The integer value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    bool GetInteger(const std::string& key, int& int_value);

    /**
     * @brief Get double value
     *
     * @param key The parameter key
     * @param double_value The double value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    bool GetDouble(const std::string& key, double& double_value);

    /**
     * @brief Check if the file is opened successfully
     *
     * @return true If the file is opened successfully
     * @return false If the file is not opened successfully
     */
    bool FileOpenedSuccessfully() {
        return fileLoaded;
    }

    /**
     * @brief Get the YAML config node
     * @return The YAML config node
     */
    YAML::Node GetConfig() const {
        return config_;
    }

    // ========== LuaParameterDictionary-style interface (with CHECK) ==========
    // These methods provide compatibility with code that expects
    // LuaParameterDictionary interface

    /**
     * @brief Returns true if the key exists
     */
    bool HasKey(const std::string& key) const;

    /**
     * @brief Get string value (CHECKs if key doesn't exist)
     * @return The string value
     */
    std::string GetString(const std::string& key);

    /**
     * @brief Get double value (CHECKs if key doesn't exist)
     * @return The double value
     */
    double GetDouble(const std::string& key);

    /**
     * @brief Get int value (CHECKs if key doesn't exist)
     * @return The int value
     */
    int GetInt(const std::string& key);

    /**
     * @brief Get bool value (CHECKs if key doesn't exist)
     * @return The bool value
     */
    bool GetBool(const std::string& key);

    /**
     * @brief Get nested dictionary (CHECKs if key doesn't exist or is not a
     * map)
     * @return A new ParamHandler instance for the nested dictionary
     */
    std::unique_ptr<ParamHandler> GetDictionary(const std::string& key);

    /**
     * @brief Get int and CHECK that it is non-negative
     * @return The non-negative int value
     */
    int GetNonNegativeInt(const std::string& key);

protected:
    YAML::Node config_;

private:
    bool fileLoaded = false;
};

}  // namespace common
}  // namespace autonomy