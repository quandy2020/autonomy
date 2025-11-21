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

#include "yaml-cpp/yaml.h"

#include "autonomy/common/port.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"

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
     * @brief Constructor
     * 
     * @param file_name The parameter file name
     */
    ParamHandler(const std::string& file_name);

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
    template<typename T>
    bool GetVector(const std::string& key, std::vector<T>& vec_value) 
    {
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
    template<typename T>
    bool GetVector(const std::string& category, const std::string& key, std::vector<T>& vec_value) 
    {
        try {
            vec_value = config_[category][key].as<std::vector<T> >();
        } catch (std::exception& e) {
            return false;
        }
        return true;
    }

    template<typename T>
    bool Get2DArray(const std::string& category, const std::string& key, std::vector<std::vector<T> >& vec_value) 
    {
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
     * @param T_value The value
     * @return true If the parameter is found
     * @return false If the parameter is not found
     */
    template<typename T>
    bool GetValue(const std::string& key, T& T_value) 
    {
        try {
            T_value = config_[key].as<T>();
        } catch (std::exception& e) {
            return false;
        }
        return true;
    }

    template<typename T>
    bool GetValue(const std::string& category, const std::string& key, T& T_value) {
        try {
            T_value = config_[category][key].as<T>();
            return true;
        } catch (std::exception &e) {
            return false;
        }
        return true;
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
    bool GetBoolean(const std::string& category, const std::string& key, bool& bool_value) 
    {
        try {
            bool_value = config_[category][key].as<bool>();
            return true;
        } catch (std::exception& e) {
            return false;
        }
        return true;
    }

    /**
     * @brief Get keys
     * 
     * @return std::vector<std::string> The keys
     */
    std::vector<std::string> GetKeys() {
        std::vector<std::string> v;
        v.reserve(config_.size());
        for(auto it = config_.begin(); it != config_.end(); it++) {
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
    bool GetInteger(const std::string &key, int &int_value);

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
    bool FileOpenedSuccessfully() 
    {
        return fileLoaded;
    }

    /**
     * @brief Get the YAML config node
     * @return The YAML config node
     */
    YAML::Node GetConfig() const
    {
        return config_;
    }

protected:
    YAML::Node config_;

private:
    bool fileLoaded = false;
};

}  // namespace common
}  // namespace autonomy