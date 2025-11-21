
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

#include "autonomy/common/param_handler.hpp"

namespace autonomy {
namespace common {

ParamHandler::ParamHandler(const std::string& file_name) 
{
    try {
        config_ = YAML::LoadFile(file_name);
        fileLoaded = true;
    } catch (std::exception& e) {
        fileLoaded = false;
    }
}

ParamHandler::~ParamHandler() {}

bool ParamHandler::GetString(const std::string& key, std::string& str_value) 
{
    try {
        str_value = config_[key].as<std::string>();
    } catch (std::exception &e) {
        return false;
    }
    return true;
}
bool ParamHandler::GetString(const std::string& category, const std::string& key, std::string& str_value) 
{
    try {
        str_value = config_[category][key].as<std::string>();
    } catch(std::exception &e) {
        return false;
    }
    return true;
}

bool ParamHandler::GetBoolean(const std::string& key, bool& bool_value) 
{
    try {
        bool_value = config_[key].as<bool>();
    } catch (std::exception &e) {
        return false;
    }
    return true;
}

bool ParamHandler::GetInteger(const std::string& key, int& int_value) 
{
    try {
        int_value = config_[key].as<int>();
    } catch (std::exception &e) {
        return false;
    }
    return true;
}

bool ParamHandler::GetDouble(const std::string& key, double& double_value) 
{
    try {
        double_value = config_[key].as<double>();
    } catch (std::exception &e) {
        return false;
    }
    return true;
}

}  // namespace common
}  // namespace autonomy
