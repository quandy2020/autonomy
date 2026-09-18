
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

#include <glog/logging.h>
#include <stdexcept>

namespace autonomy {
namespace common {

ParamHandler::ParamHandler(const std::string& file_name) {
    try {
        config_ = YAML::LoadFile(file_name);
        fileLoaded = true;
    } catch (std::exception& e) {
        fileLoaded = false;
    }
}

ParamHandler::ParamHandler(const YAML::Node& node)
    : config_(node), fileLoaded(true) {}

ParamHandler::~ParamHandler() {}

bool ParamHandler::GetString(const std::string& key, std::string& str_value) {
    return GetValue(key, str_value);
}

bool ParamHandler::GetString(const std::string& category,
                             const std::string& key, std::string& str_value) {
    return GetValue(category, key, str_value);
}

bool ParamHandler::GetBoolean(const std::string& key, bool& bool_value) {
    return GetValue(key, bool_value);
}

bool ParamHandler::GetInteger(const std::string& key, int& int_value) {
    return GetValue(key, int_value);
}

bool ParamHandler::GetDouble(const std::string& key, double& double_value) {
    return GetValue(key, double_value);
}

// ========== LuaParameterDictionary-style interface implementation ==========

bool ParamHandler::HasKey(const std::string& key) const {
    return config_[key].IsDefined();
}

std::string ParamHandler::GetString(const std::string& key) {
    CHECK(HasKey(key)) << "Key '" << key << "' not found in YAML dictionary";
    std::string value;
    CHECK(GetValue(key, value))
        << "Failed to get string value for key '" << key << "'";
    return value;
}

double ParamHandler::GetDouble(const std::string& key) {
    CHECK(HasKey(key)) << "Key '" << key << "' not found in YAML dictionary";
    double value;
    CHECK(GetValue(key, value))
        << "Failed to get double value for key '" << key << "'";
    return value;
}

int ParamHandler::GetInt(const std::string& key) {
    CHECK(HasKey(key)) << "Key '" << key << "' not found in YAML dictionary";
    int value;
    CHECK(GetValue(key, value))
        << "Failed to get int value for key '" << key << "'";
    return value;
}

bool ParamHandler::GetBool(const std::string& key) {
    CHECK(HasKey(key)) << "Key '" << key << "' not found in YAML dictionary";
    bool value;
    CHECK(GetValue(key, value))
        << "Failed to get bool value for key '" << key << "'";
    return value;
}

std::unique_ptr<ParamHandler> ParamHandler::GetDictionary(
    const std::string& key) {
    CHECK(HasKey(key)) << "Key '" << key << "' not found in YAML dictionary";
    CHECK(config_[key].IsMap())
        << "Key '" << key << "' is not a map/dictionary";
    return std::make_unique<ParamHandler>(config_[key]);
}

int ParamHandler::GetNonNegativeInt(const std::string& key) {
    int value = GetInt(key);
    CHECK_GE(value, 0) << "Value for key '" << key
                       << "' must be non-negative, got " << value;
    return value;
}

YAML::Node ParamHandler::GetChild(const std::string& key) const {
    if (!config_[key].IsDefined()) {
        return YAML::Node();
    }
    return config_[key];
}

std::vector<std::vector<float>> ParamHandler::ParseRectangles(
    const YAML::Node& node) {
    auto rectangles =
        node.as<std::vector<std::vector<float>>>(std::vector<std::vector<float>>());
    for (const auto& v : rectangles) {
        if (v.size() != 4) {
            throw std::runtime_error("mask rectangle must contain four parameters");
        }
        if (v.at(0) >= v.at(1)) {
            throw std::runtime_error("x_max must be greater than x_min");
        }
        if (v.at(2) >= v.at(3)) {
            throw std::runtime_error("y_max must be greater than y_min");
        }
    }
    return rectangles;
}

}  // namespace common
}  // namespace autonomy
