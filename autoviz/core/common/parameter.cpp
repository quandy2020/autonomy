/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "parameter.hpp"

namespace foxglove_ws {

ParameterValue::ParameterValue()
    : _type(ParameterType::PARAMETER_NOT_SET) {}
ParameterValue::ParameterValue(bool value)
    : _type(ParameterType::PARAMETER_BOOL)
    , _value(value) {}
ParameterValue::ParameterValue(int value)
    : _type(ParameterType::PARAMETER_INTEGER)
    , _value(static_cast<int64_t>(value)) {}
ParameterValue::ParameterValue(int64_t value)
    : _type(ParameterType::PARAMETER_INTEGER)
    , _value(value) {}
ParameterValue::ParameterValue(double value)
    : _type(ParameterType::PARAMETER_DOUBLE)
    , _value(value) {}
ParameterValue::ParameterValue(const std::string& value)
    : _type(ParameterType::PARAMETER_STRING)
    , _value(value) {}
ParameterValue::ParameterValue(const char* value)
    : _type(ParameterType::PARAMETER_STRING)
    , _value(std::string(value)) {}
ParameterValue::ParameterValue(const std::vector<unsigned char>& value)
    : _type(ParameterType::PARAMETER_BYTE_ARRAY)
    , _value(value) {}
ParameterValue::ParameterValue(const std::vector<ParameterValue>& value)
    : _type(ParameterType::PARAMETER_ARRAY)
    , _value(value) {}
ParameterValue::ParameterValue(const std::unordered_map<std::string, ParameterValue>& value)
    : _type(ParameterType::PARAMETER_STRUCT)
    , _value(value) {}

Parameter::Parameter() {}
Parameter::Parameter(const std::string& name)
    : _name(name)
    , _value(ParameterValue()) {}
Parameter::Parameter(const std::string& name, const ParameterValue& value)
    : _name(name)
    , _value(value) {}

}  // namespace foxglove_ws
