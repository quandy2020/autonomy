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

#pragma once 

#include "absl/memory/memory.h"

#include "autonomy/common/time.hpp"

namespace autonomy {

namespace map {
namespace common {
    class MapInterface;
}   // namespace common
}   // namespace map 

namespace sensor {

class Data 
{
public:
    explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
    virtual ~Data() {}

    virtual Time GetTime() const = 0;
    const std::string &GetSensorId() const { return sensor_id_; }
    virtual void AddToCostmap(map::common::MapInterface *costmap_builder) = 0;

protected:
    const std::string sensor_id_;
};

}  // namespace sensor
}  // namespace autonomy