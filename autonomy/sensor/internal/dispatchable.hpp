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

#include <functional>

#include "autonomy/sensor/data.hpp"

namespace autonomy {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data 
{
public:
    Dispatchable(const std::string &sensor_id, const DataType &data)
        : Data(sensor_id), data_(data) {}

    Time GetTime() const override { return data_.time; }

    void AddToCostmap(map::common::MapInterface *costmap_builder) override 
    {
        // costmap_builder->AddSensorData(sensor_id_, data_);
    }
    const DataType &data() const { return data_; }

private:
    const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace autonomy