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

#ifndef ARGUS_SENSOR_INTERNAL_DISPATCHABLE_HPP_
#define ARGUS_SENSOR_INTERNAL_DISPATCHABLE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "sensor/data.hpp"

namespace argus {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data {
 public:
  Dispatchable(std::string sensor_id, DataType data)
      : Data(std::move(sensor_id)), data_(std::move(data)) {}

  double GetTime() const override { return data_.time; }
  const DataType& data() const { return data_; }

 private:
  const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string& sensor_id, const DataType& data) {
  return std::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_INTERNAL_DISPATCHABLE_HPP_
