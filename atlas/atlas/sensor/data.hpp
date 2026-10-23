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

#ifndef ATLAS_SENSOR_DATA_HPP_
#define ATLAS_SENSOR_DATA_HPP_

#include <string>

namespace atlas {
namespace sensor {

class Data {
 public:
  explicit Data(std::string sensor_id) : sensor_id_(std::move(sensor_id)) {}
  virtual ~Data() = default;

  virtual double GetTime() const = 0;
  const std::string& GetSensorId() const { return sensor_id_; }

 protected:
  const std::string sensor_id_;
};

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_DATA_HPP_
