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

#ifndef ATLAS_SENSOR_COLLATOR_INTERFACE_HPP_
#define ATLAS_SENSOR_COLLATOR_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

#include "sensor/data.hpp"

namespace atlas {
namespace sensor {

class CollatorInterface {
 public:
  using Callback =
      std::function<void(const std::string&, std::unique_ptr<Data>)>;

  CollatorInterface() = default;
  virtual ~CollatorInterface() = default;
  CollatorInterface(const CollatorInterface&) = delete;
  CollatorInterface& operator=(const CollatorInterface&) = delete;

  virtual void AddTrajectory(
      int trajectory_id,
      const std::unordered_set<std::string>& expected_sensor_ids,
      const Callback& callback) = 0;
  virtual void FinishTrajectory(int trajectory_id) = 0;
  virtual void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) = 0;
  virtual void Flush() = 0;
  virtual std::optional<int> GetBlockingTrajectoryId() const = 0;
};

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_COLLATOR_INTERFACE_HPP_
