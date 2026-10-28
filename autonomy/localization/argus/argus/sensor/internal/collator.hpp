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

#ifndef ARGUS_SENSOR_INTERNAL_COLLATOR_HPP_
#define ARGUS_SENSOR_INTERNAL_COLLATOR_HPP_

#include <unordered_map>
#include <vector>

#include "sensor/collator_interface.hpp"
#include "sensor/internal/ordered_multi_queue.hpp"

namespace argus {
namespace sensor {

class Collator : public CollatorInterface {
 public:
  Collator() = default;
  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<std::string>& expected_sensor_ids,
                     const Callback& callback) override;
  void FinishTrajectory(int trajectory_id) override;
  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;
  void Flush() override;
  std::optional<int> GetBlockingTrajectoryId() const override;

 private:
  OrderedMultiQueue queue_;
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_INTERNAL_COLLATOR_HPP_
