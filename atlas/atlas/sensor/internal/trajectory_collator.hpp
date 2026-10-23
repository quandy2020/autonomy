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

#ifndef ATLAS_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_HPP_
#define ATLAS_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_HPP_

#include <unordered_map>
#include <vector>

#include "common/logging.hpp"
#include "sensor/collator_interface.hpp"
#include "sensor/internal/ordered_multi_queue.hpp"

namespace atlas {
namespace sensor {

// Sorts one trajectory at a time. Unlike Collator, it does not wait for other
// trajectories before dispatching.
class TrajectoryCollator : public CollatorInterface {
 public:
  TrajectoryCollator() = default;
  TrajectoryCollator(const TrajectoryCollator&) = delete;
  TrajectoryCollator& operator=(const TrajectoryCollator&) = delete;

  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<std::string>& expected_sensor_ids,
                     const Callback& callback) override;
  void FinishTrajectory(int trajectory_id) override;
  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;
  void Flush() override;
  std::optional<int> GetBlockingTrajectoryId() const override;

 private:
  std::unordered_map<int, OrderedMultiQueue> trajectory_to_queue_;
  std::unordered_map<int, std::vector<QueueKey>> trajectory_to_queue_keys_;
};

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_HPP_
