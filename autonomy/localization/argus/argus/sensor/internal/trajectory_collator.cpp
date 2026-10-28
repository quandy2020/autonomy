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

#include "sensor/internal/trajectory_collator.hpp"

namespace argus {
namespace sensor {

void TrajectoryCollator::AddTrajectory(
    int trajectory_id,
    const std::unordered_set<std::string>& expected_sensor_ids,
    const Callback& callback) {
  ARGUS_CHECK_EQ(trajectory_to_queue_.count(trajectory_id), 0u);
  for (const std::string& sensor_id : expected_sensor_ids) {
    const QueueKey queue_key{trajectory_id, sensor_id};
    trajectory_to_queue_[trajectory_id].AddQueue(
        queue_key, [callback, sensor_id](std::unique_ptr<Data> data) {
          callback(sensor_id, std::move(data));
        });
    trajectory_to_queue_keys_[trajectory_id].push_back(queue_key);
  }
}

void TrajectoryCollator::FinishTrajectory(int trajectory_id) {
  for (const QueueKey& queue_key : trajectory_to_queue_keys_[trajectory_id]) {
    trajectory_to_queue_.at(trajectory_id).MarkQueueAsFinished(queue_key);
  }
}

void TrajectoryCollator::AddSensorData(int trajectory_id,
                                       std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  trajectory_to_queue_.at(trajectory_id).Add(queue_key, std::move(data));
}

void TrajectoryCollator::Flush() {
  for (auto& entry : trajectory_to_queue_) {
    entry.second.Flush();
  }
}

std::optional<int> TrajectoryCollator::GetBlockingTrajectoryId() const {
  return std::nullopt;
}

}  // namespace sensor
}  // namespace argus
