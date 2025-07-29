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

#include "autonomy/sensor/internal/collator.hpp"

namespace autonomy {
namespace sensor {

// void Collator::AddToCostmap(
//     const absl::flat_hash_set<std::string>& expected_sensor_ids,
//     const Callback& callback) 
// {
// //   for (const auto& sensor_id : expected_sensor_ids) {
// //     const auto queue_key = QueueKey{sensor_id};
// //     queue_.AddQueue(queue_key,
// //                     [callback, sensor_id](std::unique_ptr<Data> data) {
// //                       callback(sensor_id, std::move(data));
// //                     });
// //     queue_keys_.push_back(queue_key);
// //   }
// }

void Collator::FinishCostmap() 
{
//   for (const auto& queue_key : queue_keys_) {
//     queue_.MarkQueueAsFinished(queue_key);
//   }
}

void Collator::AddSensorData(std::unique_ptr<Data> data) 
{
//   QueueKey queue_key{data->GetSensorId()};
//   queue_.Add(std::move(queue_key), std::move(data));
}

void Collator::Flush() 
{ 
    // queue_.Flush(); 
}

}  // namespace sensor
}  // namespace autonomy