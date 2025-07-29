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
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "autonomy/sensor/collator_interface.hpp"
#include "autonomy/sensor/data.hpp"
#include "autonomy/sensor/internal/ordered_multi_queue.hpp"


namespace autonomy {
namespace sensor {

class Collator : public CollatorInterface 
{
public:
    Collator() {}

    Collator(const Collator&) = delete;
    Collator& operator=(const Collator&) = delete;

    // void AddToCostmap(
    //     const absl::flat_hash_set<std::string>& expected_sensor_ids,
    //     const Callback& callback) override;

    void FinishCostmap() override;

    void AddSensorData(std::unique_ptr<Data> data) override;

    void Flush() override;

 private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
//   OrderedMultiQueue queue_;

//   // Map of trajectory ID to all associated QueueKeys.
//   absl::flat_hash_set<std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace autonomy