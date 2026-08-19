/*
 * Copyright 2026 Autodriver contributors
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

#ifndef AUTODRIVER_ALIGNED_SNAPSHOT_HPP_
#define AUTODRIVER_ALIGNED_SNAPSHOT_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {

struct AlignedSnapshot {
    autolink::Time time;
    std::unordered_map<SensorId, std::shared_ptr<SensorSample>> samples;

    template <typename SampleT>
    const SampleT* Get(const SensorId& id) const {
        const auto it = samples.find(id);
        if (it == samples.end() || !it->second) {
            return nullptr;
        }
        return dynamic_cast<const SampleT*>(it->second.get());
    }

    template <typename SampleT>
    std::vector<const SampleT*> GetAll(SensorType type) const {
        std::vector<const SampleT*> out;
        for (const auto& entry : samples) {
            if (!entry.second || entry.second->type() != type) {
                continue;
            }
            if (const auto* data =
                    dynamic_cast<const SampleT*>(entry.second.get())) {
                out.push_back(data);
            }
        }
        return out;
    }
};

}  // namespace autodriver

#endif  // AUTODRIVER_ALIGNED_SNAPSHOT_HPP_
