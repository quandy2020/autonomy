/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTODRIVER_SAMPLE_SINK_HPP_
#define AUTODRIVER_SAMPLE_SINK_HPP_

#include <memory>

#include "autodriver/config.hpp"
#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

// Receives attach/detach and samples. Autolink publishing lives in bridge.
class SampleSink {
public:
    virtual ~SampleSink() = default;

    virtual bool OnAttach(const Config::Sensor& sensor, SensorType type) = 0;
    virtual void OnDetach(const SensorId& id) = 0;
    virtual void OnSample(std::shared_ptr<SensorSample> sample) = 0;

protected:
    SampleSink() = default;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SAMPLE_SINK_HPP_
